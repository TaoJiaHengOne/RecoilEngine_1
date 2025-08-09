/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#include "IPathFinder.h"
#include "PathFinderDef.h"
#include "PathLog.h"
#include "Sim/Path/HAPFS/PathGlobal.h"
#include "Sim/MoveTypes/MoveDefHandler.h"
#include "System/Log/ILog.h"
#include "System/TimeProfiler.h"

#include "System/Misc/TracyDefs.h"

// #include "PathGlobal.h"
// #include "System/Threading/ThreadPool.h"
// #include "Game/SelectedUnitsHandler.h"
// #include "Sim/Objects/SolidObject.h"

// #include <mutex>

namespace HAPFS {

static std::vector<PathNodeStateBuffer> nodeStateBuffers;
static std::vector<IPathFinder*> pathFinderInstances;


void IPathFinder::InitStatic() { pathFinderInstances.reserve(8); }
void IPathFinder::KillStatic() { pathFinderInstances.clear  ( ); }

/**
 * 这个函数的核心目标是初始化一个具体的寻路器实例
 * （无论是高分辨率的 CPathFinder 还是低分辨率的 CPathEstimator）。
 * 它根据传入的分辨率 (_BLOCK_SIZE) 设置该实例的所有基本参数，
 * 并为其分配进行A*搜索所必需的内存和数据结构。
 * _BLOCK_SIZE，用于定义此寻路器实例的工作分辨率
*/
void IPathFinder::Init(unsigned int _BLOCK_SIZE)
{
	{
		RECOIL_DETAILED_TRACY_ZONE;
		// 这是此函数最重要的操作之一，决定了寻路器是在精细网格还是粗糙网格上工作。
		BLOCK_SIZE = _BLOCK_SIZE;
		// 预先计算并存储一个“块”在世界坐标中的像素尺寸，用于后续的坐标转换，提高效率。
		BLOCK_PIXEL_SIZE = BLOCK_SIZE * SQUARE_SIZE;
		// : 根据地图的总宽度 (mapDims.mapx) 和块的大小，计算出寻路网格总共有多少个块。
		nbrOfBlocks.x = mapDims.mapx / BLOCK_SIZE;
		nbrOfBlocks.y = mapDims.mapy / BLOCK_SIZE;
		// 将用于A*搜索的起始块和目标块索引初始化为0。
		mStartBlockIdx = 0;
		mGoalBlockIdx = 0;
		// 将记录最近目标点的启发式函数值（H值）初始化为0.0
		mGoalHeuristic = 0.0f;
		// 将搜索相关的计数器（最大搜索块数、已测试块数）重置为0。
		maxBlocksToBeSearched = 0;
		testedBlocks = 0;
		// 获取当前已创建的寻路器实例总数，并将这个值作为当前实例的唯一索引 
		// instanceIndex。这个索引在后续分配共享资源（如 nodeStateBuffers）时至关重要。
		instanceIndex = pathFinderInstances.size();
	}
	{
		// 清空用于存储开放列表节点的内存缓冲池 openBlockBuffer
		openBlockBuffer.Clear();
		// handled via AllocStateBuffer
		// blockStates.Clear();
		// done in ResetSearch
		// openBlocks.Clear();
		// 清空 dirtyBlocks 向量。这个向量用于记录上一次搜索中被修改过的节点，以便在下一次搜索前能高效地重置它们的状态
		dirtyBlocks.clear();
	}
	{
		// 将当前正在初始化的这个寻路器实例的指针 (this) 添加到一个全局的静态向量 pathFinderInstances 中。这使得系统可以集中管理所有已创建的寻路器实例。
		pathFinderInstances.push_back(this);
	}
	// 调用一个关键的辅助函数，它负责为当前寻路器实例分配或重用一个巨大的节点状态缓冲区 (PathNodeStateBuffer)，
	// 并将其赋值给 blockStates 成员变量。这是为A*搜索准备“草稿纸”的核心步骤。
	AllocStateBuffer();
	// 调用 ResetSearch 函数，执行最后的准备工作。它会清空开放列表优先队列 openBlocks，
	// 并为 dirtyBlocks 向量预留内存，使寻路器完全准备好执行第一次搜索。
	ResetSearch();
}

void IPathFinder::Kill()
{
	RECOIL_DETAILED_TRACY_ZONE;
	// allow our PNSB to be reused across reloads
	if (instanceIndex < nodeStateBuffers.size())
		nodeStateBuffers[instanceIndex] = std::move(blockStates);
}


void IPathFinder::AllocStateBuffer()
{
	RECOIL_DETAILED_TRACY_ZONE;
	// 代码检查当前实例的ID是否超出了缓冲池的当前大小。
	// 如果是（例如，游戏刚启动，这是第一个被初始化的寻路器），emplace_back() 就会在池中创建一个新的、空的 PathNodeStateBuffer。
	// 这确保了每个寻路器实例在池中都有一个对应的存储槽
	if (instanceIndex >= nodeStateBuffers.size())
		nodeStateBuffers.emplace_back();
	// 找到池中属于当前实例的那个 PathNodeStateBuffer，并调用其 Clear() 方法。
	// 这会重置缓冲区内部的状态，确保即使这个缓冲区是从上一局游戏中回收再利用的，也不会有任何残留的旧数据。
	nodeStateBuffers[instanceIndex].Clear();
	// Resize() 方法会确保这个缓冲区的容量足以容纳当前寻路器网格中的所有节点（nbrOfBlocks 是网格的二维尺寸(降采样后的网格)）。
	// 如果缓冲区的容量已经足够，这个操作可能会很快完成；如果不够，它会重新分配内存以满足需求。
	nodeStateBuffers[instanceIndex].Resize(nbrOfBlocks, int2(mapDims.mapx, mapDims.mapy));

	// steal memory, returned in dtor
	blockStates = std::move(nodeStateBuffers[instanceIndex]);
}

/// @brief 的核心作用是高效地清理上一次A*搜索留下的痕迹，为下一次新的搜索做准备。
/// 它通过只重置被“弄脏”的节点，而不是整个地图的状态，来实现极高的执行效率。
void IPathFinder::ResetSearch()
{
	RECOIL_DETAILED_TRACY_ZONE;
	// 循环持续进行，直到 dirtyBlocks 向量变为空。dirtyBlocks 是一个特殊的列表，
	// 它在上一次搜索过程中，记录了所有被访问过（被设为“开放”或“关闭”状态）的节点的索引。
	while (!dirtyBlocks.empty()) {
		// 从 dirtyBlocks 列表的末尾取出一个“脏”节点的索引，并调用 ClearSquare 函数。这个函数会重置该节点在 blockStates 缓冲区中的状态，
		// 例如将其F值和G值设回无穷大，并清除其状态掩码，使其恢复到“未访问”状态。
		blockStates.ClearSquare(dirtyBlocks.back());
		dirtyBlocks.pop_back();
	}

	// reserve a batch of dirty blocks
	dirtyBlocks.reserve(4096);
	openBlocks.Clear();

	testedBlocks = 0;
}

//std::mutex cacheAccessLock;

IPath::SearchResult IPathFinder::GetPath(
	const MoveDef& moveDef,
	const CPathFinderDef& pfDef,
	const CSolidObject* owner,
	float3 startPos,
	IPath::Path& path,
	const unsigned int maxNodes
) {
	RECOIL_DETAILED_TRACY_ZONE;
	startPos.ClampInBounds();

	// clear the path
	path.path.clear();
	path.squares.clear();
	path.pathCost = PATHCOST_INFINITY;

	// initial calculations
	if (BLOCK_SIZE != 1) {
		maxBlocksToBeSearched = std::min(MAX_SEARCHED_NODES_PE - 8U, maxNodes);
	} else {
		maxBlocksToBeSearched = std::min(MAX_SEARCHED_NODES_PF - 8U, maxNodes);
	}

	if (!SetStartBlock(moveDef, pfDef, owner, startPos))
		return IPath::SearchResult::Error;

	assert(static_cast<unsigned int>(mStartBlock.x) < nbrOfBlocks.x);
	assert(static_cast<unsigned int>(mStartBlock.y) < nbrOfBlocks.y);

	// check cache (when there is one)
	int2 goalBlock;
	goalBlock = {int(pfDef.goalSquareX / BLOCK_SIZE), int(pfDef.goalSquareZ / BLOCK_SIZE)};

	// if (gs->frameNum == 2251 && BLOCK_SIZE == 16){
	// 	debugLoggingActive = ThreadPool::GetThreadNum();
	// if (owner != nullptr && selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end()){
	// 	LOG("Starting deeper logging for query: start (%d, %d) -> (%d, %d) (%d -> %d) [%f:%d] [%d] = %d"
	// 			, mStartBlock.x, mStartBlock.y
	// 			, goalBlock.x, goalBlock.y
	// 			, mStartBlockIdx, mGoalBlockIdx
	// 			, pfDef.sqGoalRadius, moveDef.pathType
	// 			, BLOCK_SIZE, debugLoggingActive);
	// }


	const CPathCache::CacheItem& ci = GetCache(mStartBlock, goalBlock, pfDef.sqGoalRadius, moveDef.pathType, pfDef.synced);

	if (ci.pathType != -1) {
		path = ci.path;
		return ci.result;
	}

	// start up a new search
	const IPath::SearchResult result = InitSearch(moveDef, pfDef, owner);

	// if search was successful, generate new path and cache it
	if (result == IPath::Ok || result == IPath::GoalOutOfRange) {
		FinishSearch(moveDef, pfDef, path);

		//if (ci.pathType == -1)
		// When the MT 'Pathing System' is running, it will handle updating the cache separately.
		if (!ThreadPool::inMultiThreadedSection)
			AddCache(&path, result, mStartBlock, goalBlock, pfDef.sqGoalRadius, moveDef.pathType, pfDef.synced);
		// else{
		// 	if (debugLoggingActive == ThreadPool::GetThreadNum()){
		// 	if (ci.path.path.size() != path.path.size())
		// 		LOG("!!!!!! Pathing Difference DETECTED !!!!!!");
		// 	LOG("===== Cache Returned the following =====");
		// 	LOG("Query was: start (%d, %d) -> (%d, %d) [~%f:%d]"
		// 			, ci.strtBlock.x, ci.strtBlock.y
		// 			, ci.goalBlock.x, ci.goalBlock.y
		// 			, ci.goalRadius, ci.pathType);
		// 	LOG("Path Resolution level %d (cost: %f)", BLOCK_SIZE, ci.path.pathCost);
		// 	LOG("Desired Goal (%f, %f, %f)", ci.path.desiredGoal.x, ci.path.desiredGoal.y, ci.path.desiredGoal.z);
		// 	LOG("Path Goal (%f, %f, %f)", ci.path.pathGoal.x, ci.path.pathGoal.y, ci.path.pathGoal.z);

		// 	for (int j = 0; j<ci.path.path.size(); j++){
		// 		LOG("Path Step %d (%f, %f, %f)", j, ci.path.path[j].x, ci.path.path[j].y, ci.path.path[j].z);
		// 	}
		// 	for (int j = 0; j<ci.path.squares.size(); j++){
		// 		LOG("Square Step %d (%d, %d)", j, ci.path.squares[j].x, ci.path.squares[j].y);
		// 	}
		// 	}
		// }
	}
	// 	if (LOG_IS_ENABLED(L_DEBUG)) {
	// 		LOG_L(L_DEBUG, "==== %s: Search completed ====", (BLOCK_SIZE != 1) ? "PE" : "PF");
	// 		LOG_L(L_DEBUG, "Tested blocks: %u", testedBlocks);
	// 		LOG_L(L_DEBUG, "Open blocks: %u", openBlockBuffer.GetSize());
	// 		LOG_L(L_DEBUG, "Path length: " _STPF_, path.path.size());
	// 		LOG_L(L_DEBUG, "Path cost: %f", path.pathCost);
	// 		LOG_L(L_DEBUG, "==============================");
	// 	}
	// } else {
	// 	if (LOG_IS_ENABLED(L_DEBUG)) {
	// 		LOG_L(L_DEBUG, "==== %s: Search failed! ====", (BLOCK_SIZE != 1) ? "PE" : "PF");
	// 		LOG_L(L_DEBUG, "Tested blocks: %u", testedBlocks);
	// 		LOG_L(L_DEBUG, "Open blocks: %u", openBlockBuffer.GetSize());
	// 		LOG_L(L_DEBUG, "============================");
	// 	}
	// }

	// if (debugLoggingActive == ThreadPool::GetThreadNum()){
	// 	LOG("===== Actual result from query attempt =====");
	// 	LOG("Query was: start (%d, %d) -> (%d, %d) [~%f:%d]"
	// 			, mStartBlock.x, mStartBlock.y
	// 			, goalBlock.x, goalBlock.y
	// 			, pfDef.sqGoalRadius, moveDef.pathType);
	// 	LOG("Path Resolution level %d (cost: %f)", BLOCK_SIZE, path.pathCost);
	// 	LOG("Desired Goal (%f, %f, %f)", path.desiredGoal.x, path.desiredGoal.y, path.desiredGoal.z);
	// 	LOG("Path Goal (%f, %f, %f)", path.pathGoal.x, path.pathGoal.y, path.pathGoal.z);

	// 	for (int j = 0; j<path.path.size(); j++){
	// 		LOG("Path Step %d (%f, %f, %f)", j, path.path[j].x, path.path[j].y, path.path[j].z);
	// 	}
	// 	for (int j = 0; j<path.squares.size(); j++){
	// 		LOG("Square Step %d (%d, %d)", j, path.squares[j].x, path.squares[j].y);
	// 	}
	// 	LOG("===== End of comparison =====");
	// }

	// if (mStartBlock.x == 41 && mStartBlock.y == 28
	// 		&& goalBlock.x == 39 && goalBlock.y == 31
	// 		&& moveDef.pathType == 44 && BLOCK_SIZE == 16){
	// if (gs->frameNum == 2251 && BLOCK_SIZE == 16){
	// 	LOG("Deactivate deeper logging.");
	// 	debugLoggingActive = -1;
	// }

	return result;
}


// 初始化A*寻路搜索的核心函数 - 设置起始状态、检查早期退出条件、配置搜索参数并执行实际搜索
IPath::SearchResult IPathFinder::InitSearch(const MoveDef& moveDef, const CPathFinderDef& pfDef, const CSolidObject* owner)
{
	// 启用Tracy性能分析器的详细分析区域，用于监控函数执行性能
	RECOIL_DETAILED_TRACY_ZONE;
	
	// 初始化实际搜索起始方格坐标，默认使用块级起始位置
	int2 square = mStartBlock;

	// 检查是否为低分辨率寻路器（PathEstimator），BLOCK_SIZE > 1 表示多个地图方格组成一个搜索块
	if (BLOCK_SIZE != 1){
		// 如果存在父级共享状态缓冲区（通常用于分层寻路中的高低分辨率配合）
		if (psBlockStates != nullptr)
			// 使用共享状态缓冲区中针对特定移动类型预计算的节点偏移，获得更精确的起始位置
			square = (*psBlockStates).peNodeOffsets[moveDef.pathType][mStartBlockIdx];
		else
			// 使用本地状态缓冲区中的预计算偏移，这些偏移考虑了不同移动类型的特殊需求
			square = blockStates.peNodeOffsets[moveDef.pathType][mStartBlockIdx];
	}

	// 检查起始方格是否已经位于目标区域内，避免不必要的搜索
	const bool isStartGoal = pfDef.IsGoal(square.x, square.y);
	// 获取路径定义中是否允许起始位置在目标半径内的标志
	const bool startInGoal = pfDef.startInGoalRadius;

	// 获取是否允许原始路径搜索的标志（通常指直线路径或简单几何路径）
	const bool allowRawPath = pfDef.allowRawPath;
	// 获取是否允许默认路径搜索的标志（通常指完整的A*搜索）
	const bool allowDefPath = pfDef.allowDefPath;

	// 断言确保至少允许一种搜索方式，防止配置错误导致的死锁
	assert(allowRawPath || allowDefPath);

	// 清理上一次搜索的残留状态：重置脏块列表、清空开放队列、重置计数器
	ResetSearch();

	// 预定义可能的搜索结果数组：[0]=原始搜索失败, [1]=成功, [2]=默认搜索失败
	IPath::SearchResult results[] = {IPath::CantGetCloser, IPath::Ok, IPath::CantGetCloser};

	// 优化检查：如果起始位置已在目标区域且配置允许，直接返回结果避免搜索开销
	// 虽然起始方格在目标半径内，但起始坐标可能在半径外，此时不返回CantGetCloser而是返回到起始方格的路径
	if (isStartGoal && startInGoal)
		// 如果允许原始路径则返回Ok，否则返回CantGetCloser
		return results[allowRawPath];

	// 初始化起始节点的A*算法状态：清除所有状态位但保留过时标记（用于缓存失效检测）
	blockStates.nodeMask[mStartBlockIdx] &= PATHOPT_OBSOLETE;
	// 将起始节点标记为开放状态，表示它在开放列表中待处理
	blockStates.nodeMask[mStartBlockIdx] |= PATHOPT_OPEN;
	// 设置起始节点的F成本为0（A*中的f(n) = g(n) + h(n)，起始点g=0，h稍后计算）
	blockStates.fCost[mStartBlockIdx] = 0.0f;
	// 设置起始节点的G成本为0（从起始点到自身的实际成本为0）
	blockStates.gCost[mStartBlockIdx] = 0.0f;
	// 更新全局F成本的最大值跟踪（用于搜索统计和调试）
	blockStates.SetMaxCost(NODE_COST_F, 0.0f);
	// 更新全局G成本的最大值跟踪（用于搜索统计和调试）
	blockStates.SetMaxCost(NODE_COST_G, 0.0f);

	// 将起始节点索引加入脏块列表，确保搜索结束后能正确重置其状态
	dirtyBlocks.push_back(mStartBlockIdx);

	// 初始化开放节点缓冲区：清空缓冲区准备存储新的搜索节点
	openBlockBuffer.SetSize(0);
	// 从缓冲区获取一个新的路径节点对象用于表示起始节点
	PathNode* ob = openBlockBuffer.GetNode(openBlockBuffer.GetSize());
		// 设置起始节点的F成本为0（总估计成本）
		ob->fCost   = 0.0f;
		// 设置起始节点的G成本为0（实际已走成本）
		ob->gCost   = 0.0f;
		// 存储起始节点的二维坐标位置
		ob->nodePos = mStartBlock;
		// 存储起始节点的一维数组索引，用于快速访问状态数组
		ob->nodeNum = mStartBlockIdx;
		// 检查起始位置是否为"仅出口"区域（某些特殊地形单位只能离开不能进入）
		ob->exitOnly = moveDef.IsInExitOnly(mStartBlock.x, mStartBlock.y);
	// 将配置好的起始节点加入开放列表优先队列，开始A*搜索过程
	openBlocks.push(ob);

	// 计算并存储起始位置到目标的启发式距离估计（A*算法中的h值）
	// 注释掉的旧代码：mGoalHeuristic = pfDef.Heuristic(square.x, square.y, BLOCK_SIZE);
	// 使用多态的启发式函数，不同的寻路器子类可能有不同的启发式计算方法
	mGoalHeuristic = GetHeuristic(moveDef, pfDef, square);

	// 定义搜索类型的枚举常量，提高代码可读性
	enum {
		RAW = 0,    // 原始搜索索引（直线路径检测等快速方法）
		IPF = 1,    // 完整搜索索引（标准A*算法）
	};

	// 执行两阶段搜索策略：首先尝试快速原始搜索，失败后再尝试完整搜索
	// 第一阶段：如果允许原始路径，执行原始搜索（通常是直线可达性检测），否则直接标记为错误
	results[RAW] = (allowRawPath                                )? DoRawSearch(moveDef, pfDef, owner): IPath::Error;
	// 第二阶段：如果允许默认搜索且原始搜索失败，执行完整A*搜索，否则使用原始搜索的结果
	results[IPF] = (allowDefPath && results[RAW] == IPath::Error)? DoSearch(moveDef, pfDef, owner): results[RAW];

	// 优先返回成功的搜索结果：如果完整搜索成功，立即返回成功状态
	if (results[IPF] == IPath::Ok)
		return IPath::Ok;
	// 如果找到了部分路径（目标块不等于起始块），返回部分搜索结果（可能是GoalOutOfRange等）
	if (mGoalBlockIdx != mStartBlockIdx)
		return results[IPF];

	// 处理起始和目标在同一块内但为不同方格的特殊情况：
	// 这种情况下通常无法更接近目标，应返回CantGetCloser，除非调用者仅请求原始搜索
	// 复杂的布尔表达式计算最终返回值：考虑搜索类型限制和目标状态
	return results[IPF + ((!allowRawPath || allowDefPath) && (!isStartGoal || startInGoal))];
}

bool IPathFinder::SetStartBlock(
	const MoveDef& moveDef,
	const CPathFinderDef& peDef,
	const CSolidObject* owner,
	float3 startPos
)
{
	RECOIL_DETAILED_TRACY_ZONE;
	mStartBlock.x  = startPos.x / BLOCK_PIXEL_SIZE;
	mStartBlock.y  = startPos.z / BLOCK_PIXEL_SIZE;
	mStartBlockIdx = BlockPosToIdx(mStartBlock);
	mGoalBlockIdx  = mStartBlockIdx;

	return true;
}

}
