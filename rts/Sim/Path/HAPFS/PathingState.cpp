/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#include "PathingState.h"

#include "zlib.h"
#include "minizip/zip.h"

#include "Game/GlobalUnsynced.h"
#include "Game/LoadScreen.h"
#include "Net/Protocol/NetProtocol.h"

#include "Sim/Misc/ModInfo.h"
#include "Sim/MoveTypes/MoveDefHandler.h"
#include "Sim/MoveTypes/MoveMath/MoveMath.h"
#include "PathFinder.h"
#include "IPath.h"
#include "PathConstants.h"
#include "PathFinderDef.h"
#include "PathLog.h"
#include "Sim/Path/HAPFS/PathGlobal.h"
#include "PathMemPool.h"

#include "System/Config/ConfigHandler.h"
#include "System/FileSystem/Archives/IArchive.h"
#include "System/FileSystem/ArchiveLoader.h"
#include "System/FileSystem/DataDirsAccess.h"
#include "System/FileSystem/FileSystem.h"
#include "System/FileSystem/FileQueryFlags.h"
#include "System/Platform/Threading.h"
#include "System/StringUtil.h"
#include "System/Threading/ThreadPool.h" // for_mt

#include "System/Misc/TracyDefs.h"

#define ENABLE_NETLOG_CHECKSUM 1

static constexpr int BLOCK_UPDATE_DELAY_FRAMES = GAME_SPEED / 2;

namespace HAPFS {

bool TEST_ACTIVE = false;

static std::vector<PathNodeStateBuffer> nodeStateBuffers;
static size_t pathingStates = 0;

PCMemPool pcMemPool;
// PEMemPool peMemPool;

static const std::string GetPathCacheDir() {
	RECOIL_DETAILED_TRACY_ZONE;
	return (FileSystem::GetCacheDir() + FileSystemAbstraction::GetNativePathSeparator() + "paths" + FileSystemAbstraction::GetNativePathSeparator());
}

static const std::string GetCacheFileName(const std::string& fileHashCode, const std::string& peFileName, const std::string& mapFileName) {
	RECOIL_DETAILED_TRACY_ZONE;
	return (GetPathCacheDir() + mapFileName + "." + peFileName + "-" + fileHashCode + ".zip");
}

void PathingState::KillStatic() { pathingStates = 0; }

PathingState::PathingState()
{
	RECOIL_DETAILED_TRACY_ZONE;
	pathCache[0] = nullptr;
	pathCache[1] = nullptr;
}

void PathingState::Init(std::vector<IPathFinder*> pathFinderlist, PathingState* parentState, unsigned int _BLOCK_SIZE, const std::string& peFileName, const std::string& mapFileName)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// 将函数传入的分辨率参数 _BLOCK_SIZE 赋值给当前 PathingState 实例的成员变量 BLOCK_SIZE。
	// 这是最关键的一步，它决定了这个 PathingState 是为低分辨率（例如32x32）还是中分辨率（例如16x16）的路径估算器服务的
	BLOCK_SIZE = _BLOCK_SIZE;
	// 预先计算并存储一个“宏观块”在世界坐标中的边长（以像素或游戏单位“elmos”计）。
	// SQUARE_SIZE 是一个常量，代表一个最精细地图方格的边长。这样做可以避免在后续计算中重复进行乘法，提升效率
	BLOCK_PIXEL_SIZE = BLOCK_SIZE * SQUARE_SIZE;

	{
		// 56 x 16 elms for QuickSilver
		// 用地图的总宽度（以精细方格为单位）除以一个宏观块的边长，计算出当前网格在X轴方向上有多少个块
		mapDimensionsInBlocks.x = mapDims.mapx / BLOCK_SIZE;
		// 同上，计算Y轴方向上的块数。
		mapDimensionsInBlocks.y = mapDims.mapy / BLOCK_SIZE;
		// 计算并存储当前网格的总块数，这个值将用于后续分配各种数据数组的大小。
        mapBlockCount = mapDimensionsInBlocks.x * mapDimensionsInBlocks.y;

		// LOG("TK PathingState::Init X(%d) = mapx(%d) / blks(%d), Y(%d) = mapy(%d) / blks(%d)"
		// 	, mapDimensionsInBlocks.x
		// 	, mapDims.mapx
		// 	, BLOCK_SIZE
		// 	, mapDimensionsInBlocks.y
		// 	, mapDims.mapy
		// 	, BLOCK_SIZE
		// 	);
		// 这两行代码重复了上面的计算，并将结果存储在另一个成员变量 nbrOfBlocks 中。
		// 这可能是为了兼容从 IPathFinder 基类继承来的接口，因为基类也使用了名为 nbrOfBlocks 的变量。
		nbrOfBlocks.x = mapDims.mapx / BLOCK_SIZE;
		nbrOfBlocks.y = mapDims.mapy / BLOCK_SIZE;
		// pathingStates: 这是一个静态（static）变量，意味着它被所有 PathingState 实例共享。
		instanceIndex = pathingStates++;
	}

	AllocStateBuffer();

	{
		RECOIL_DETAILED_TRACY_ZONE;
		// 将传入的 pathFinderlist (一个包含所有寻路器实例指针的列表) 存储到成员变量 pathFinders 中。
		// pathFinderlist 存储的是相对应分辨率的寻路器实例 
		pathFinders = pathFinderlist;
		// : 这一行将一个以“精细方格”为单位的更新工作量 (SQUARES_TO_UPDATE)，转换为以当前分辨率的“宏观块”为单位的工作量。
		// 这个值用于控制当地图变化后，每帧增量更新多少路径数据。
		BLOCKS_TO_UPDATE = (SQUARES_TO_UPDATE) / (BLOCK_SIZE * BLOCK_SIZE) + 1;
		// 可能是一个用于节流更新速率的惩罚计数器
		blockUpdatePenalty = 0;
		// 用于控制向加载屏幕或网络日志发送进度更新消息的频率
		nextOffsetMessageIdx = 0;
		nextCostMessageIdx = 0;
		// 初始化路径数据的校验和为0
	 	pathChecksum = 0;
	 	fileHashCode = CalcHash(__func__);
		// 初始化两个原子计数器。它们被设置为当前分辨率网格的总块数。
		// 在多线程预计算过程中，各个线程会以原子方式（线程安全地）递减这两个计数器来领取任务（即要处理的块），从而实现高效的任务分配。
		offsetBlockNum = {mapDimensionsInBlocks.x * mapDimensionsInBlocks.y};
		costBlockNum = {mapDimensionsInBlocks.x * mapDimensionsInBlocks.y};

		vertexCosts.clear();
		// vertexCosts 向量分配一块巨大的内存。它的大小由 移动类型总数 * 块总数 * 每个块的出边方向数(8) 决定。
		// 这个向量将存储所有移动类型在所有宏观块之间移动的预计算成本
		// 所有成本被初始化为无穷大，表示在计算完成前，所有路径都是“不通”的。
		vertexCosts.resize(moveDefHandler.GetNumMoveDefs() * blockStates.GetSize() * PATH_DIRECTION_VERTICES, PATHCOST_INFINITY);
		// 分配并初始化 maxSpeedMods 向量。这个向量为每种移动类型存储一个值，
		// 代表该移动类型在整个地图上可能遇到的最大地形速度修正系数。这是一个预计算的优化，用于在A*搜索中校正启发式函数（H值）的估
		// maxSpeedMods 每个元素都是某一个MoveDef在整个地图上移动速度最快的移动系数
		maxSpeedMods.clear();
		maxSpeedMods.resize(moveDefHandler.GetNumMoveDefs(), 0.001f);
		// 清空几个用于预计算和增量更新过程中的临时数据容器，确保它们在使用前处于干净状态。
		// updatedBlocks: 存储因地图变化而需要更新的块队列
		// consumedBlocks: 存储在一个更新周期中正被处理的块列表
		// offsetBlocksSortedByCost: 存储用于在宏观块内寻找最佳通行点的偏移量列表
		updatedBlocks.clear();
		consumedBlocks.clear();
		offsetBlocksSortedByCost.clear();
	}
	// 这部分代码负责建立 PathingState 实例之间的父子链接。
	// childPE = this: 创建一个名为 childPE 的本地指针，指向当前正在初始化的这个 PathingState 实例 (this)。
	// parentPE = parentState: 创建一个名为 parentPE 的本地指针，指向通过函数参数传入的父级 PathingState 实例。
	// if (parentPE != nullptr): 检查是否存在一个父级实例。对于最低分辨率的 PathingState，
	// 它的父级就是中等分辨率的实例。而中等分辨率的父级可能是 nullptr
	// parentPE->nextPathState = childPE;: 如果父级存在，就将父级的 nextPathState 指针指向当前这个子级实例。
	// 这样就形成了一个单向链表，从高分辨率层级指向低分辨率层级。
	PathingState*  childPE = this;
	PathingState* parentPE = parentState;
	if (parentPE != nullptr)
		parentPE->nextPathState = childPE;


	// 为 FindBlockPosOffset 函数进行预计算。
	// precalc for FindBlockPosOffset()
	{
		offsetBlocksSortedByCost.reserve(BLOCK_SIZE * BLOCK_SIZE);
		// 这段双层循环的目的是为一个宏观块内的每一个精细方格计算一个“成本值”
		// for (..): 循环遍历一个 BLOCK_SIZE x BLOCK_SIZE 区域内的所有坐标 (x, z)
		// const float dx = ...; const float dz = ...;: 计算当前方格 (x, z) 相对于宏观块中心点的坐标 (dx, dz)。
		// 例如，在一个16x16的块中，中心点是(7.5, 7.5)，那么左下角(0,0)的相对坐标就是(-7.5, -7.5)。
		// const float cost = (dx * dx + dz * dz);: 计算该方格到块中心点的距离的平方作为其基础成本。
		// offsetBlocksSortedByCost.emplace_back(cost, x, z);: 将计算出的成本 cost 和方格的原始偏移坐标 (x, z) 一起存入 offsetBlocksSortedByCost 向量中。
		for (unsigned int z = 0; z < BLOCK_SIZE; ++z) {
			for (unsigned int x = 0; x < BLOCK_SIZE; ++x) {
				const float dx = x - (float)(BLOCK_SIZE - 1) * 0.5f;
				const float dz = z - (float)(BLOCK_SIZE - 1) * 0.5f;
				const float cost = (dx * dx + dz * dz);

				offsetBlocksSortedByCost.emplace_back(cost, x, z);
			}
		}
		// 在所有方格的成本都计算完毕后，这一步对整个 offsetBlocksSortedByCost 向量进行排序。应该按照每个元素 cost 成员的升序来排列
		// 执行完毕后，offsetBlocksSortedByCost 向量中存储了从块中心点由近及远的所有方格的偏移量。
		// 当 FindBlockPosOffset 函数需要在一个宏观块内寻找最佳通行点时，它可以直接遍历这个已排序的列表，从最靠近中心的点开始检查，
		// 一旦找到一个可通行的点并且后续点的距离成本已经超出现有最佳成本，就可以提前终止搜索，从而极大地提高了效率。
		std::stable_sort(offsetBlocksSortedByCost.begin(), offsetBlocksSortedByCost.end(), [](const SOffsetBlock& a, const SOffsetBlock& b) {
			return (a.cost < b.cost);
		});
	}
	// 这个代码块是一个在游戏加载时执行的、计算量巨大的一次性预处理步骤。
	// 它的核心目标是为每一种移动类型（MoveDef）找出它在整张地图上可能遇到的最快地形速度修正系数，并对这个结果进行处理，
	// 以供后续的A*寻路算法进行一项关键的性能和准确性优化。
	if (BLOCK_SIZE == LOWRES_PE_BLOCKSIZE) { //这整个代码块仅当当前正在初始化的 PathingState 实例是用于最低分辨率的路径估算器时才会执行
		assert(parentPE != nullptr);

		// calculate map-wide maximum positional speedmod for each MoveDef
		for_mt(0, moveDefHandler.GetNumMoveDefs(), [&](unsigned int i) {
			const MoveDef* md = moveDefHandler.GetMoveDefByPathType(i);

			for (int y = 0; y < mapDims.mapy; y++) {
				for (int x = 0; x < mapDims.mapx; x++) { // 迭代地图每一个网格
					// 对于每一个方格，调用此函数计算出当前移动类型 md 在该方格上的地形速度修正系数（例如，公路上 > 1.0，泥地里 < 1.0）。
					// childPE = this 
					childPE->maxSpeedMods[i] = std::max(childPE->maxSpeedMods[i], CMoveMath::GetPosSpeedMod(*md, x, y));
				}
			}
		});

		// calculate reciprocals, avoids divisions in TestBlock
		// 计算倒数，以避免在 TestBlock 函数（A*搜索的核心循环）中进行昂贵的除法运算

		for (unsigned int i = 0; i < maxSpeedMods.size(); i++) {
			// 将 maxSpeedMods 数组中存储的最大速度修正系数替换为其倒数。
			// 这样，在后续需要用“距离除以最大速度”的地方，就可以用“距离乘以这个倒数”来代替，因为乘法通常比浮点数除法更快。
			childPE->maxSpeedMods[i] = 1.0f / childPE->maxSpeedMods[i];
			//  将计算并处理好的倒数值复制一份给父级 PathingState 实例。这样，中等分辨率的 PathingState 就不需要再重复进行上面那段耗时巨大的全图扫描计算了。
			parentPE->maxSpeedMods[i] = childPE->maxSpeedMods[i];
		}
	}

	// load precalculated data if it exists
	InitEstimator(peFileName, mapFileName);
}

void PathingState::Terminate()
{
	RECOIL_DETAILED_TRACY_ZONE;
	if (pathCache[0] != nullptr)
		pcMemPool.free(pathCache[0]);
	
	if (pathCache[1] != nullptr)
		pcMemPool.free(pathCache[1]);

	//LOG("Pathing unporcessed updatedBlocks is %llu", updatedBlocks.size());

	// Clear out lingering unprocessed map changes
	while (!updatedBlocks.empty()) {
		const int2& pos = updatedBlocks.front();
		const int idx = BlockPosToIdx(pos);
		updatedBlocks.pop_front();
		blockStates.nodeMask[idx] &= ~PATHOPT_OBSOLETE;
		blockStates.nodeLinksObsoleteFlags[idx] = 0;
	}

	// allow our PNSB to be reused across reloads
	if (instanceIndex < nodeStateBuffers.size())
		nodeStateBuffers[instanceIndex] = std::move(blockStates);
}

void PathingState::AllocStateBuffer()
{
	RECOIL_DETAILED_TRACY_ZONE;
	if (instanceIndex >= nodeStateBuffers.size())
		nodeStateBuffers.emplace_back();

	nodeStateBuffers[instanceIndex].Clear();
	nodeStateBuffers[instanceIndex].Resize(nbrOfBlocks, int2(mapDims.mapx, mapDims.mapy));

	// steal memory, returned in dtor
	blockStates = std::move(nodeStateBuffers[instanceIndex]);
}

bool PathingState::RemoveCacheFile(const std::string& peFileName, const std::string& mapFileName)
{
	RECOIL_DETAILED_TRACY_ZONE;
	return (FileSystem::Remove(GetCacheFileName(IntToString(fileHashCode, "%x"), peFileName, mapFileName)));
}


void PathingState::InitEstimator(const std::string& peFileName, const std::string& mapFileName)
{
	RECOIL_DETAILED_TRACY_ZONE;
	const unsigned int numThreads = ThreadPool::GetNumThreads();
	//LOG("TK PathingState::InitEstimator: %d threads available", numThreads);

	// Not much point in multithreading these...
	// 调用辅助函数，其主要作用是为 blockStates.peNodeOffsets 这个巨大的向量分配内存，
	// 确保它有足够的空间来存储每一种移动类型在每一个宏观块中的“最佳通行点”数据。
	InitBlocks();
	// 这是函数的核心分支。它尝试调用 ReadFile 从磁盘加载预计算数据。
	// 如果 ReadFile 返回 false（即缓存文件不存在、已损坏或与当前版本不匹配），则执行 if 块内的代码来从头生成数据。
	if (!ReadFile(peFileName, mapFileName)) {
		char calcMsg[512];
		const char* fmtStrs[4] = {
			"[%s] creating PE%u cache with %u PF threads",
			"[%s] creating PE%u cache with %u PF thread",
			"[%s] writing PE%u cache-file %s-%x",
			"[%s] written PE%u cache-file %s-%x",
		};

		{
			sprintf(calcMsg, fmtStrs[numThreads==1], __func__, BLOCK_SIZE, numThreads);
			loadscreen->SetLoadMessage(calcMsg);
		}

		// Mark block directions as dirty to ensure they get updated.
		// 注释说明了其意图：将所有块的连接方向标记为“脏”的，以确保它们都会被更新
		auto& nodeFlags = blockStates.nodeLinksObsoleteFlags;
		// 遍历 nodeLinksObsoleteFlags 数组，并将每个元素都设置为 PATH_DIRECTIONS_HALF_MASK。
		// 这相当于一个全局标记，告诉后续的计算函数：“所有宏观块之间的通行成本都需要被重新计算”
		std::for_each(nodeFlags.begin(), nodeFlags.end(), [](std::uint8_t& f){ f = PATH_DIRECTIONS_HALF_MASK; });

		// note: only really needed if numExtraThreads > 0
		// 创建一个线程屏障 (barrier)。它是一个同步工具，能确保所有线程都执行到屏障点后，才能一起继续执行后续代码
		spring::barrier pathBarrier(numThreads);
		// 这个函数内部包含两个计算阶段，中间由 pathBarrier 隔开。所有线程必须先全部完成第一阶段（计算块内偏移量），
		// 才能开始第二阶段（计算块间通行成本），因为第二阶段依赖第一阶段的结果
		for_mt(0, numThreads, [this, &pathBarrier](int i) {
			CalcOffsetsAndPathCosts(ThreadPool::GetThreadNum(), &pathBarrier);
		});

		std::for_each(nodeFlags.begin(), nodeFlags.end(), [](std::uint8_t& f){ f = 0; });
		// 在所有计算都完成后，再次遍历 nodeLinksObsoleteFlags 数组，并将所有标志位清零。这表示所有数据都已是最新，不再“脏”了
		sprintf(calcMsg, fmtStrs[2], __func__, BLOCK_SIZE, peFileName.c_str(), fileHashCode);
		loadscreen->SetLoadMessage(calcMsg, true);
		//  更新加载屏幕，告知用户正在将计算结果写入缓存文件，然后调用 WriteFile 函数执行实际的写入操作
		WriteFile(peFileName, mapFileName);

		sprintf(calcMsg, fmtStrs[3], __func__, BLOCK_SIZE, peFileName.c_str(), fileHashCode);
		loadscreen->SetLoadMessage(calcMsg, true);
	}

	// calculate checksum over block-offsets and vertex-costs
	// 无论数据是从文件加载的还是刚刚生成的，都会调用 CalcChecksum 函数，对内存中最终的预计算数据计算一次校验和。
	// 这个校验和将在多人游戏中用于同步检查，确保所有玩家的寻路数据完全一致
	pathChecksum = CalcChecksum();
	// 最后，为 PathingState 实例创建两个路径缓存对象 (CPathCache)。一个用于同步环境([1])，一个用于非同步环境([0])。
	// 这些缓存用于存储运行时的寻路结果，与预计算数据是不同的
	pathCache[0] = pcMemPool.alloc<CPathCache>(mapDimensionsInBlocks.x, mapDimensionsInBlocks.y);
	pathCache[1] = pcMemPool.alloc<CPathCache>(mapDimensionsInBlocks.x, mapDimensionsInBlocks.y);
}

void PathingState::InitBlocks()
{
	RECOIL_DETAILED_TRACY_ZONE;
	// TK NOTE: moveDefHandler.GetNumMoveDefs() == 47
	blockStates.peNodeOffsets.resize(moveDefHandler.GetNumMoveDefs());
	for (unsigned int idx = 0; idx < moveDefHandler.GetNumMoveDefs(); idx++) {
		blockStates.peNodeOffsets[idx].resize(mapDimensionsInBlocks.x * mapDimensionsInBlocks.y);
		//LOG("TK PathingState::InitBlocks: blockStates.peNodeOffsets %d now %d", idx, blockStates.peNodeOffsets[idx].size());
	}
}


__FORCE_ALIGN_STACK__
void PathingState::CalcOffsetsAndPathCosts(unsigned int threadNum, spring::barrier* pathBarrier)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// reset FPU state for synced computations
	//streflop::streflop_init<streflop::Simple>();

	// NOTE: EstimatePathCosts() [B] is temporally dependent on CalculateBlockOffsets() [A],
	// A must be completely finished before B_i can be safely called. This means we cannot
	// let thread i execute (A_i, B_i), but instead have to split the work such that every
	// thread finishes its part of A before any starts B_i.
	const unsigned int maxBlockIdx = blockStates.GetSize() - 1;
	int i;

	while ((i = --offsetBlockNum) >= 0)
		CalculateBlockOffsets(maxBlockIdx - i, threadNum);

	pathBarrier->wait();

	while ((i = --costBlockNum) >= 0)
		EstimatePathCosts(maxBlockIdx - i, threadNum);
}

void PathingState::CalculateBlockOffsets(unsigned int blockIdx, unsigned int threadNum)
{
	RECOIL_DETAILED_TRACY_ZONE;
	const int2 blockPos = BlockIdxToPos(blockIdx);

	if (threadNum == 0 && blockIdx >= nextOffsetMessageIdx) {
		nextOffsetMessageIdx = blockIdx + blockStates.GetSize() / 16;
		clientNet->Send(CBaseNetProtocol::Get().SendCPUUsage(BLOCK_SIZE | (blockIdx << 8)));
	}

	for (unsigned int i = 0; i < moveDefHandler.GetNumMoveDefs(); i++) {
		const MoveDef* md = moveDefHandler.GetMoveDefByPathType(i);

		//LOG("TK PathingState::InitBlocks: blockStates.peNodeOffsets %d now %d looking up %d", i, blockStates.peNodeOffsets[md->pathType].size(), blockIdx);
		blockStates.peNodeOffsets[md->pathType][blockIdx] = FindBlockPosOffset(*md, blockPos.x, blockPos.y, threadNum);
		// LOG("UPDATED blockStates.peNodeOffsets[%d][%d] = (%d, %d) : (%d, %d)"
		// 		, md->pathType, blockIdx
		// 		, blockStates.peNodeOffsets[md->pathType][blockIdx].x, blockStates.peNodeOffsets[md->pathType][blockIdx].y
		// 		, blockPos.x, blockPos.y);
	}
}

/**
 * Move around the blockPos a bit, so we `surround` unpassable blocks.
 */
int2 PathingState::FindBlockPosOffset(const MoveDef& moveDef, unsigned int blockX, unsigned int blockZ, int threadNum) const
{
	RECOIL_DETAILED_TRACY_ZONE;
	// lower corner position of block
	const unsigned int lowerX = blockX * BLOCK_SIZE;
	const unsigned int lowerZ = blockZ * BLOCK_SIZE;
	const unsigned int blockArea = (BLOCK_SIZE * BLOCK_SIZE) / SQUARE_SIZE;

	int2 bestPos(lowerX + (BLOCK_SIZE >> 1), lowerZ + (BLOCK_SIZE >> 1));
	float bestCost = std::numeric_limits<float>::max();

	// same as above, but with squares sorted by their baseCost
	// s.t. we can exit early when a square exceeds our current
	// best (from testing, on avg. 40% of blocks can be skipped)
	for (const SOffsetBlock& ob: offsetBlocksSortedByCost) {
		if (ob.cost >= bestCost)
			break;

		const int2 blockPos(lowerX + ob.offset.x, lowerZ + ob.offset.y);
		const float speedMod = CMoveMath::GetPosSpeedMod(moveDef, blockPos.x, blockPos.y);

		//assert((blockArea / (0.001f + speedMod) >= 0.0f);
		const float cost = ob.cost + (blockArea / (0.001f + speedMod));

		if (cost >= bestCost)
			continue;

		if (!CMoveMath::IsBlockedStructure(moveDef, blockPos.x, blockPos.y, nullptr, threadNum)
				&& !moveDef.IsInExitOnly(blockPos.x, blockPos.y)) {
			bestCost = cost;
			bestPos  = blockPos;
		}
	}

	// return the offset found
	return bestPos;
}

void PathingState::EstimatePathCosts(unsigned int blockIdx, unsigned int threadNum)
{
	RECOIL_DETAILED_TRACY_ZONE;
	const int2 blockPos = BlockIdxToPos(blockIdx);

	if (threadNum == 0 && blockIdx >= nextCostMessageIdx) {
		nextCostMessageIdx = blockIdx + blockStates.GetSize() / 16;

		char calcMsg[128];
		sprintf(calcMsg, "[%s] precached %d of %d blocks", __func__, blockIdx, blockStates.GetSize());

		clientNet->Send(CBaseNetProtocol::Get().SendCPUUsage(0x1 | BLOCK_SIZE | (blockIdx << 8)));
		loadscreen->SetLoadMessage(calcMsg, (blockIdx != 0));
	}

	for (unsigned int i = 0; i < moveDefHandler.GetNumMoveDefs(); i++) {
		const MoveDef* md = moveDefHandler.GetMoveDefByPathType(i);

		CalcVertexPathCosts(*md, blockPos, threadNum);
	}
}

/**
 * Calculate costs of paths to all vertices connected from the given block
 */
void PathingState::CalcVertexPathCosts(const MoveDef& moveDef, int2 block, unsigned int threadNum)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// see GetBlockVertexOffset(); costs are bi-directional and only
	// calculated for *half* the outgoing edges (while costs for the
	// other four directions are stored at the adjacent vertices)
	auto idx = BlockPosToIdx(block);
	const uint8_t nodeLinksObsoleteFlags = blockStates.nodeLinksObsoleteFlags[idx]
								  		 & (moveDef.allowDirectionalPathing) ? PATH_DIRECTIONS_MASK : PATH_DIRECTIONS_HALF_MASK;

	int pathdir = 0;
	for (int checkBit = 1; checkBit <= PATHDIR_LEFT_DOWN_MASK; checkBit <<= 1, ++pathdir) {
		if (nodeLinksObsoleteFlags & checkBit)
			CalcVertexPathCost(moveDef, block, pathdir, threadNum);
	}
}

void PathingState::CalcVertexPathCost(
	const MoveDef& moveDef,
	int2 parentBlockPos,
	unsigned int pathDir,
	unsigned int threadNum
) {
	RECOIL_DETAILED_TRACY_ZONE;
	const int2 childBlockPos = parentBlockPos + PE_DIRECTION_VECTORS[pathDir];

	const unsigned int parentBlockIdx = BlockPosToIdx(parentBlockPos);
	const unsigned int  childBlockIdx = BlockPosToIdx( childBlockPos);

	// LOG("TK PathingState::CalcVertexPathCost parent (%d, %d) = %d (of %d), child (%d,%d) = %d (of %d)"
	// 		, parentBlockPos.x
	// 		, parentBlockPos.y
	// 		, parentBlockIdx
	// 		, mapDimensionsInBlocks.x
	// 		, childBlockPos.x
	// 		, childBlockPos.y
	// 		, childBlockIdx
	// 		, mapDimensionsInBlocks.y
	// 		);

	const unsigned int  vertexCostIdx =
		moveDef.pathType * mapBlockCount * PATH_DIRECTION_VERTICES +
		parentBlockIdx * PATH_DIRECTION_VERTICES +
		pathDir;

	// outside map?
	if ((unsigned)childBlockPos.x >= mapDimensionsInBlocks.x || (unsigned)childBlockPos.y >= mapDimensionsInBlocks.y) {
		vertexCosts[vertexCostIdx] = PATHCOST_INFINITY;
		return;
	}


	// start position within parent block, goal position within child block
	const int2 parentSquare = blockStates.peNodeOffsets[moveDef.pathType][parentBlockIdx];
	const int2  childSquare = blockStates.peNodeOffsets[moveDef.pathType][ childBlockIdx];

	const float3 startPos = SquareToFloat3(parentSquare.x, parentSquare.y);
	const float3  goalPos = SquareToFloat3( childSquare.x,  childSquare.y);

	// keep search exactly contained within the two blocks
	CRectangularSearchConstraint pfDef(startPos, goalPos, 0.0f, BLOCK_SIZE);

	// LOG("TK PathingState::CalcVertexPathCost: (%d,%d -> %d,%d) (%d,%d -> %d,%d [%d])"
	// 	, parentSquare.x, parentSquare.y
	// 	, childSquare.x,  childSquare.y
	// 	, pfDef.startSquareX, pfDef.startSquareZ
	// 	, pfDef.goalSquareX, pfDef.goalSquareZ
	// 	, BLOCK_SIZE);

	// we never want to allow searches from any blocked starting positions
	// (otherwise PE and PF can disagree), but are more lenient for normal
	// searches so players can "unstuck" units
	// note: PE itself should ensure this never happens to begin with?
	//
	// blocked goal positions are always early-outs (no searching needed)
	const bool strtBlocked = ((CMoveMath::IsBlocked(moveDef, startPos, nullptr, threadNum) & CMoveMath::BLOCK_STRUCTURE) != 0);
	const bool goalBlocked = pfDef.IsGoalBlocked(moveDef, CMoveMath::BLOCK_STRUCTURE, nullptr, threadNum);

	if (strtBlocked || goalBlocked) {
		vertexCosts[vertexCostIdx] = PATHCOST_INFINITY;
		return;
	}

	// find path from parent to child block
	pfDef.skipSubSearches = true;
	pfDef.testMobile      = false;
	pfDef.needPath        = false;
	pfDef.exactPath       = true;
	pfDef.dirIndependent  = true;

	IPath::Path path;
	IPath::SearchResult result = pathFinders[threadNum]->GetPath(moveDef, pfDef, nullptr, startPos, path, MAX_SEARCHED_NODES_PF >> 2);

	// store the result
	if (result == IPath::Ok) {
		vertexCosts[vertexCostIdx] = path.pathCost;
	} else {
		vertexCosts[vertexCostIdx] = PATHCOST_INFINITY;
	}
}


/**
 * Try to read offset and vertices data from file, return false on failure
 */
bool PathingState::ReadFile(const std::string& peFileName, const std::string& mapFileName)
{
	RECOIL_DETAILED_TRACY_ZONE;
	const std::string hashHexString = IntToString(fileHashCode, "%x");
	const std::string cacheFileName = GetCacheFileName(hashHexString, peFileName, mapFileName);

	LOG("[PathEstimator::%s] hash=%s file=\"%s\" (exists=%d)", __func__, hashHexString.c_str(), cacheFileName.c_str(), FileSystem::FileExists(cacheFileName));

	if (!FileSystem::FileExists(cacheFileName))
		return false;

	std::unique_ptr<IArchive> upfile(archiveLoader.OpenArchive(dataDirsAccess.LocateFile(cacheFileName), "sdz"));

	if (upfile == nullptr || !upfile->IsOpen()) {
		FileSystem::Remove(cacheFileName);
		return false;
	}

	char calcMsg[512];
	sprintf(calcMsg, "Reading Estimate PathCosts [%d]", BLOCK_SIZE);
	loadscreen->SetLoadMessage(calcMsg);

	const unsigned fid = upfile->FindFile("pathinfo");
	if (fid >= upfile->NumFiles()) {
		FileSystem::Remove(cacheFileName);
		return false;
	}

	std::vector<std::uint8_t> buffer;

	if (!upfile->GetFile(fid, buffer) || buffer.size() < 4) {
		FileSystem::Remove(cacheFileName);
		return false;
	}

	const unsigned int filehash = *(reinterpret_cast<unsigned int*>(&buffer[0]));
	const unsigned int blockSize = blockStates.GetSize() * sizeof(short2);
	unsigned int pos = sizeof(unsigned);

	if (filehash != fileHashCode) {
		FileSystem::Remove(cacheFileName);
		return false;
	}

	if (buffer.size() < (pos + blockSize * moveDefHandler.GetNumMoveDefs())) {
		FileSystem::Remove(cacheFileName);
		return false;
	}

	// read center-offset data
	for (int pathType = 0; pathType < moveDefHandler.GetNumMoveDefs(); ++pathType) {
		std::memcpy(&blockStates.peNodeOffsets[pathType][0], &buffer[pos], blockSize);
		pos += blockSize;
	}

	// read vertex-cost data
	if (buffer.size() < (pos + vertexCosts.size() * sizeof(float))) {
		FileSystem::Remove(cacheFileName);
		return false;
	}

	std::memcpy(&vertexCosts[0], &buffer[pos], vertexCosts.size() * sizeof(float));
	return true;
}


/**
 * Try to write offset and vertex data to file.
 */
bool PathingState::WriteFile(const std::string& peFileName, const std::string& mapFileName)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// we need this directory to exist
	if (!FileSystem::CreateDirectory(GetPathCacheDir()))
		return false;

	const std::string hashHexString = IntToString(fileHashCode, "%x");
	const std::string cacheFileName = GetCacheFileName(hashHexString, peFileName, mapFileName);

	LOG("[PathEstimator::%s] hash=%s file=\"%s\" (exists=%d)", __func__, hashHexString.c_str(), cacheFileName.c_str(), FileSystem::FileExists(cacheFileName));

	// open file for writing in a suitable location
	zipFile file = zipOpen(dataDirsAccess.LocateFile(cacheFileName, FileQueryFlags::WRITE).c_str(), APPEND_STATUS_CREATE);

	if (file == nullptr)
		return false;

	zipOpenNewFileInZip(file, "pathinfo", nullptr, nullptr, 0, nullptr, 0, nullptr, Z_DEFLATED, Z_BEST_COMPRESSION);

	// write hash-code (NOTE: this also affects the CRC!)
	zipWriteInFileInZip(file, (const void*) &fileHashCode, 4);

	// write center-offsets
	for (int pathType = 0; pathType < moveDefHandler.GetNumMoveDefs(); ++pathType) {
		zipWriteInFileInZip(file, (const void*) &blockStates.peNodeOffsets[pathType][0], blockStates.peNodeOffsets[pathType].size() * sizeof(short2));
	}

	// write vertex-costs
	zipWriteInFileInZip(file, vertexCosts.data(), vertexCosts.size() * sizeof(float));

	zipCloseFileInZip(file);
	zipClose(file, nullptr);


	// get the CRC over the written path data
	std::unique_ptr<IArchive> upfile(archiveLoader.OpenArchive(dataDirsAccess.LocateFile(cacheFileName), "sdz"));

	if (upfile == nullptr || !upfile->IsOpen()) {
		FileSystem::Remove(cacheFileName);
		return false;
	}

	assert(upfile->FindFile("pathinfo") < upfile->NumFiles());
	return true;
}


/**
 * Update some obsolete blocks using the FIFO-principle
 */
void PathingState::Update()
{
	RECOIL_DETAILED_TRACY_ZONE;
	pathCache[0]->Update();
	pathCache[1]->Update();

	//LOG("PathingState::Update %d", BLOCK_SIZE);

	const unsigned int numMoveDefs = moveDefHandler.GetNumMoveDefs();

	if (numMoveDefs == 0)
		return;

	if (updatedBlocks.empty())
		return;

	// determine how many blocks we should update
	int blocksToUpdate = 0;
	{
		const int progressiveUpdates = std::ceil(updatedBlocks.size() * (1.f / (BLOCKS_TO_UPDATE<<2)) * modInfo.pfUpdateRateScale);
		const int MIN_BLOCKS_TO_UPDATE = 1;
		const int MAX_BLOCKS_TO_UPDATE = std::max<int>(BLOCKS_TO_UPDATE >> 1, MIN_BLOCKS_TO_UPDATE);

		blocksToUpdate = std::clamp(progressiveUpdates, MIN_BLOCKS_TO_UPDATE, MAX_BLOCKS_TO_UPDATE) * numMoveDefs;
	
		// LOG("[%d] blocksToUpdate=%d progressiveUpdates=%d [%f]"
		// 		, BLOCK_SIZE, blocksToUpdate, progressiveUpdates, modInfo.pfUpdateRateScale);
	}

	//LOG("PathingState::Update blocksToUpdate %d", blocksToUpdate);

	if (blocksToUpdate == 0)
		return;

	//LOG("PathingState::Update updatedBlocks.empty == %d", (int)updatedBlocks.empty());
	//LOG("PathingState::Update updatedBlocksDelayActive %d", (int)updatedBlocksDelayActive);

	UpdateVertexPathCosts(blocksToUpdate);
}

void PathingState::UpdateVertexPathCosts(int blocksToUpdate)
{
	RECOIL_DETAILED_TRACY_ZONE;
	const unsigned int numMoveDefs = moveDefHandler.GetNumMoveDefs();

	if (numMoveDefs == 0)
		return;

	if (blocksToUpdate == -1)
		blocksToUpdate = updatedBlocks.size() * numMoveDefs;

	int consumeBlocks = int(blocksToUpdate != 0) * int(ceil(float(blocksToUpdate) / numMoveDefs)) * numMoveDefs;

	consumedBlocks.clear();
	consumedBlocks.reserve(consumeBlocks);

	//LOG("PathingState::Update %d", updatedBlocks.size());

	std::vector<int> blockIds;
	blockIds.reserve(updatedBlocks.size());

	// get blocks to update
	while (!updatedBlocks.empty()) {
		const int2& pos = updatedBlocks.front();
		const int idx = BlockPosToIdx(pos);

		if ((blockStates.nodeMask[idx] & PATHOPT_OBSOLETE) == 0) {
			updatedBlocks.pop_front();
			continue;
		}

		if (consumedBlocks.size() >= blocksToUpdate)
			break;

		// issue repathing for all active movedefs
		for (unsigned int i = 0; i < numMoveDefs; i++) {
			const MoveDef* md = moveDefHandler.GetMoveDefByPathType(i);

			consumedBlocks.emplace_back(pos, md);
			//LOG("TK PathingState::Update: moveDef = %d %p (%p)", consumedBlocks.size(), &consumedBlocks.back(), consumedBlocks.back().moveDef);
		}

		updatedBlocks.pop_front(); // must happen _after_ last usage of the `pos` reference!
		blockStates.nodeMask[idx] &= ~PATHOPT_OBSOLETE;
		blockIds.emplace_back(idx);
	}

	// FindOffset (threadsafe)
	{
		SCOPED_TIMER("Sim::Path::Estimator::FindOffset");

		auto updateOffset = [&](const int n) {
				// copy the next block in line
				const SingleBlock sb = consumedBlocks[n];
				const int blockN = BlockPosToIdx(sb.blockPos);
				const MoveDef* currBlockMD = sb.moveDef;
				blockStates.peNodeOffsets[currBlockMD->pathType][blockN] = FindBlockPosOffset(*currBlockMD, sb.blockPos.x, sb.blockPos.y, ThreadPool::GetThreadNum());
			};

		for_mt(0, consumedBlocks.size(), updateOffset);
	}

	{
		SCOPED_TIMER("Sim::Path::Estimator::CalcVertexPathCosts");
		std::atomic<std::int64_t> updateCostBlockNum = consumedBlocks.size();
		const size_t threadsUsed = std::min(consumedBlocks.size(), (size_t)ThreadPool::GetNumThreads());

		auto updateVertexPathCosts = [this, &updateCostBlockNum](int threadNum){
				std::int64_t n;
				while ((n = --updateCostBlockNum) >= 0){
					//LOG("TK PathingState::Update: PROC moveDef = %d %p (%p)", n, &consumedBlocks[n], consumedBlocks[n].moveDef);
					CalcVertexPathCosts(*consumedBlocks[n].moveDef, consumedBlocks[n].blockPos, threadNum);
				}
			};

		for_mt(0, threadsUsed, updateVertexPathCosts);
	}

	std::for_each(blockIds.begin(), blockIds.end(), [this](int idx){ blockStates.nodeLinksObsoleteFlags[idx] = 0; });
}


/**
 * Mark affected blocks as obsolete
 */
void PathingState::MapChanged(unsigned int x1, unsigned int z1, unsigned int x2, unsigned z2)
{
	assert(x2 >= x1);
	assert(z2 >= z1);

	const int lowerX = int(x1 / BLOCK_SIZE) - 1;
	const int upperX = int(x2 / BLOCK_SIZE) + 1;
	const int lowerZ = int(z1 / BLOCK_SIZE) - 1;
	const int upperZ = int(z2 / BLOCK_SIZE) + 0;

	// find the upper and lower corner of the rectangular area
	const int startX = std::clamp(lowerX, 0, int(mapDimensionsInBlocks.x - 1));
	const int endX   = std::clamp(upperX, 0, int(mapDimensionsInBlocks.x - 1));
	const int startZ = std::clamp(lowerZ, 0, int(mapDimensionsInBlocks.y - 1));
	const int endZ   = std::clamp(upperZ, 0, int(mapDimensionsInBlocks.y - 1));

	bool pathingDirectional = pathManager->AllowDirectionalPathing();

	// LOG("%s: clamped to [%d, %d] -> [%d, %d]", __func__, lowerX, lowerZ, upperX, upperZ);

	constexpr uint32_t ALL_LINKS = PATH_DIRECTIONS_MASK;
	constexpr uint32_t MASK_REMOVE_LEFT = ~(PATHDIR_LEFT_MASK | PATHDIR_LEFT_UP_MASK | PATHDIR_LEFT_DOWN_MASK);
	constexpr uint32_t MASK_REMOVE_RIGHT = ~(PATHDIR_RIGHT_MASK | PATHDIR_RIGHT_UP_MASK| PATHDIR_RIGHT_DOWN_MASK);
	constexpr uint32_t MASK_REMOVE_UP = ~(PATHDIR_UP_MASK | PATHDIR_LEFT_UP_MASK | PATHDIR_RIGHT_UP_MASK);
	constexpr uint32_t MASK_REMOVE_DOWN = ~(PATHDIR_DOWN_MASK | PATHDIR_LEFT_DOWN_MASK | PATHDIR_RIGHT_DOWN_MASK);

	constexpr uint32_t activeLinks[] = {
		ALL_LINKS & MASK_REMOVE_LEFT  & MASK_REMOVE_UP,
		ALL_LINKS                     & MASK_REMOVE_UP,
		ALL_LINKS & MASK_REMOVE_RIGHT & MASK_REMOVE_UP,
		ALL_LINKS & MASK_REMOVE_LEFT,
		ALL_LINKS,
		ALL_LINKS & MASK_REMOVE_RIGHT,
		ALL_LINKS & MASK_REMOVE_LEFT  & MASK_REMOVE_DOWN,
		ALL_LINKS                     & MASK_REMOVE_DOWN,
		ALL_LINKS & MASK_REMOVE_RIGHT & MASK_REMOVE_DOWN,
	};

	auto getIdxFromZ = [&](int z){
			if (z == lowerZ) return 0;
			else if (z == upperZ) return 6;
			else return 3;
	};
	auto getIdxFromX = [&](int x){
			if (x == lowerX) return 0;
			else if (x == upperX) return 2;
			else return 1;
	};

	// mark the blocks inside the rectangle, enqueue them
	// from upper to lower because of the placement of the
	// bi-directional vertices
	for (int z = endZ; z >= startZ; z--) {
		for (int x = endX; x >= startX; x--) {
			const int idx = BlockPosToIdx(int2(x, z));
			std::uint8_t blockOrigLinkFlags = blockStates.nodeLinksObsoleteFlags[idx];

			uint8_t linkType = getIdxFromZ(z) + getIdxFromX(x);
			blockStates.nodeLinksObsoleteFlags[idx] = uint8_t(activeLinks[linkType]);
			if (!pathingDirectional) 
				blockStates.nodeLinksObsoleteFlags[idx] &= PATH_DIRECTIONS_HALF_MASK;

			if (blockStates.nodeLinksObsoleteFlags[idx] == blockOrigLinkFlags)
				continue;

			//if ((blockStates.nodeMask[idx] & PATHOPT_OBSOLETE) != 0)
			//	continue;

			//LOG("%s: [%d, %d] lower is %02x", __func__, x, z, blockOrigLinkFlags);
			//LOG("%s: clamped to [%d, %d] -> [%d, %d]", __func__, lowerX, lowerZ, upperX, upperZ);
			//LOG("%s: [%d, %d] result is %02x", __func__, x, z, blockStates.nodeLinksObsoleteFlags[idx]);

			if (blockOrigLinkFlags != 0)
				continue;

			updatedBlocks.emplace_back(x, z);
			blockStates.nodeMask[idx] |= PATHOPT_OBSOLETE;
		}
	}
}


std::uint32_t PathingState::CalcChecksum() const
{
	RECOIL_DETAILED_TRACY_ZONE;
	std::uint32_t chksum = 0;
	std::uint64_t nbytes = vertexCosts.size() * sizeof(float);
	std::uint64_t offset = 0;

	#if (ENABLE_NETLOG_CHECKSUM == 1)
	std::array<char, 128 + sha512::SHA_LEN * 2 + 1> msgBuffer;

	sha512::hex_digest hexChars;
	sha512::raw_digest shaBytes;
	std::vector<uint8_t> rawBytes;
	#endif

	#if (ENABLE_NETLOG_CHECKSUM == 1)
	for (const auto& pathTypeOffsets: blockStates.peNodeOffsets) {
		nbytes += (pathTypeOffsets.size() * sizeof(short2));
	}

	rawBytes.clear();
	rawBytes.resize(nbytes);

	for (const auto& pathTypeOffsets: blockStates.peNodeOffsets) {
		nbytes = pathTypeOffsets.size() * sizeof(short2);
		offset += nbytes;

		std::memcpy(&rawBytes[offset - nbytes], pathTypeOffsets.data(), nbytes);
	}

	// for (int i=0; i<blockStates.peNodeOffsets.size(); i++){
	// 	for (int j =0; j<blockStates.peNodeOffsets[i].size(); j++){
	// 		LOG("blockStates.peNodeOffsets[%d][%d] = (%d %d)", i, j
	// 		, blockStates.peNodeOffsets[i][j].x
	// 		, blockStates.peNodeOffsets[i][j].y);
	// 	}
	// }

	{
		nbytes = vertexCosts.size() * sizeof(float);
		offset += nbytes;

		std::memcpy(&rawBytes[offset - nbytes], vertexCosts.data(), nbytes);

		sha512::calc_digest(rawBytes, shaBytes); // hash(offsets|costs)
		sha512::dump_digest(shaBytes, hexChars); // hexify(hash)

		SNPRINTF(msgBuffer.data(), msgBuffer.size(), "[PE::%s][BLK_SIZE=%d][SHA_DATA=%s]", __func__, BLOCK_SIZE, hexChars.data());
		CLIENT_NETLOG(gu->myPlayerNum, LOG_LEVEL_INFO, msgBuffer.data());
	}
	#endif

	// make path-estimator checksum part of synced state s.t. when
	// a client has a corrupted or stale cache it desyncs from the
	// start, not minutes later
	for (size_t i = 0, n = shaBytes.size() / 4; i < n; i += 1) {
		const uint16_t hi = (shaBytes[i * 4 + 0] << 8) | (shaBytes[i * 4 + 1] << 0);
		const uint16_t lo = (shaBytes[i * 4 + 2] << 8) | (shaBytes[i * 4 + 3] << 0);

		const SyncedUint su = (hi << 16) | (lo << 0);

		// copy first four bytes to reduced checksum
		if (chksum == 0)
			chksum = su;
	}

	return chksum;
}

// CPathCache::CacheItem PathingState::GetCache(const int2 strtBlock, const int2 goalBlock, float goalRadius, int pathType, const bool synced) const
// {
// 	const std::lock_guard<std::mutex> lock(cacheAccessLock);
// 	return pathCache[synced]->GetCachedPath(strtBlock, goalBlock, goalRadius, pathType);
// }

void PathingState::AddCache(const IPath::Path* path, const IPath::SearchResult result, const int2 strtBlock, const int2 goalBlock, float goalRadius, int pathType, const bool synced)
{
	RECOIL_DETAILED_TRACY_ZONE;
	const std::lock_guard<std::mutex> lock(cacheAccessLock);
	pathCache[synced]->AddPath(path, result, strtBlock, goalBlock, goalRadius, pathType);
}

void PathingState::AddPathForCurrentFrame(const IPath::Path* path, const IPath::SearchResult result, const int2 strtBlock, const int2 goalBlock, float goalRadius, int pathType, const bool synced)
{
	RECOIL_DETAILED_TRACY_ZONE;
	//const std::lock_guard<std::mutex> lock(cacheAccessLock);
	//pathCache[synced]->AddPathForCurrentFrame(path, result, strtBlock, goalBlock, goalRadius, pathType);
}

void PathingState::PromotePathForCurrentFrame(
		const IPath::Path* path,
		const IPath::SearchResult result,
		const float3 startPosition,
		const float3 goalPosition,
		float goalRadius,
		int pathType,
		const bool synced
	)
{
	RECOIL_DETAILED_TRACY_ZONE;
	int2 strtBlock = {int(startPosition.x / BLOCK_PIXEL_SIZE), int(startPosition.z / BLOCK_PIXEL_SIZE)};;
	int2 goalBlock = {int(goalPosition.x / BLOCK_PIXEL_SIZE), int(goalPosition.z / BLOCK_PIXEL_SIZE)};

	pathCache[synced]->AddPath(path, result, strtBlock, goalBlock, goalRadius, pathType);
}

std::uint32_t PathingState::CalcHash(const char* caller) const
{
	RECOIL_DETAILED_TRACY_ZONE;
	const unsigned int hmChecksum = readMap->CalcHeightmapChecksum();
	const unsigned int tmChecksum = readMap->CalcTypemapChecksum();
	const unsigned int mdChecksum = moveDefHandler.GetCheckSum();
	const unsigned int peHashCode = (hmChecksum + tmChecksum + mdChecksum + BLOCK_SIZE + PATHESTIMATOR_VERSION);

	LOG("[PathingState::%s][%s] BLOCK_SIZE=%u", __func__, caller, BLOCK_SIZE);
	LOG("[PathingState::%s][%s] PATHESTIMATOR_VERSION=%u", __func__, caller, PATHESTIMATOR_VERSION);
	LOG("[PathingState::%s][%s] heightMapChecksum=%x", __func__, caller, hmChecksum);
	LOG("[PathingState::%s][%s] typeMapChecksum=%x", __func__, caller, tmChecksum);
	LOG("[PathingState::%s][%s] moveDefChecksum=%x", __func__, caller, mdChecksum);
	LOG("[PathingState::%s][%s] estimatorHashCode=%x", __func__, caller, peHashCode);

	return peHashCode;
}

}