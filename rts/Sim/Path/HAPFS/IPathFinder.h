/* 本文件是 Spring 引擎的一部分 (遵循 GPL v2 或更高版本协议)，详见 LICENSE.html */

#ifndef HAPFS_IPATH_FINDER_H // 防止头文件被重复包含
#define HAPFS_IPATH_FINDER_H

#include <cstdlib>

#include "IPath.h"
#include "PathCache.h"
#include "PathConstants.h"
#include "PathDataTypes.h"

// 前向声明，以避免包含完整的头文件
struct MoveDef;
class CPathFinderDef;
class CSolidObject;

namespace HAPFS {

// 路径规划器接口（抽象基类）
class IPathFinder {
public:
	// 虚析构函数，确保派生类对象可以被正确销毁
	virtual ~IPathFinder() {}

	// 初始化寻路器，传入块的大小（分辨率）
	void Init(unsigned int BLOCK_SIZE);
	// 清理和销毁寻路器
	void Kill();

	// 静态初始化函数，用于设置类级别的共享资源
	static void InitStatic();
	// 静态销毁函数，用于清理类级别的共享资源
	static void KillStatic();

	// 我们持有的已分配内存区域的大小（不包括 sizeof(*this)）
	// (PathManager 存储了 HeatMap 和 FlowMap，所以我们不需要将它们加进来)
	size_t GetMemFootPrint() const { return (blockStates.GetMemFootPrint()); }

	// 获取节点状态缓冲区的引用
	PathNodeStateBuffer& GetNodeStateBuffer() { return blockStates; }

	// 获取块的大小（分辨率）
	unsigned int GetBlockSize() const { return BLOCK_SIZE; }
	// 获取地图上的块总数（二维）
	int2 GetNumBlocks() const { return nbrOfBlocks; }
	// 将块的一维索引转换为二维位置
	int2 BlockIdxToPos(const unsigned idx) const { return int2(idx % nbrOfBlocks.x, idx / nbrOfBlocks.x); }
	// 将块的二维位置转换为一维索引
	int  BlockPosToIdx(const int2 pos) const { return (pos.y * nbrOfBlocks.x + pos.x); }


	/**
	 * @brief 给出从指定起始位置到 CPathFinderDef 中定义的目标的路径（如果存在）。
	 *
	 * 如果没有找到完整的路径，将创建一条尽可能“接近”目标的路径，并返回 SearchResult::OutOfRange。
	 * 只有当找不到比给定起始位置更“近”的位置时，才不会创建路径，并返回 SearchResult::CantGetCloser。
	 *
	 * @param moveDef 定义请求路径的单位的占地面积（footprint）。
	 * @param startPos 路径的起始位置。(投影到(x,z)平面)
	 * @param pfDef 定义搜索目标的对象。也可用于对搜索空间施加约束。
	 * @param path 如果能找到任何路径，它将被生成并放入此结构中。
	 * @param exactPath 覆盖返回“最近”路径的行为。
	 * 如果此选项为true，则仅当路径完全到达 pfDef 中定义的目标时才返回路径。
	 * 所有 SearchResult::OutOfRange 都会被转换为 SearchResult::CantGetCloser。
	 * @param maxNodes 搜索允许分析的最大节点/方格数。
	 * 在CPU消耗至关重要的情况下，可以使用此限制。
	 */
	IPath::SearchResult GetPath(
		const MoveDef& moveDef,
		const CPathFinderDef& pfDef,
		const CSolidObject* owner,
		float3 startPos,
		IPath::Path& path,
		const unsigned int maxNodes
	);

	// 获取更高分辨率的父寻路器（用于分层寻路），基类默认返回nullptr
	virtual IPathFinder* GetParent() { return nullptr; }

protected:
	// 初始化一次新的搜索
	IPath::SearchResult InitSearch(const MoveDef&, const CPathFinderDef&, const CSolidObject* owner);

	// 分配状态缓冲区
	void AllocStateBuffer();
	// 清理上次搜索留下的状态。
	void ResetSearch();

	// 执行“原始”搜索（通常是直线检测），基类提供一个默认的失败实现
	virtual IPath::SearchResult DoRawSearch(const MoveDef&, const CPathFinderDef&, const CSolidObject* owner) { return IPath::Error; }
	// 执行主搜索算法（纯虚函数，必须由派生类实现）
	virtual IPath::SearchResult DoSearch(const MoveDef&, const CPathFinderDef&, const CSolidObject* owner) = 0;

	/**
	 * @brief 测试一个块的可用性和价值，并可能将其添加到开放块的队列中。
	 */
	virtual bool TestBlock(
		const MoveDef& moveDef,
		const CPathFinderDef& pfDef,
		const PathNode* parentSquare,
		const CSolidObject* owner,
		const unsigned int pathOptDir,
		const unsigned int blockStatus,
		float speedMod
	) = 0;

	/**
	 * @brief 重新创建寻路器找到的路径。
	 * 从目标方格开始并向后追踪。
	 *
	 * 同时执行路径点调整，使转弯不全是90度或45度。
	 */
	virtual void FinishSearch(const MoveDef& moveDef, const CPathFinderDef& pfDef, IPath::Path& path) const = 0;


	// 获取路径缓存项（纯虚函数）
	virtual const CPathCache::CacheItem& GetCache(
		const int2 strtBlock,
		const int2 goalBlock,
		float goalRadius,
		int pathType,
		const bool synced
	) const = 0;

	// 添加路径到缓存中（纯虚函数）
	virtual void AddCache(
		const IPath::Path* path,
		const IPath::SearchResult result,
		const int2 strtBlock,
		const int2 goalBlock,
		float goalRadius,
		int pathType,
		const bool synced
	) = 0;

	// 设置搜索的起始块
	virtual bool SetStartBlock(
		const MoveDef& moveDef,
		const CPathFinderDef& peDef,
		const CSolidObject* owner,
		float3 startPos
	);

	// 获取启发式函数值（A*算法中的'h'值）（纯虚函数）
	virtual float GetHeuristic(const MoveDef& moveDef, const CPathFinderDef& pfDef, const int2& square) const = 0;

public:
	// 块的大小（以地图方格为单位），如果大于1，则此寻路器是估算器(Estimator)
	unsigned int BLOCK_SIZE = 0;
	// 块的像素尺寸（BLOCK_SIZE * SQUARE_SIZE）
	unsigned int BLOCK_PIXEL_SIZE = 0;

	// 地图中的块数量（宽度和高度）
	int2 nbrOfBlocks;
	// 起始块的二维坐标
	int2 mStartBlock;

	// 起始块的一维索引
	unsigned int mStartBlockIdx = 0;
	// 目标块的索引（在每次搜索中被设置为最接近目标的块）
	unsigned int mGoalBlockIdx = 0;

	// goalSquareIdx 的启发式函数值
	float mGoalHeuristic = 0.0f;

	// 单次搜索允许探索的最大块数
	unsigned int maxBlocksToBeSearched = 0;
	// 本次搜索已测试的块数
	unsigned int testedBlocks = 0;

	// 此寻路器实例的索引号
	unsigned int instanceIndex = 0;

	// 开放列表节点的内存缓冲池
	PathNodeBuffer openBlockBuffer;
	// 存储所有块（节点）状态的缓冲区
	PathNodeStateBuffer blockStates;
	// 开放列表的优先队列
	PathPriorityQueue openBlocks;

	// 上次搜索中被修改过的块列表，用于高效重置
	std::vector<unsigned int> dirtyBlocks;

	// 指向共享的路径状态缓冲区的指针（主要由PathEstimator使用）
	PathNodeStateBuffer* psBlockStates = nullptr;
};

}

#endif // IPATH_FINDER_H