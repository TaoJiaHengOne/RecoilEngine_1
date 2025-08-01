/* 本文件是 Spring 引擎的一部分 (遵循 GPL v2 或更高版本协议)，详见 LICENSE.html */

#ifndef HAPFS_PATHINGSTATESYSTEM_H // 防止头文件被重复包含
#define HAPFS_PATHINGSTATESYSTEM_H

#include <atomic>
#include <string>
#include <vector>

#include "IPathFinder.h"
#include "PathDataTypes.h"
#include "System/Threading/SpringThreading.h"

#include "Sim/Path/HAPFS/PathEstimator.h"
#include "Sim/Path/HAPFS/PathManager.h"

// 前向声明
struct HAPFSPathDrawer;

namespace HAPFS {

class CPathEstimator;
class CPathFinder;

/**
 * @brief PathingState 类
 * HAPFS 寻路系统的共享数据中心。
 * 它为特定分辨率层级的所有寻路器实例存储和管理共享数据，
 * 包括节点状态、预计算的通行成本、路径缓存等。
 */
class PathingState {
public:
	// 默认构造函数
	PathingState();

	// 初始化函数，配置状态、加载或生成预计算数据
    void Init(std::vector<IPathFinder*> pathFinderlist, PathingState* parentState, unsigned int BLOCK_SIZE, const std::string& peFileName, const std::string& mapFileName);

	// 终止并清理资源，将 blockStates 缓冲区归还给内存池
    void Terminate();

	// 静态清理函数，重置静态计数器
	static void KillStatic();

	// 从全局缓冲池中分配或重用一个 PathNodeStateBuffer
	void AllocStateBuffer();

	// 从缓存目录中移除预计算数据文件
    bool RemoveCacheFile(const std::string& peFileName, const std::string& mapFileName);

	// 获取指定路径类型的最大速度修正系数
    float GetMaxSpeedMod(unsigned int pathType) const { return maxSpeedMods[pathType]; };

	// 获取指定索引的预计算顶点（宏观块连接处）通行成本
    float GetVertexCost(size_t index) const { return vertexCosts[index]; };

	// 获取整个预计算顶点成本向量的常量引用
	const std::vector<float>& GetVertexCosts() const { return vertexCosts; }
	// 获取待更新的块队列的常量引用
	const std::deque<int2>& GetUpdatedBlocks() const { return updatedBlocks; }

	// 用于在宏观块内寻找最佳可通行点的辅助结构体
	struct SOffsetBlock {
		float cost;      // 成本（距离和地形惩罚的组合）
		int2 offset;     // 相对于块左下角的偏移量
		SOffsetBlock(const float _cost, const int x, const int y) : cost(_cost), offset(x,y) {}
	};
	// 获取按成本排序的块内偏移量列表
    const std::vector<SOffsetBlock>& getOffsetBlocksSortedByCost() const { return offsetBlocksSortedByCost; };

	// 块的一维索引与二维坐标之间的转换函数
    int2 BlockIdxToPos(const unsigned idx) const { return int2(idx % mapDimensionsInBlocks.x, idx / mapDimensionsInBlocks.x); }
    int  BlockPosToIdx(const int2 pos) const { return (pos.y * mapDimensionsInBlocks.x + pos.x); }

	// 计算预计算数据的校验和，用于同步检查
	std::uint32_t CalcChecksum() const;
	// 计算哈希码，用于验证缓存文件是否与当前地图/MOD匹配
	std::uint32_t CalcHash(const char* caller) const;

	// 获取当前分辨率层级的块大小
	unsigned int GetBlockSize() const { return BLOCK_SIZE; }
	// 获取当前分辨率层级的块数量
	int2 GetNumBlocks() const { return nbrOfBlocks; }

	// 每帧调用，处理待更新块的增量计算
    void Update();

	// 更新指定数量块的顶点通行成本
	void UpdateVertexPathCosts(int blocksToUpdate);

	/**
	 * @brief 当地图的地面结构发生变化时（例如爆炸或新建筑），调用此函数。
	 * 受影响的矩形区域由 (x1, z1)-(x2, z2) 定义。
	 * 估算器本身将决定是否需要更新该区域。
	 */
	void MapChanged(unsigned int x1, unsigned int z1, unsigned int x2, unsigned int z2);

	/**
	 * @brief 返回一个校验和，可用于检查每个玩家是否拥有相同的路径数据。
	 */
	std::uint32_t GetPathChecksum() const { return pathChecksum; }

    // 可重入 - 返回值不能是引用，以避免竞争条件
	CPathCache::CacheItem GetCache(
		const int2 strtBlock,
		const int2 goalBlock,
		float goalRadius,
		int pathType,
		const bool synced
	) const {
		const std::lock_guard<std::mutex> lock(cacheAccessLock);
		return pathCache[synced]->GetCachedPath(strtBlock, goalBlock, goalRadius, pathType);
	}

    // 可重入，但不是多线程同步安全的
	void AddCache(
		const IPath::Path* path,
		const IPath::SearchResult result,
		const int2 strtBlock,
		const int2 goalBlock,
		float goalRadius,
		int pathType,
		const bool synced
	);

	// 为当前帧添加一条路径到缓存
	void AddPathForCurrentFrame(
		const IPath::Path* path,
		const IPath::SearchResult result,
		const int2 strtBlock,
		const int2 goalBlock,
		float goalRadius,
		int pathType,
		const bool synced
	);

	// 将当前帧的路径提升到长期缓存
	void PromotePathForCurrentFrame(
		const IPath::Path* path,
		const IPath::SearchResult result,
		const float3 startPosition,
		const float3 goalPosition,
		float goalRadius,
		int pathType,
		const bool synced
	);

	// 获取核心的节点状态缓冲区的引用
	PathNodeStateBuffer& GetNodeStateBuffer() { return blockStates; }

private:
	// 友元声明，允许这些类访问本类的私有成员
	friend class HAPFS::CPathManager;
	friend struct ::HAPFSPathDrawer;

	// 初始化估算器（加载或生成预计算数据）
    void InitEstimator(const std::string& peFileName, const std::string& mapFileName);
	// 初始化块相关的内部数据结构
    void InitBlocks();

	// 在多线程中计算块偏移和路径成本
    void CalcOffsetsAndPathCosts(unsigned int threadNum, spring::barrier* pathBarrier);
	// 计算每个宏观块内的最佳可通行点偏移量
    void CalculateBlockOffsets(unsigned int, unsigned int);
	// 估算宏观块之间的通行成本
    void EstimatePathCosts(unsigned int, unsigned int);

	// 寻找一个宏观块内的最佳可通行点（偏移量）
    int2 FindBlockPosOffset(const MoveDef&, unsigned int, unsigned int, int threadNum) const;
	// 计算从一个块出发到所有相邻块的顶点通行成本
    void CalcVertexPathCosts(const MoveDef&, int2, unsigned int threadNum = 0);
	// 计算从一个块到指定方向相邻块的单条顶点通行成本
    void CalcVertexPathCost(const MoveDef&, int2, unsigned int pathDir, unsigned int threadNum = 0);

	// 从文件读取预计算数据
	bool ReadFile(const std::string& peFileName, const std::string& mapFileName);
	// 将预计算数据写入文件
	bool WriteFile(const std::string& peFileName, const std::string& mapFileName);

	// 获取待更新块的数量
	std::size_t getCountOfUpdates() const { return updatedBlocks.size(); }

private:
	// 友元声明
	friend class HAPFS::CPathEstimator;

	// 当前分辨率层级的块大小
    unsigned int BLOCK_SIZE = 0;
	// 块的像素尺寸
	unsigned int BLOCK_PIXEL_SIZE = 0;
	// 每帧需要更新的块数
    unsigned int BLOCKS_TO_UPDATE = 0;

	// 路径数据的校验和
    std::uint32_t pathChecksum = 0;
	// 预计算数据文件的哈希码
    std::uint32_t fileHashCode = 0;

	// 用于保护缓存访问的互斥锁
    mutable std::mutex cacheAccessLock;

	// 块更新惩罚计数器
    int blockUpdatePenalty = 0;
	// 当前 PathingState 实例的索引
	unsigned int instanceIndex = 0;

	// 用于预计算的原子计数器
	std::atomic<std::int64_t> offsetBlockNum = {0};
	std::atomic<std::int64_t> costBlockNum = {0};

	// 用于调试和网络消息的索引
    unsigned int nextOffsetMessageIdx = 0;
    unsigned int nextCostMessageIdx = 0;

	//IPathFinder* parentPathFinder; // 父级 (如果 BLOCK_SIZE 是 16 则为 PF, 如果是 32 则为 PE[16])
	// 指向下一个更低分辨率的 PathingState
    PathingState* nextPathState = nullptr;

	// 路径缓存，[0]用于非同步，[1]用于同步
    CPathCache* pathCache[2];

	// 地图中的块总数
    unsigned int mapBlockCount = 0;
	// 地图的块维度
    int2 mapDimensionsInBlocks = {0, 0};
	// 地图的块数量（同上，冗余存储）
	int2 nbrOfBlocks;

	// 与此状态关联的寻路器实例列表
    std::vector<IPathFinder*> pathFinders;

	// 存储每种移动类型的最大速度修正系数
    std::vector<float> maxSpeedMods;
	// 存储预计算的宏观块之间通行成本的巨大向量
    std::vector<float> vertexCosts;
	// 因地图变化而需要更新的块队列
    std::deque<int2> updatedBlocks;

	// 核心的节点状态缓冲区
    PathNodeStateBuffer blockStates;

	// 用于在更新期间临时存储块信息的结构体
	struct SingleBlock {
		int2 blockPos;
		const MoveDef* moveDef;
		SingleBlock(const int2& pos, const MoveDef* md) : blockPos(pos), moveDef(md) {}
	};

	// 在更新周期中消耗的块列表
    std::vector<SingleBlock> consumedBlocks;
	// 按成本排序的块内偏移量列表，用于快速查找最佳通行点
	std::vector<SOffsetBlock> offsetBlocksSortedByCost;
};

}

#endif