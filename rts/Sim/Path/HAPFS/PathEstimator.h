/* 本文件是 Spring 引擎的一部分 (遵循 GPL v2 或更高版本协议)，详见 LICENSE.html */

#ifndef HAPFS_PATHESTIMATOR_H // 防止头文件被重复包含
#define HAPFS_PATHESTIMATOR_H

#include <atomic>
#include <cinttypes>
#include <deque>
#include <string>
#include <vector>

#include "PathingState.h"

#include "IPath.h"
#include "IPathFinder.h"
#include "PathConstants.h"
#include "PathDataTypes.h"
#include "System/float3.h"
//#include "System/Threading/SpringThreading.h"


// 前向声明
struct MoveDef;
class CPathEstimatorDef;
class CPathFinderDef;
class CPathCache;
class CSolidObject;
struct HAPFSPathDrawer;

namespace HAPFS {

class PathingState;
class CPathFinder;

/**
 * @brief CPathEstimator 类
 * 继承自 IPathFinder 接口，是分层寻路系统中的低分辨率寻路器（“路径估算器”）。
 * 它在一个粗糙的宏观网格上进行搜索，以快速找到一条横跨大地图的路径，为高精度寻路提供引导。
 */
class CPathEstimator: public IPathFinder {
public:
	/**
	 * @brief 根据一些参数创建一个新的估算器。
	 * @param pathFinder
	 * 用于精确计算顶点成本的更高分辨率的寻路器。
	 *
	 * @param BSIZE
	 * 估算器的分辨率，以地图方格为单位。
	 *
	 * @param peFileName
	 * 存储预计算数据的磁盘文件名。
	 * 给定的名称会附加到文件名末尾，位于相应地图名称之后。
	 * 例如：PE名称 "pe" + 地图名 "Desert" => "Desert.pe"
	 */
	// 初始化路径估算器
	void Init(IPathFinder*, unsigned int BSIZE, PathingState* ps);
	// 清理和销毁路径估算器
	void Kill();

	/**
	 * @brief 每帧调用
	 */
	//void Update();

	// 获取更高分辨率的父寻路器，重写基类方法
	IPathFinder* GetParent() override { return parentPathFinder; }

	//const std::vector<float>& GetVertexCosts() const { return vertexCosts; }
	//const std::deque<int2>& GetUpdatedBlocks() const { return updatedBlocks; }


protected: // IPathFinder 接口的实现
	// 在两个宏观块之间进行一次高精度搜索，以验证连通性
	IPath::SearchResult DoBlockSearch(const CSolidObject* owner, const MoveDef& moveDef, const int2 s, const int2 g);
	IPath::SearchResult DoBlockSearch(const CSolidObject* owner, const MoveDef& moveDef, const float3 sw, const float3 gw);
	// 执行主A*搜索算法，重写基类方法
	IPath::SearchResult DoSearch(const MoveDef&, const CPathFinderDef&, const CSolidObject* owner) override;

	// 测试一个宏观块（邻居节点），重写基类方法
	bool TestBlock(
		const MoveDef& moveDef,
		const CPathFinderDef& pfDef,
		const PathNode* parentSquare,
		const CSolidObject* owner,
		const unsigned int pathOptDir,
		const unsigned int blockStatus,
		float speedMod
	) override;
	// 在搜索成功后构建最终路径，重写基类方法
	void FinishSearch(const MoveDef& moveDef, const CPathFinderDef& pfDef, IPath::Path& path) const override;

	// 从共享的 PathingState 获取路径缓存项，重写基类方法
	const CPathCache::CacheItem& GetCache(
		const int2 strtBlock,
		const int2 goalBlock,
		float goalRadius,
		int pathType,
		const bool synced
	) const override;

	// 将路径添加到共享的 PathingState 缓存中，重写基类方法
	void AddCache(
		const IPath::Path* path,
		const IPath::SearchResult result,
		const int2 strtBlock,
		const int2 goalBlock,
		float goalRadius,
		int pathType,
		const bool synced
	) override;

	// 设置搜索的起始块，会进行可达性检查，重写基类方法
	bool SetStartBlock(
		const MoveDef& moveDef,
		const CPathFinderDef& peDef,
		const CSolidObject* owner,
		float3 startPos
	) override;

	// 获取启发式函数值（A*算法中的'h'值），重写基类方法
	float GetHeuristic(const MoveDef& moveDef, const CPathFinderDef& pfDef, const int2& square) const override;

private:
	// 初始化估算器的内部状态
	void InitEstimator();
	// 初始化块数据
	void InitBlocks();

	// 使用父寻路器测试一个宏观块是否可以从一个精确的起点到达
	bool TestBlockReachability(
		const MoveDef& moveDef,
		const CPathFinderDef& peDef,
		const CSolidObject* owner,
		const unsigned int testBlockIdx
	);

	// 计算一个哈希值，用于验证预计算数据是否与当前地图和MOD匹配
	std::uint32_t CalcHash(const char* caller) const;

private:
	// 友元声明，允许这些类访问本类的私有成员
	friend class CPathManager;
	friend struct ::HAPFSPathDrawer;

	// 需要更新的块数量
	unsigned int BLOCKS_TO_UPDATE = 0;

	// 用于调试和网络消息的索引
	unsigned int nextOffsetMessageIdx = 0;
	//unsigned int nextCostMessageIdx = 0;

	// 路径数据的校验和
	std::uint32_t pathChecksum = 0;
	// 预计算数据文件的哈希码
	std::uint32_t fileHashCode = 0;

	// 用于预计算过程的原子计数器，保证线程安全
	std::atomic<std::int64_t> offsetBlockNum = {0};
	std::atomic<std::int64_t> costBlockNum = {0};

	// 指向父寻路器（更高分辨率的寻路器）
	IPathFinder* parentPathFinder; // 父级 (如果 BLOCK_SIZE 是 16 则为 PF, 如果是 32 则为 PE[16])
	//CPathEstimator* nextPathEstimator; // 下一个更低分辨率的估算器

	// 用于GetCache函数返回的临时可变缓存项
	mutable CPathCache::CacheItem tempCacheItem;

	// 指向共享的路径状态对象，包含缓存、预计算成本等
	PathingState* pathingState;
};

}

#endif