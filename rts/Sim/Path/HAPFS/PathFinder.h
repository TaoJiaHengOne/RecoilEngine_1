/* 本文件是 Spring 引擎的一部分 (遵循 GPL v2 或更高版本协议)，详见 LICENSE.html */

#ifndef HAPFS_PATH_FINDER_H // 防止头文件被重复包含
#define HAPFS_PATH_FINDER_H

#include <vector>

#include "IPath.h"
#include "IPathFinder.h"
#include "PathConstants.h"
#include "PathDataTypes.h"
#include "Sim/MoveTypes/MoveMath/MoveMath.h"
#include "Sim/Objects/SolidObject.h"

// 前向声明
struct MoveDef;
class CPathFinderDef;

namespace HAPFS {

/**
 * @brief CPathFinder 类
 * 这是 IPathFinder 接口的具体实现，负责在高分辨率（1x1 地图方格）网格上执行 A* 寻路算法。
 * 它计算的是单位可以精确跟随的最终详细路径。
 */
class CPathFinder: public IPathFinder {
public:
	// 静态初始化函数，用于设置类级别的共享资源（如方向成本数组）
	static void InitStatic();

	// 默认构造函数，延迟初始化
	CPathFinder() = default;
	// 带参数的构造函数，立即进行初始化
	CPathFinder(bool threadSafe) { Init(threadSafe); }

	// 初始化寻路器实例
	void Init(bool threadSafe);
	// 清理和销毁寻路器，调用基类的 Kill 方法
	void Kill() { IPathFinder::Kill(); }

	// 定义一个函数指针类型 BlockCheckFunc，用于检查一个方格是否被阻挡
	typedef CMoveMath::BlockType (*BlockCheckFunc)(const MoveDef&, int, int, const CSolidObject*);

protected:
	/// 执行实际的搜索。
	// 执行“原始”搜索（通常是直线检测），重写基类方法
	IPath::SearchResult DoRawSearch(const MoveDef& moveDef, const CPathFinderDef& pfDef, const CSolidObject* owner) override;
	// 执行主A*搜索算法，重写基类方法
	IPath::SearchResult DoSearch(const MoveDef& moveDef, const CPathFinderDef& pfDef, const CSolidObject* owner) override;

	/**
	 * @brief 测试一个方格的可用性和价值，并可能将其添加到开放方格的队列中。
	 */
	// 重写基类方法，测试一个方格（邻居节点）
	bool TestBlock(
		const MoveDef& moveDef,
		const CPathFinderDef& pfDef,
		const PathNode* parentSquare,
		const CSolidObject* owner,
		const unsigned int pathOptDir,
		const unsigned int blockStatus,
		float speedMod
	) override;
	/**
	 * @brief 重新创建寻路器找到的路径。
	 * 从目标方格开始并向后追踪。
	 *
	 * 同时执行路径点调整，使转弯不全是90度或45度。
	 */
	// 重写基类方法，在搜索成功后构建并平滑最终路径
	void FinishSearch(const MoveDef&, const CPathFinderDef&, IPath::Path&) const override;

	// 获取路径缓存项
	const CPathCache::CacheItem& GetCache(
		const int2 strtBlock,
		const int2 goalBlock,
		float goalRadius,
		int pathType,
		const bool synced
	) const override {
		// 只在估算器(Estimator)中进行缓存！（因为有流场和热力图等动态因素）
		return dummyCacheItem;
	}

	// 添加路径到缓存
	void AddCache(
		const IPath::Path* path,
		const IPath::SearchResult result,
		const int2 strtBlock,
		const int2 goalBlock,
		float goalRadius,
		int pathType,
		const bool synced
	) override { 
		// 空实现，因为高精度寻路器不使用缓存
	}

	// 获取启发式函数值（A*算法中的'h'值），重写基类方法
	float GetHeuristic(const MoveDef& moveDef, const CPathFinderDef& pfDef, const int2& square) const override;

private:
	// 测试一个节点的8个相邻方格
	void TestNeighborSquares(
		const MoveDef& moveDef,
		const CPathFinderDef& pfDef,
		const PathNode* parentSquare,
		const CSolidObject* owner,
		int thread
	);

	/**
	 * @brief 调整找到的路径，在可能的情况下“切角”以使其更平滑。
	 */
	void AdjustFoundPath(
		const MoveDef&,
		IPath::Path&,
		const int2& p1, // 当前点之前的第二个点
		const int2& p2, // 当前点之前的一个点
		const int2& p0  // 当前点
	) const;

	// 内联函数，用于平滑路径中的中间路径点
	inline void SmoothMidWaypoint(
		const int2 testSqr,
		const int2 prevSqr,
		const MoveDef& moveDef,
		IPath::Path& foundPath
	) const;

	// 用于 GetCache 函数返回的虚拟缓存项
	CPathCache::CacheItem dummyCacheItem;
};

}

#endif // PATH_FINDER_H