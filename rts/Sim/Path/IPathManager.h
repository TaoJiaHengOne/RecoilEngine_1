/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */
/* 本文件是 Spring 引擎的一部分 (遵循 GPL v2 或更高版本协议)，详见 LICENSE.html */

#ifndef I_PATH_MANAGER_H
#define I_PATH_MANAGER_H

#include <vector>
#include <cinttypes>

#include "PFSTypes.h"         // 包含了寻路系统（PFS）相关的类型定义。
#include "System/type2.h"     // 包含了 int2, float2 等二维向量类型的定义。
#include "System/float3.h"    // 包含了三维浮点向量 float3 的定义。

// 前向声明
struct MoveDef;       // 定义了移动类型的属性。
class CSolidObject;   // 定义了游戏世界中的固体对象。

/**
 * @brief IPathManager (寻路管理器接口)
 *
 * 这是一个寻路管理器的抽象基类（接口）。
 * 它定义了所有具体寻路系统（如QTPFS、HAPFS）都必须实现的一组通用函数。
 * 游戏的其他部分通过这个接口与当前的寻路系统进行交互，而无需关心其具体实现。
 */
class IPathManager {
public:
	/// 获取寻路管理器实例的静态工厂方法。
	static IPathManager* GetInstance(int type);
	/// 释放寻路管理器实例的静态方法。
	static void FreeInstance(IPathManager*);

	/// 虚析构函数，确保派生类的析构函数能被正确调用。
	virtual ~IPathManager() {}

	/// 获取当前寻路器的类型。
	virtual std::int32_t GetPathFinderType() const { return NOPFS_TYPE; }
	/// 获取路径数据的校验和，用于同步检查。
	virtual std::uint32_t GetPathCheckSum() const { return 0; }

	/// 在游戏加载或初始化结束时调用，用于完成寻路器的最终设置。
	virtual std::int64_t Finalize() { return 0; }
	/// 在最终设置后调用的刷新函数。
	virtual std::int64_t PostFinalizeRefresh() { return 0; }

	/// 返回当前寻路器是否支持定向寻路（即考虑单位的朝向）。
	virtual bool AllowDirectionalPathing() { return false; }
	/// 返回当前寻路器是否允许寻找最短路径（可能会忽略一些成本因素）。
	virtual bool AllowShortestPath() { return false; }

	/**
	 * @brief 检查一条路径在 RequestPath 返回其 pathID 后是否被更改过。
	 * 这种情况可能发生，例如，当一个寻路管理器响应 TerrainChange 事件时
	 * （通过重新请求受影响的路径而不改变它们的ID）。
	 * @return bool 如果路径已更新，返回 true。
	 */
	virtual bool PathUpdated(unsigned int pathID) { return false; }
	/// 清除指定路径的“已更新”标志。
	virtual void ClearPathUpdated(unsigned int pathID) {}

	/// 移除缓存文件。
	virtual void RemoveCacheFiles() {}
	/// 每帧调用的更新函数，用于处理后台的寻路请求等。
	virtual void Update() {}
	/// 更新指定的路径（例如，如果其所有者移动了）。
	virtual void UpdatePath(const CSolidObject* owner, unsigned int pathID) {}

	/**
	 * @brief 当一条路径不再被使用时，调用此函数以从内存中释放它。
	 * @param pathID RequestPath 返回的路径ID。
	 * @param force 是否强制删除。
	 */
	virtual void DeletePath(unsigned int pathID, bool force = false) {}

	/**
	 * @brief 返回路径的下一个路径点。
	 *
	 * @param owner
	 * 使用此路径的单位，可以为 NULL。
	 * @param pathID
	 * RequestPath 返回的路径ID。
	 * @param numRetries
	 * 不要设置此参数，内部使用。
	 * @param callerPos
	 * 路径使用者的当前位置。需要这个额外信息来保持路径与其使用者相连。
	 * @param radius
	 * 可用于设置 callerPos 与返回的路径点之间的最小所需距离。
	 * @param synced
	 * 本次评估是否必须同步运行。
	 * 如果为 false，此调用不能改变任何可能影响未来路径请求的寻路管理器状态。
	 * 例如：如果 (synced == false) 则关闭热点图功能。
	 * @return
	 * 路径的下一个路径点；如果找不到新的路径点，则返回 (-1,-1,-1)。
	 */
	virtual float3 NextWayPoint(
		const CSolidObject* owner,
		unsigned int pathID,
		unsigned int numRetries,
		float3 callerPos,
		float radius,
		bool synced
	) {
		return -OnesVector;
	}

	/// 检查当前路径点是否已变得不可到达。
	virtual bool CurrentWaypointIsUnreachable(unsigned int pathID) { return false; }
	/// 检查下一个路径点是否已变得不可到达。
	virtual bool NextWayPointIsUnreachable(unsigned int pathID) { return false; }


	/**
	 * @brief 返回一条路径的所有路径点。
	 * 一条路径的不同段可能具有不同的分辨率，或者每个段可能以多个不同的分辨率级别表示。
	 * 在前一种情况下，路径点的子集（属于第i个分辨率的路径段）存储在 points[starts[i]] 和 points[starts[i + 1]] 之间。
	 * 而在后一种情况下，所有的路径点（第i个分辨率的完整路径）都存储在 points[starts[i]] 和 points[starts[i + 1]] 之间。
	 *
	 * @param pathID
	 * RequestPath 返回的路径ID。
	 * @param points
	 * 用于存储路径点的向量。
	 * @param starts
	 * 用于存储不同分辨率路径段起始索引的向量。
	 */
	virtual void GetPathWayPoints(
		unsigned int pathID,
		std::vector<float3>& points,
		std::vector<int>& starts
	) const {
	}


	/**
	 * @brief 生成一条从 startPos 到由 (goalPos, goalRadius) 定义的目标的路径。
	 * 如果找不到从 startPos 到 goalPos 的完整路径，则会生成一条尽可能“接近”目标的路径。
	 *
	 * @param caller
	 * 将要使用此路径的单位或地形特征。
	 * @param moveDef
	 * 定义了要使用此路径的单位的移动细节。
	 * @param startPos
	 * 请求路径的起始位置。
	 * @param goalPos
	 * 路径目标区域的中心。
	 * @param goalRadius
	 * 使用 goalRadius 来定义一个目标区域，任何在该区域内的方块都可以被接受为路径目标。
	 * 如果想要一个单一的目标位置，请使用 goalRadius = 0。
	 * @param synced
	 * 本次评估是否必须同步运行。
	 * 如果为 false，此调用不能改变任何可能影响未来路径请求的寻路管理器状态。
	 * 例如：如果 (synced == false) 则关闭热点图功能。
	 * @param immediateResult
	 * 是否需要立即返回结果（可能会阻塞主线程）。
	 * @return
	 * 成功时返回一个 >= 1 的路径ID，失败时返回 0。
	 * 失败意味着找不到比 startPos 更“接近”goalPos 的路径。
	 */
	virtual unsigned int RequestPath(
		CSolidObject* caller,
		const MoveDef* moveDef,
		float3 startPos,
		float3 goalPos,
		float goalRadius,
		bool synced,
		bool immediateResult = false
	) {
		return 0;
	}

	/**
	 * @brief 当地形发生任何变化时（例如：爆炸、新建筑等），此函数将被调用。
	 * @param x1, z1, x2, z2
	 * 定义了受变化影响的矩形区域的对角坐标。
	 * @param type
	 * 变化的类型，参见 @TerrainChangeTypes 枚举。
	 */
	virtual void TerrainChange(unsigned int x1, unsigned int z1, unsigned int x2, unsigned int z2, unsigned int type) {}

	/// 设置整个地图节点的额外寻路成本。
	virtual bool SetNodeExtraCosts(const float* costs, unsigned int sizex, unsigned int sizez, bool synced) { return false; }
	/// 设置单个地图节点的额外寻路成本。
	virtual bool SetNodeExtraCost(unsigned int x, unsigned int z, float cost, bool synced) { return false; }
	/// 获取单个地图节点的额外寻路成本。
	virtual float GetNodeExtraCost(unsigned int x, unsigned int z, bool synced) const { return 0.0f; }
	/// 获取整个地图节点的额外寻路成本数组。
	virtual const float* GetNodeExtraCosts(bool synced) const { return nullptr; }

	/// 获取队列中等待更新的路径数量。
	virtual int2 GetNumQueuedUpdates() const { return (int2(0, 0)); }

	/// 为指定的路径ID保存路径缓存。
	virtual void SavePathCacheForPathId(int pathIdToSave) {};
};

// 指向全局唯一的寻路管理器实例的指针。
extern IPathManager* pathManager;

#endif