/* 本文件是 Spring 引擎的一部分 (遵循 GPL v2 或更高版本协议)，详见 LICENSE.html */

#ifndef HAPFS_PATHMANAGER_H // 防止头文件被重复包含
#define HAPFS_PATHMANAGER_H

#include <cinttypes>

#include "Sim/Misc/ModInfo.h"
#include "Sim/Path/IPathManager.h"
#include "IPath.h"
#include "IPathFinder.h"
#include "PathFinderDef.h"
#include "Registry.h"
#include "System/UnorderedMap.hpp"

#include <mutex> // 用于多线程同步

// 前向声明
class CSolidObject;
class PathFlowMap;

class CPathFinderDef;
struct MoveDef;

namespace HAPFS {

class CPathEstimator;
class PathHeatMap;
class PathingState;
class CPathFinder;

/**
 * @brief CPathManager 类
 * 这是 HAPFS (分层 A* 寻路系统) 的核心管理器。
 * 它作为整个寻路系统的公共接口，负责接收寻路请求，
 * 协调不同分辨率的寻路器（CPathFinder 和 CPathEstimator）工作，并管理所有路径的生命周期。
 */
class CPathManager: public IPathManager {
public:
	/**
	 * @brief MultiPath 结构体
	 * 代表一个完整的、多层级的路径请求及其结果。
	 * 它包含了从低分辨率到高分辨率的所有路径段，以及请求的原始参数。
	 */
	struct MultiPath {
		MultiPath(): moveDef(nullptr), caller(nullptr) {}
		MultiPath(const MoveDef* moveDef, const float3& startPos, const float3& goalPos, float goalRadius)
			: searchResult(IPath::Error)
			, start(startPos)
			, peDef(startPos, goalPos, goalRadius, 3.0f, 2000)
			, moveDef(moveDef)
			, caller(nullptr)
		{}

		//MultiPath(const MultiPath& mp) = delete; // 旧的删除拷贝构造函数的注释
		// 拷贝构造函数
		MultiPath(const MultiPath& mp) {
			*this = mp;
		}
		// 移动构造函数
		MultiPath(MultiPath&& mp) { *this = std::move(mp); }

		//MultiPath& operator = (const MultiPath& mp) = delete; // 旧的删除拷贝赋值运算符的注释
		// 拷贝赋值运算符
		MultiPath& operator = (const MultiPath& mp) {
			lowResPath = mp.lowResPath;
			medResPath = mp.medResPath;
			maxResPath = mp.maxResPath;

			searchResult = mp.searchResult;

			start = mp.start;
			finalGoal = mp.finalGoal;

			peDef   = mp.peDef;
			moveDef = mp.moveDef;
			caller  = mp.caller;

			updated = mp.updated;

			return *this;
		}
		// 移动赋值运算符
		MultiPath& operator = (MultiPath&& mp) {
			lowResPath = std::move(mp.lowResPath);
			medResPath = std::move(mp.medResPath);
			maxResPath = std::move(mp.maxResPath);

			searchResult = mp.searchResult;

			start = mp.start;
			finalGoal = mp.finalGoal;

			peDef   = mp.peDef;
			moveDef = mp.moveDef;
			caller  = mp.caller;

			mp.moveDef = nullptr;
			mp.caller  = nullptr;

			updated = mp.updated;

			return *this;
		}

		// 路径
		IPath::Path lowResPath; // 低分辨率路径
		IPath::Path medResPath; // 中等分辨率路径
		IPath::Path maxResPath; // 最高分辨率（详细）路径

		IPath::SearchResult searchResult; // 搜索结果状态

		// 请求定义；start 在构造后是常量
		float3 start;     // 起始位置
		float3 finalGoal; // 最终目标位置

		CCircularSearchConstraint peDef; // 路径估算器使用的圆形搜索约束

		const MoveDef* moveDef; // 请求单位的移动定义

		// 附加信息
		CSolidObject* caller; // 请求寻路的单位

		bool updated = false; // 路径是否被更新过的标志
	};

public:
	// 构造函数
	CPathManager();
	// 析构函数
	~CPathManager();

	// 获取寻路器类型标识
	std::int32_t GetPathFinderType() const override { return HAPFS_TYPE; }
	// 获取路径数据的校验和
	std::uint32_t GetPathCheckSum() const override;

	// 最终化预计算数据（通常在游戏加载时调用）
	std::int64_t Finalize() override;
	// 最终化之后刷新预计算数据
	std::int64_t PostFinalizeRefresh() override;

	// 系统是否支持方向性寻路
	bool AllowDirectionalPathing() override { return true; }

	// 移除缓存文件
	void RemoveCacheFiles() override;
	// 每帧更新管理器状态（如热力图、流场图）
	void Update() override;
	// 更新指定路径的状态（如热力图）
	void UpdatePath(const CSolidObject*, unsigned int) override;
	// 删除一个已存在的路径
	void DeletePath(unsigned int pathID, bool force = false) override;

	// 获取路径上的下一个路径点
	float3 NextWayPoint(
		const CSolidObject* owner,
		unsigned int pathID,
		unsigned int numRetries,
		float3 callerPos,
		float radius,
		bool synced
	) override;

	// 在此系统中未使用，因为路径点被消耗后会返回一个无效点。
	bool CurrentWaypointIsUnreachable(unsigned int pathID) override { return false; }

	// 请求一条新路径，这是外部模块调用寻路的主要入口
	unsigned int RequestPath(
		CSolidObject* caller,
		const MoveDef* moveDef,
		float3 startPos,
		float3 goalPos,
		float goalRadius,
		bool synced,
		bool immediateResult = false // 是否需要立即返回结果（用于同步调用）
	) override;

	/**
	 * @brief 返回最高分辨率路径段的路径点。
	 * @param pathID RequestPath返回的路径ID。
	 * @param points 详细路径点的列表。
	 */
	void GetDetailedPath(unsigned pathID, std::vector<float3>& points) const;

	/**
	 * @brief 以方格列表的形式返回最高分辨率路径段的路径点。
	 * @param pathID RequestPath返回的路径ID。
	 * @param points 详细路径点的方格列表。
	 */
	void GetDetailedPathSquares(unsigned pathID, std::vector<int2>& points) const;

	// 获取路径的所有路径点和分段起始点
	void GetPathWayPoints(unsigned int pathID, std::vector<float3>& points, std::vector<int>& starts) const override;

	// 当地形发生变化时调用，以更新路径数据
	void TerrainChange(unsigned int x1, unsigned int z1, unsigned int x2, unsigned int z2, unsigned int type) override;

	// 动态设置/获取节点的额外寻路成本
	bool SetNodeExtraCost(unsigned int, unsigned int, float, bool) override;
	bool SetNodeExtraCosts(const float*, unsigned int, unsigned int, bool) override;
	float GetNodeExtraCost(unsigned int, unsigned int, bool) const override;
	const float* GetNodeExtraCosts(bool) const override;

	// 获取队列中等待更新的节点数量
	int2 GetNumQueuedUpdates() const override;

	// Getters，用于获取内部组件的访问权限
	const CPathFinder* GetMaxResPF() const;
	const CPathEstimator* GetMedResPE() const;
	const CPathEstimator* GetLowResPE() const;
	const PathingState* GetMedResPS() const;
	const PathingState* GetLowResPS() const;

	const PathFlowMap* GetPathFlowMap() const { return pathFlowMap; }
	const PathHeatMap* GetPathHeatMap() const { return pathHeatMap; }
	int GetPathFinderGroups() const { return pathFinderGroups; }

	const spring::unordered_map<unsigned int, MultiPath>& GetPathMap() const { return pathMap; }

private:
	// 静态初始化函数
	void InitStatic();

	// 发起一次路径请求，返回一个 MultiPath 对象
	MultiPath IssuePathRequest(
		CSolidObject* caller,
		const MoveDef* moveDef,
		float3 startPos,
		float3 goalPos,
		float goalRadius,
		bool synced
	);

	// 扩展当前路径，例如当单位偏离路径时
	MultiPath ExpandCurrentPath(
		const CSolidObject* owner,
		unsigned int pathID,
		unsigned int numRetries,
		float3 callerPos,
		float radius,
		bool extendMedResPath
	);

	// 整理和协调不同层级寻路器的路径结果
	IPath::SearchResult ArrangePath(
		MultiPath* newPath,
		const MoveDef* moveDef,
		const float3& startPos,
		const float3& goalPos,
		CSolidObject* caller
	) const;

	// 通过路径ID获取 MultiPath 对象的指针
	MultiPath* GetMultiPath(int pathID) {return (const_cast<MultiPath*>(GetMultiPathConst(pathID))); }

	// 用于多线程代码 - 必须获取一个副本
	MultiPath GetMultiPathMT(int pathID) const {
		std::lock_guard<std::mutex> lock(pathMapUpdate); // 锁定以保证线程安全
		const auto pi = pathMap.find(pathID);
		if (pi == pathMap.end())
			return MultiPath();
		return pi->second;
	}

	// 用于多线程代码 - 更新一个路径
	void UpdateMultiPathMT(int pathID, MultiPath& updatedPath) {
		std::lock_guard<std::mutex> lock(pathMapUpdate); // 锁定以保证线程安全
		const auto pi = pathMap.find(pathID);
		if (pi != pathMap.end())
			pi->second = updatedPath;
	}

	// 获取 MultiPath 对象的常量指针
	const MultiPath* GetMultiPathConst(int pathID) const {
		assert(!ThreadPool::inMultiThreadedSection); // 确保不在多线程区域调用
		const auto pi = pathMap.find(pathID);
		if (pi == pathMap.end())
			return nullptr;
		return &(pi->second);
	}

	// 将一个新的 MultiPath 存储到 pathMap 中，并返回分配的ID
	unsigned int Store(MultiPath& path) {
		unsigned int assignedId = 0;
		{
			std::lock_guard<std::mutex> lock(pathMapUpdate); // 锁定以保证线程安全
			assignedId = ++nextPathID;
			pathMap[assignedId] = std::move(path);
		}
		return assignedId;
	}


	// 对找到的路径进行最终处理
	static void FinalizePath(MultiPath* path, const float3 startPos, const float3 goalPos, const bool cantGetCloser);

	// 将低分辨率路径细化为最高分辨率路径
	void LowRes2MaxRes(MultiPath& path, const float3& startPos, const CSolidObject* owner, bool synced) const;

	// 检查预计算是否已完成
	bool IsFinalized() const { return finalized; }

	// 为指定路径ID保存路径缓存
	void SavePathCacheForPathId(int pathIdToSave) override;

private:
	// 用于保护 pathMap 访问的互斥锁，确保线程安全
	mutable std::mutex pathMapUpdate;


	// 标志位，表示预计算是否已完成
	bool finalized = false;

	// 指向流场图和热力图的指针
	PathFlowMap* pathFlowMap;
	PathHeatMap* pathHeatMap;

	// 存储所有活动路径的哈希表，键为路径ID
	spring::unordered_map<unsigned int, MultiPath> pathMap;

	// 用于生成下一个唯一的路径ID
	unsigned int nextPathID;

	// 用于动态调整路径状态更新工作负载的变量
	std::int32_t frameNumToRefreshPathStateWorkloadRatio = 0;
	std::uint32_t pathStateWorkloadRatio = 0;

	// 寻路器线程组的数量
	int pathFinderGroups = 0;

	// 指向不同分辨率寻路器和其共享状态的指针
	PathingState* highPriorityResPS; // 高优先级（可能是高分辨率）的路径状态
	PathingState* lowPriorityResPS;  // 低优先级（可能是低分辨率）的路径状态
	CPathEstimator* medResPEs;       // 中等分辨率路径估算器数组
	CPathEstimator* lowResPEs;       // 低分辨率路径估算器数组
	CPathFinder* maxResPFs;          // 最高分辨率寻路器数组

	// 一个包含所有寻路器实例指针的向量，便于统一管理
	std::vector<IPathFinder*> pathFinders;
};

}

#endif