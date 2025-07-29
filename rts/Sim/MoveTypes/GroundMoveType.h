/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */
/* 本文件是 Spring 引擎的一部分 (遵循 GPL v2 或更高版本协议)，详见 LICENSE.html */

#ifndef GROUNDMOVETYPE_H
#define GROUNDMOVETYPE_H

#include <array>    // 用于 std::array
#include <tuple>    // 用于 std::tuple

#include "MoveType.h"                          // 包含了移动类型的基类 AMoveType。
#include "Sim/Path/IPathController.h"          // 包含了路径控制器接口。
#include "System/Sync/SyncedFloat3.h"          // 包含了同步的三维浮点向量类型。

// 前向声明，以避免不必要的头文件包含
struct UnitDef;         // 定义了单位的静态属性（蓝图）。
struct MoveDef;         // 定义了移动类型的属性。
class CSolidObject;     // 定义了游戏世界中的固体对象。

/**
 * @brief CGroundMoveType 类
 * * 负责处理地面单位的移动逻辑。
 * 它管理着寻路、障碍物躲避、速度、加速度、转弯、打滑以及与其他对象的碰撞。
 * 这是游戏中所有地面单位（如坦克、机器人）的核心移动组件。
 */
class CGroundMoveType : public AMoveType
{
	// 声明该类派生自 CR_BASE，用于引擎的反射和序列化系统。
	CR_DECLARE_DERIVED(CGroundMoveType)

public:
	// 构造函数
	CGroundMoveType(CUnit* owner);
	// 析构函数
	~CGroundMoveType();

	// 定义航向改变原因的常量
	static constexpr int HEADING_CHANGED_NONE = 0; // 航向未改变
	static constexpr int HEADING_CHANGED_MOVE = 1; // 因移动而改变航向
	static constexpr int HEADING_CHANGED_STOP = 2; // 因停止而改变航向
	static constexpr int HEADING_CHANGED_STUN = 3; // 因眩晕而改变航向

	// 用于反射系统的成员数据结构，将成员变量的哈希值映射到其指针
	struct MemberData {
		std::array<std::pair<unsigned int,  bool*>, 3>  bools;
		std::array<std::pair<unsigned int, short*>, 1> shorts;
		std::array<std::pair<unsigned int, float*>, 9> floats;
	};

	void PostLoad(); // 在对象从序列化数据加载后调用
	void* GetPreallocContainer() { return owner; }  // creg: 返回用于内存预分配的容器

	bool Update() override;     // 每帧调用的主更新函数
	void SlowUpdate() override; // 较慢频率调用的更新函数，用于处理不那么紧急的逻辑

	// 决定单位如何移动以执行障碍物躲避和路径跟随。
	// 实际的移动必须推迟到 UpdateUnitPosition() 中执行，因为单位的航向、速度和位置会影响其他单位的障碍物躲避决策。
	// 此函数应是多线程安全的。
	void UpdateTraversalPlan();

	// 根据 UpdateTraversalPlan() 中障碍物躲避和路径跟随的决策来更新单位的移动。
	// 此函数应是多线程安全的。
	void UpdateUnitPosition();

	// 解析 UpdateTraversalPlan() 和 UpdateUnitPosition() 之后必须在单线程中执行的任务。
	void UpdatePreCollisions();

	// 执行单位碰撞检测和解析。实际的移动将在稍后的 Update() 中进行，
	// 因为移动单位会影响这些检查期间的进一步碰撞。所有碰撞事件必须记录在
	// 当前线程相应的 GroundMoveSystemComponent 事件列表中。这些事件将在之后
	// 在调用 Update() 之前单线程地发布。这是为了确保响应碰撞事件的单位
	// 是响应碰撞时的状态，而不是碰撞后的状态。
	// 此函数应是多线程安全的。
	void UpdateCollisionDetections();


	// 更新障碍物躲避逻辑
	void UpdateObstacleAvoidance();

	void StartMovingRaw(const float3 moveGoalPos, float moveGoalRadius) override; // 直接向原始目标点移动（可能不使用寻路）
	void StartMoving(float3 pos, float moveGoalRadius) override; // 开始移动到指定位置
	void StartMoving(float3 pos, float moveGoalRadius, float speed) override { StartMoving(pos, moveGoalRadius); } // 开始移动到指定位置（带速度参数，但当前实现忽略）
	void StopMoving(bool callScript = false, bool hardStop = false, bool cancelRaw = false) override; // 停止移动
	bool IsMovingTowards(const float3& pos, float radius, bool checkProgress) const override { // 检查是否正在向指定位置移动
		return (goalPos == pos * XZVector && goalRadius == radius && (!checkProgress || progressState == Active));
	}

	void KeepPointingTo(float3 pos, float distance, bool aggressive) override; // 保持朝向一个位置
	void KeepPointingTo(CUnit* unit, float distance, bool aggressive) override; // 保持朝向一个单位

	void TestNewTerrainSquare(); // 测试脚下新的地形方格属性
	bool CanApplyImpulse(const float3&) override; // 检查是否可以施加冲量
	void LeaveTransport() override; // 当单位离开运输工具时调用
	void Connect() override; // 连接（例如，附加到另一物体）时调用
	void Disconnect() override; // 断开连接时调用

	void InitMemberPtrs(MemberData* memberData); // 初始化成员指针，用于反射
	bool SetMemberValue(unsigned int memberHash, void* memberValue) override; // 通过哈希值设置成员变量的值

	bool OnSlope(float minSlideTolerance); // 检查单位是否在会滑动的斜坡上
	bool IsReversing() const override { return reversing; } // 是否正在倒车
	bool IsPushResistant() const override { return pushResistant; } // 是否抗推动
	bool IsPushResitanceBlockActive() const override { return pushResistanceBlockActive; } // 抗推动的阻塞效果是否激活
	bool WantToStop() const { return (pathID == 0 && (!useRawMovement || atEndOfPath)); } // 是否想要停止移动

	void TriggerSkipWayPoint() { // 触发跳过当前路径点
		earlyCurrWayPoint.y = -2.0f;
	}
	void TriggerCallArrived() { // 触发调用“已到达”逻辑
		atEndOfPath = true;
		atGoal = true;
		pathingArrived = true;
	}


	// --- Getters ---
	float GetTurnRate() const { return turnRate; }     // 获取最大转向速率
	float GetTurnSpeed() const { return turnSpeed; }   // 获取当前转向速度
	float GetTurnAccel() const { return turnAccel; }   // 获取转向加速度

	float GetAccRate() const { return accRate; }       // 获取加速度
	float GetDecRate() const { return decRate; }       // 获取减速度
	float GetMyGravity() const { return myGravity; }   // 获取单位受到的重力
	float GetOwnerRadius() const { return ownerRadius; } // 获取所属单位的半径

	float GetMaxReverseSpeed() const { return maxReverseSpeed; } // 获取最大倒车速度
	float GetWantedSpeed() const { return wantedSpeed; }     // 获取期望速度
	float GetCurrentSpeed() const { return currentSpeed; }   // 获取当前速度
	float GetDeltaSpeed() const { return deltaSpeed; }       // 获取速度变化量

	float GetCurrWayPointDist() const { return currWayPointDist; } // 获取到当前路径点的距离
	float GetPrevWayPointDist() const { return prevWayPointDist; } // 获取到上一个路径点的距离
	float GetGoalRadius(float s = 0.0f) const override { return (goalRadius + extraRadius * s); } // 获取目标半径

	unsigned int GetPathID() const { return pathID; } // 获取当前路径的ID

	const SyncedFloat3& GetCurrWayPoint() const { return currWayPoint; } // 获取当前路径点
	const SyncedFloat3& GetNextWayPoint() const { return nextWayPoint; } // 获取下一个路径点

	const float3& GetFlatFrontDir() const { return flatFrontDir; } // 获取水平的前方向量
	const float3& GetGroundNormal(const float3&) const; // 获取指定位置的地面法线
	float GetGroundHeight(const float3&) const; // 获取指定位置的地面高度

	void SyncWaypoints() {
		// 同步变量在改变时会触发校验和更新，这很昂贵，所以我们应该
		// 在触发更新前检查是否真的发生了变化。
		if (!currWayPoint.bitExactEquals(earlyCurrWayPoint))
			currWayPoint = earlyCurrWayPoint;
		if (!nextWayPoint.bitExactEquals(earlyNextWayPoint))
			nextWayPoint = earlyNextWayPoint;
	}
	unsigned int GetPathId() { return pathID; } // 获取路径ID

	float GetTurnRadius() { // 计算转弯半径
		const float absTurnSpeed = std::max(0.0001f, math::fabs(turnRate));
		const float framesToTurn = SPRING_CIRCLE_DIVS / absTurnSpeed;
		return std::max((currentSpeed * framesToTurn) * math::INVPI2, currentSpeed * 1.05f);
	}

	bool IsAtGoal() const override { return atGoal; } // 检查是否已到达最终目标
	void OwnerMayBeStuck() { forceStaticObjectCheck = true; }; // 标记所属单位可能卡住了
	void SetMtJobId(int _jobId) { jobId = _jobId; } // 设置多线程作业ID

private:
	// 私有方法
	float3 GetObstacleAvoidanceDir(const float3& desiredDir); // 获取障碍物躲避方向
	float3 Here() const; // 获取单位当前精确位置

	// 如果速度向量和方向向量之间的夹角 > arccos(2*sqSkidSpeedMult-1)/2，则开始打滑
	bool StartSkidding(const float3& vel, const float3& dir) const { return ((SignedSquare(vel.dot(dir)) + 0.01f) < (vel.SqLength() * sqSkidSpeedMult)); }
	// 停止打滑的条件
	bool StopSkidding(const float3& vel, const float3& dir) const { return ((SignedSquare(vel.dot(dir)) + 0.01f) >= (vel.SqLength() * sqSkidSpeedMult)); }
	// 开始飞行的条件（例如，飞跃山头）
	bool StartFlying(const float3& vel, const float3& dir) const { return (vel.dot(dir) > 0.2f); }
	// 停止飞行的条件
	bool StopFlying(const float3& vel, const float3& dir) const { return (vel.dot(dir) <= 0.2f); }

	float Distance2D(CSolidObject* object1, CSolidObject* object2, float marginal = 0.0f); // 计算两个对象间的2D距离

	unsigned int GetNewPath(); // 获取一个新路径

	void SetNextWayPoint(int thread); // 设置下一个路径点
	bool CanSetNextWayPoint(int thread); // 是否可以设置下一个路径点
	void ReRequestPath(bool forceRequest); // 重新请求路径

	void StartEngine(bool callScript); // 启动引擎（视觉/声音效果）
	void StopEngine(bool callScript, bool hardStop = false); // 停止引擎

	void Arrived(bool callScript); // 到达目的地时的处理
	void Fail(bool callScript); // 寻路失败时的处理

	void HandleObjectCollisions(); // 处理对象碰撞
	bool HandleStaticObjectCollision( // 处理与静态对象的碰撞
		CUnit* collider,
		CSolidObject* collidee,
		const MoveDef* colliderMD,
		const float colliderRadius,
		const float collideeRadius,
		const float3& separationVector,
		bool canRequestPath,
		bool checkYardMap,
		bool checkTerrain,
		int curThread
	);

    void HandleUnitCollisions( // 处理与单位的碰撞
        CUnit *collider,
        const float3 &colliderParams,
        const UnitDef *colliderUD,
        const MoveDef *colliderMD,
        int curThread);
    float3 CalculatePushVector(const float3 &colliderParams, const float2 &collideeParams, const bool allowUCO, const float4 &separationVect, CUnit *collider, CUnit *collidee); // 计算推动向量
    void HandleFeatureCollisions( // 处理与地形特征的碰撞
        CUnit *collider,
        const float3 &colliderParams,
        const UnitDef *colliderUD,
        const MoveDef *colliderMD,
        int curThread);

public:
    void SetMainHeading(); // 设置主航向
    void ChangeSpeed(float, bool, bool = false); // 改变速度
	void ChangeHeading(short newHeading); // 改变航向
private:
	// 更多私有方法
	void UpdateSkid(); // 更新打滑状态
	void UpdateControlledDrop(); // 更新受控下落（例如，从高处落下）
	void CheckCollisionSkid(); // 检查碰撞引起的打滑
	void CalcSkidRot(); // 计算打滑旋转

	void AdjustPosToWaterLine(); // 将位置调整到水线
	bool UpdateDirectControl(); // 更新直接控制（非寻路）移动
	void UpdateOwnerAccelAndHeading(); // 更新所属单位的加速度和航向
	void UpdatePos(const CUnit* unit, const float3&, float3& resultantMove, int thread) const; // 更新位置
	void UpdateOwnerPos(const float3&, const float3&); // 更新所属单位的位置
	bool UpdateOwnerSpeed(float oldSpeedAbs, float newSpeedAbs, float newSpeedRaw); // 更新所属单位的速度
	bool OwnerMoved(const short, const float3&, const float3&); // 检查所属单位是否已移动
	bool FollowPath(int thread); // 跟随路径
	bool WantReverse(const float3& wpDir, const float3& ffDir) const; // 判断是否需要倒车
	void SetWaypointDir(const float3& cwp, const float3 &opos); // 设置朝向路径点的方向

private:
	// 成员变量
	GMTDefaultPathController pathController; // 默认的路径控制器实例

	int jobId = 0; // 多线程作业ID

	SyncedFloat3 currWayPoint; // 当前路径点（同步）
	SyncedFloat3 nextWayPoint; // 下一个路径点（同步）

	float3 earlyCurrWayPoint; // （多线程中）提前计算的当前路径点
	float3 earlyNextWayPoint; // （多线程中）提前计算的下一个路径点

	float3 waypointDir;      // 指向当前路径点的方向向量
	float3 flatFrontDir;     // 单位在水平面上的前方向量
	float3 lastAvoidanceDir; // 上一次障碍物躲避的方向
	float3 mainHeadingPos;   // 主航向的目标位置
	float3 skidRotVector;    // 与打滑方向正交的向量，用于旋转

	float turnRate = 0.1f;    // 最大转向速率 (角度单位/帧)
	float turnSpeed = 0.0f;   // 当前转向速度 (角度单位/帧)
	float turnAccel = 0.0f;   // 转向加速度 (角度单位/帧^2)

	float accRate = 0.01f;    // 加速率
	float decRate = 0.01f;    // 减速率
	float myGravity = 0.0f;   // 单位受到的重力

	float maxReverseDist = 0.0f;    // 开始考虑倒车的最大距离
	float minReverseAngle = 0.0f;   // 倒车所需的最小转弯角度
	float maxReverseSpeed = 0.0f;   // 最大倒车速度
	float sqSkidSpeedMult = 0.95f;  // 用于判断打滑的速度乘数平方

	float wantedSpeed = 0.0f;   // 期望达到的速度
	float currentSpeed = 0.0f;  // 当前速度
	float deltaSpeed = 0.0f;    // 速度变化量

	float currWayPointDist = 0.0f; // 到当前路径点的距离
	float prevWayPointDist = 0.0f; // 到上一个路径点的距离

	float goalRadius = 0.0f;    // StartMoving* 传入的原始半径
	float ownerRadius = 0.0f;   // 所属单位的 MoveDef 占地半径
	float extraRadius = 0.0f;   // 如果目标位置有效，为 max(0, ownerRadius - goalRadius)，否则为 0

	float skidRotSpeed = 0.0f;  // 打滑时的旋转速度 (弧度 / (GAME_SPEED 帧))
	float skidRotAccel = 0.0f;  // 打滑时的旋转加速度 (弧度 / (GAME_SPEED 帧^2))

	float3 forceFromMovingCollidees; // 来自移动碰撞物体的力
	float3 forceFromStaticCollidees; // 来自静态碰撞物体的力
	float3 resultantForces;          // 合力

	unsigned int pathID = 0;       // 当前路径的ID
	unsigned int nextPathId = 0;   // 下一个要请求的路径ID
	unsigned int deletePathId = 0; // 需要删除的路径ID

	unsigned int numIdlingUpdates = 0;      // 如果 idling 为 true/false 且 pathId != 0，在每次 Update 中增/减
	unsigned int numIdlingSlowUpdates = 0;  // 如果 idling 为 true/false 且 pathId != 0，在每次 SlowUpdate 中增/减

	short wantedHeading = 0;          // 期望的航向
	short minScriptChangeHeading = 0; // 在调用 script->ChangeHeading 之前所需转动的最小角度

	int wantRepathFrame = std::numeric_limits<int>::min(); // 期望重新寻路的帧
	int lastRepathFrame = std::numeric_limits<int>::min(); // 上次重新寻路的帧
	float bestLastWaypointDist = std::numeric_limits<float>::infinity(); // 到达最后一个路径点的最佳距离记录
	float bestReattemptedLastWaypointDist = std::numeric_limits<float>::infinity(); // 重试后到达最后一个路径点的最佳距离记录
	int setHeading = 0;              // 1 = 常规 (使用 setHeadingDir), 2 = 主航向
	short setHeadingDir = 0;         // 设置的航向方向
	short limitSpeedForTurning = 0;  // 如果设置，将在接下来的 N 个路径点转弯时特别小心，防止过冲。

	float oldSpeed = 0.f; // 旧速度
	float newSpeed = 0.f; // 新速度

	// 状态布尔标志
	bool atGoal = true;          // 是否在最终目标点
	bool atEndOfPath = true;     // 是否在路径的终点
	bool wantRepath = false;     // 是否想要重新寻路
	bool lastWaypoint = false;   // 当前是否是最后一个路径点

	bool reversing = false;      // 是否正在倒车
	bool idling = false;         // 是否处于空闲状态
	bool pushResistant = false;  // 是否抗推动
	bool pushResistanceBlockActive = false; // 抗推动阻塞效果是否激活
	bool canReverse = false;     // 是否能够倒车
	bool useMainHeading = false; // 如果为 true，则转向 mainHeadingPos，直到武器[0]可以尝试瞄准它
	bool useRawMovement = false; // 如果为 true，则直接向目标移动而不调用寻路系统 (与 MoveDef::allowRawMovement 无关)
	bool pathingFailed = false;  // 寻路是否失败
	bool pathingArrived = false; // 寻路是否已到达
	bool positionStuck = false;  // 是否位置卡住
	bool forceStaticObjectCheck = false; // 是否强制进行静态对象检查
	bool avoidingUnits = false; // 是否正在躲避其他单位
};

#endif // GROUNDMOVETYPE_H