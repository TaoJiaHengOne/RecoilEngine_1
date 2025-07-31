/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */
/* 这个文件是 Spring 引擎的一部分 (使用 GPL v2 或更高版本许可证), 详见 LICENSE.html */

#ifndef PATH_CONTROLLER_H
#define PATH_CONTROLLER_H

#include "System/float3.h"

// 前向声明，以避免不必要的头文件包含
class CUnit;
class CFeature;
struct MoveDef;

// provides control inputs for a ground-unit following a path
// 为遵循路径的地面单位提供控制输入
//
// inputs are not required to obey a unit's "physical" limits
// (but doing so is a good idea because they are NOT clamped
// before being applied)
// 输入值不要求必须遵守单位的“物理”限制
// （但这样做是个好主意，因为它们在被应用前不会被强制截断）
//
// TODO: for crowds, track units with the same move order
// 待办事项：对于集群单位，应跟踪具有相同移动指令的单位

/**
 * @class IPathController
 * @brief 路径控制器接口
 *
 * 这是一个抽象基类，定义了单位在遵循路径时如何计算其速度和朝向变化。
 * 移动类型（MoveType）会持有一个该接口的实例，并通过调用其方法来获取每一帧的移动“微调”指令。
 * 这允许将高层的移动决策（如速度和转向）与底层的物理实现分离，并为不同类型的单位或AI行为提供定制化的可能性。
 */
class IPathController {
protected:
	// 构造函数，需要一个所属单位的指针
	IPathController(CUnit* owner_): owner(owner_) {}
	// 虚析构函数，允许派生类正确地被销毁
	virtual ~IPathController() {}

public:
	/**
	 * @brief 计算速度变化量 (加速度/减速度)
	 * 如果一个单位有路径要走（且没有被眩晕、正在建造等），这个函数会在每个模拟帧被调用。
	 * @param pathID 当前路径的ID
	 * @param targetSpeed 单位被允许达到的最大目标速度
	 * @param currentSpeed 单位当前的速度
	 * @param maxAccRate 单位的最大加速度率
	 * @param maxDecRate 单位的最大减速度率
	 * @param wantReverse 单位当前是否想要倒车
	 * @param isReversing 单位上一帧是否正在倒车
	 * @return float 本帧应该施加的速度变化量 (delta-speed)
	 */
	virtual float GetDeltaSpeed(
		unsigned int pathID,
		float targetSpeed,  // max. speed <owner> is ALLOWED to be moving at
		float currentSpeed, // speed <owner> is currently moving at
		float maxAccRate,
		float maxDecRate,
		bool wantReverse,
		bool isReversing
	) const = 0;

	// if a unit has a path to follow (and is not stunned,
	// being built, etc) this gets called every sim-frame
	// 如果一个单位有路径要走（且没有被眩晕、正在建造等），这个函数会在每个模拟帧被调用。
	#if 1
	/**
	 * @brief 计算朝向变化量 (简单模型)
	 * 这是一个没有考虑旋转惯性的简单转向模型。
	 * @param pathID 当前路径的ID
	 * @param newHeading 期望达到的新航向
	 * @param oldHeading 当前的航向
	 * @param maxTurnRate 单位的最大转向速率
	 * @return short 本帧应该转动的航向角度增量
	 */
	virtual short GetDeltaHeading(
		unsigned int pathID,
		short newHeading,
		short oldHeading,
		float maxTurnRate
	) const = 0;
	#endif

	/**
	 * @brief 计算朝向变化量 (带惯性模型)
	 * 这是一个更复杂的转向模型，模拟了旋转的加速度和惯性，使转向更平滑。
	 * @param pathID 当前路径的ID
	 * @param newHeading 期望达到的新航向
	 * @param oldHeading 当前的航向
	 * @param maxTurnSpeed 最大转向速度
	 * @param maxTurnAccel 最大转向加速度
	 * @param turnBrakeDist 转向所需的“刹车距离”（角度）
	 * @param curTurnSpeed 指向当前转向速度的指针，函数会更新这个值
	 * @return short 本帧应该转动的航向角度增量
	 */
	virtual short GetDeltaHeading(
		unsigned int pathID,
		short newHeading,
		short oldHeading,
		float maxTurnSpeed,
		float maxTurnAccel,
		float turnBrakeDist,
		float* curTurnSpeed
	) const = 0;

	// 允许或禁止设置临时目标点
	virtual bool AllowSetTempGoalPosition(unsigned int pathID, const float3& pos) const = 0;
	// 设置临时目标点（通常是路径上的下一个路径点）
	virtual void SetTempGoalPosition(unsigned int pathID, const float3& pos) = 0;
	// 设置最终目标点（路径的终点）
	virtual void SetRealGoalPosition(unsigned int pathID, const float3& pos) = 0;

	// 判断是否应忽略地形对移动的影响（例如，单位在空中时）
	virtual bool IgnoreTerrain(const MoveDef& md, const float3& pos) const = 0;
	// if these return true, the respective collisions are NOT handled
	// 如果这些函数返回 true，相应的碰撞将不会被处理
	virtual bool IgnoreCollision(const CUnit* collider, const CUnit* collidee) const = 0;
	virtual bool IgnoreCollision(const CUnit* collider, const CFeature* collidee) const = 0;

protected:
	// 指向拥有此路径控制器的单位
	CUnit* owner;
};


/**
 * @class GMTDefaultPathController
 * @brief 默认的地面移动类型路径控制器
 *
 * 这是 IPathController 接口的一个具体实现，提供了标准的地面单位移动控制逻辑。
 * 它实现了基本的加减速和转向计算。
 */
class GMTDefaultPathController: public IPathController {
public:
	// 构造函数
	GMTDefaultPathController(CUnit* owner_): IPathController(owner_) {}

	// 实现 IPathController 的纯虚函数
	float GetDeltaSpeed(
		unsigned int pathID,
		float targetSpeed,
		float currentSpeed,
		float maxAccRate,
		float maxDecRate,
		bool wantReverse,
		bool isReversing
	) const;

	#if 1
	short GetDeltaHeading(
		unsigned int pathID,
		short newHeading,
		short oldHeading,
		float maxTurnRate
	) const;
	#endif

	short GetDeltaHeading(
		unsigned int pathID,
		short newHeading,
		short oldHeading,
		float maxTurnSpeed,
		float maxTurnAccel,
		float turnBrakeDist,
		float* curTurnSpeed
	) const;

	// 默认总是允许设置临时目标点
	bool AllowSetTempGoalPosition(unsigned int pathID, const float3& pos) const { return true; }
	// 设置临时目标点（在默认实现中，被错误地赋值给了 realGoalPos，可能是个笔误或历史遗留问题）
	void SetTempGoalPosition(unsigned int pathID, const float3& pos) { realGoalPos = pos; }
	// 设置最终目标点（在默认实现中，被错误地赋值给了 tempGoalPos）
	void SetRealGoalPosition(unsigned int pathID, const float3& pos) { tempGoalPos = pos; }

	// 默认实现中，只有当单位在空中时才忽略地形
	bool IgnoreTerrain(const MoveDef& md, const float3& pos) const;
	// 默认不忽略任何单位碰撞
	bool IgnoreCollision(const CUnit* collider, const CUnit* collidee) const { return false; }
	// 默认不忽略任何地形特征碰撞
	bool IgnoreCollision(const CUnit* collider, const CFeature* collidee) const { return false; }

private:
	// where <owner> ultimately wants to go (its final waypoint)
	// 单位最终想去的地方（它的最终路径点）
	float3 realGoalPos;
	// where <owner> currently wants to go (its next waypoint)
	// 单位当前想去的地方（它的下一个路径点）
	float3 tempGoalPos;
};

#endif
