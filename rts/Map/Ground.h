/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */
/* 本文件是 Spring 引擎的一部分 (遵循 GPL v2 或更高版本协议)，详见 LICENSE.html */

#ifndef GROUND_H
#define GROUND_H

#include "System/float3.h" // 包含了三维浮点向量 float3 的定义。
#include "System/type2.h"  // 包含了二维整型向量 int2 的定义。


/**
 * @brief CGround 类
 *
 * 一个静态工具类，提供了查询地形属性（如高度、法线、坡度）
 * 和进行地面碰撞检测的功能。所有成员函数都是静态的，无需创建实例即可调用。
 */
class CGround
{
public:
	/// 类似于 GetHeightReal，但使用最近邻过滤而不是对高度图进行插值。速度更快，但精度较低。
	/// @param x 世界坐标x
	/// @param z 世界坐标z
	/// @param synced true 使用同步状态（用于游戏逻辑），false 使用非同步状态（用于视觉效果）。
	static float GetApproximateHeight(float x, float z, bool synced = true);
	/// 不安全的版本，直接使用整数坐标访问高度图，不进行边界检查。
	static float GetApproximateHeightUnsafe(int x, int z, bool synced = true);
	/// 不安全的指针版本，直接返回指向高度图数据的指针。
	static const float* GetApproximateHeightUnsafePtr(int x, int z, bool synced = true);

	/// 返回指定位置处高于水面的高度，结果被限制为非负值。如果地形低于水面，则返回水面高度。
	static float GetHeightAboveWater(float x, float z, bool synced = true);
	/// 返回指定位置的真实高度，可以低于0。此函数会对高度图进行插值，以获得平滑的高度值。
	static float GetHeightReal(float x, float z, bool synced = true);
	/// 返回指定位置的原始地图高度（未经游戏中的地形改造）。
	static float GetOrigHeight(float x, float z);

	/// 获取水面的平面高度。
	static consteval float GetWaterPlaneLevel() {
		/* 水面高度目前是硬编码的。
		 * 这个函数的存在是为了未来的扩展，因为动态水面是一个常见的需求，
		 * 同时也是为了给这个原本是魔法数字 "0" 的常量一个有意义的名字（包括在Lua中）。
		 * 这个函数是后来才添加的，所以它的使用还不算非常普遍。
		 * 当然，如果你要添加动态水面，请不要犹豫，直接移除 `consteval` 关键字。*/
		return .0f;
	}

	/// 获取指定位置的水面高度。
	static constexpr float GetWaterLevel(float x, float z, bool synced = true) {
		// 目前水是一个平面，所以无论XZ坐标如何，水的高度都一样。
		return GetWaterPlaneLevel();
	}

	/// 获取指定位置的地形坡度（以弧度为单位）。
	static float GetSlope(float x, float z, bool synced = true);
	/// 获取指定位置的地形表面法线向量（一个指向正上方的单位向量）。
	static const float3& GetNormal(float x, float z, bool synced = true);
	/// 获取指定位置处高于水面的地形法线。
	static const float3& GetNormalAboveWater(float x, float z, bool synced = true);
	/// 获取指定位置的平滑地形法线（通过对周围法线进行插值计算得到）。
	static float3 GetSmoothNormal(float x, float z, bool synced = true);

	// --- 以下是使用 float3 作为参数的便利重载版本 ---
	static float GetApproximateHeight(const float3& p, bool synced = true) { return (GetApproximateHeight(p.x, p.z, synced)); }
	static float GetHeightAboveWater(const float3& p, bool synced = true) { return (GetHeightAboveWater(p.x, p.z, synced)); }
	static float GetHeightReal(const float3& p, bool synced = true) { return (GetHeightReal(p.x, p.z, synced)); }
	static float GetOrigHeight(const float3& p) { return (GetOrigHeight(p.x, p.z)); }

	static float GetSlope(const float3& p, bool synced = true) { return (GetSlope(p.x, p.z, synced)); }
	static const float3& GetNormal(const float3& p, bool synced = true) { return (GetNormal(p.x, p.z, synced)); }
	static const float3& GetNormalAboveWater(const float3& p, bool synced = true) { return (GetNormalAboveWater(p.x, p.z, synced)); }
	static float3 GetSmoothNormal(const float3& p, bool synced = true) { return (GetSmoothNormal(p.x, p.z, synced)); }


	/// 计算一条线段与地面的交点。返回交点距离起点的距离，如果没有交点则返回-1。
	static float LineGroundCol(float3 from, float3 to, bool synced = true);
	/// 计算一条射线与地面的交点。
	static float LineGroundCol(const float3 pos, const float3 dir, float len, bool synced = true);
	/// 计算一条射线与一个水平平面的交点。
	static float LinePlaneCol(const float3 pos, const float3 dir, float len, float hgt);
	/// 计算一条射线与地面或水面的交点。
	static float LineGroundWaterCol(const float3 pos, const float3 dir, float len, bool testWater, bool synced = true);

	/// 计算抛物线轨迹与地面的交点。
	static float TrajectoryGroundCol(const float3& trajStartPos, const float3& trajTargetDir, float length, float linCoeff, float qdrCoeff);
	/// 计算模拟的抛物线轨迹（考虑重力）与地面的交点距离。
	static float SimTrajectoryGroundColDist(const float3& startPos, const float3& trajStartDir, const float3& acc, const float2& args);

	/// 获取指定位置所在的地图方格的索引。
	static int GetSquare(const float3& pos);
};

#endif // GROUND_H