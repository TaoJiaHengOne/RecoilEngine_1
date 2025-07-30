/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */
/* 这个文件是 Spring 引擎的一部分 (使用 GPL v2 或更高版本许可证), 详见 LICENSE.html */

#ifndef MOVEDEF_HANDLER_H
#define MOVEDEF_HANDLER_H

#include <array>
#include <limits>
#include <string>

#include "System/float3.h"
#include "System/type2.h"
#include "System/UnorderedMap.hpp"
#include "System/creg/creg_cond.h"

class MoveDefHandler;
class CSolidObject;
class CUnit;
class LuaTable;

// 前向声明 MoveTypes 命名空间
namespace MoveTypes {
	// 前向声明 CheckCollisionQuery 类，用于碰撞检测查询
	class CheckCollisionQuery;
}

// 包含与移动定义（MoveDefs）相关的辅助结构体
namespace MoveDefs {
	// 用于跟踪碰撞查询状态的结构体，以避免重复计算
	struct CollisionQueryStateTrack {
		/// 上一次查询的 Y 坐标（通常是地形高度）
		float lastPosY = std::numeric_limits<float>::infinity();
		/// 上一次的入水状态 (-2=未初始化, 0=不在水中, 1=在水中)
		int lastInWater = -2;
		/// 上一次的水中碰撞状态 (-2=未初始化, 0=无碰撞, 1=有碰撞)
		int lastWaterCollisions = -2;
		/// 是否需要刷新碰撞缓存（例如，当高度变化时）
		bool refreshCollisionCache = false;
	};
}

// 定义单位移动能力的结构体
struct MoveDef {
	// 用于 Creg 序列化系统
	CR_DECLARE_STRUCT(MoveDef)

	// 默认构造函数
	MoveDef();
	// 从 Lua 表格构造的构造函数
	MoveDef(const LuaTable& moveDefTable);
	// 禁止拷贝构造函数
	MoveDef(const MoveDef& moveDef) = delete;
	// 默认移动构造函数
	MoveDef(MoveDef&& moveDef) = default;

	// 禁止拷贝赋值操作符
	MoveDef& operator = (const MoveDef& moveDef) = delete;
	// 默认移动赋值操作符
	MoveDef& operator = (MoveDef&& moveDef) = default;

	/// @brief 执行一次“原始”搜索，通常是沿直线检测路径是否可行。
	/// @param nearestSquare 仅当函数返回 true 时，该参数才会被赋予有意义的值，表示最接近目标的有效方格。
	bool DoRawSearch(
		const CSolidObject* collider,
		const MoveDef* md,
		const float3 startPos,
		const float3 endPos,
		float goalRadius,
		bool testTerrain,
		bool testObjects,
		bool centerOnly,
		float* minSpeedModPtr,
		int* maxBlockBitPtr,
		int2* nearestSquare,
		int thread = 0
	) const;
	/// @brief 根据单位在新位置的状态更新碰撞查询对象。
	void UpdateCheckCollisionQuery(MoveTypes::CheckCollisionQuery& collider, MoveDefs::CollisionQueryStateTrack& state, const int2 pos) const;
	/// @brief 测试单位是否可以移动到目标方格范围内。
	bool TestMoveSquareRange(
		const MoveTypes::CheckCollisionQuery& collider,
		const float3 rangeMins,
		const float3 rangeMaxs,
		const float3 testMoveDir,
		bool testTerrain = true,       // 是否检测地形（坡度、地形类型等）
		bool testObjects = true,       // 是否检测物体（建筑、其他单位等）
		bool centerOnly = false,       // 是否只检测中心点
		float* minSpeedModPtr = nullptr, // 用于返回路径上的最低速度系数
		int* maxBlockBitPtr = nullptr, // 用于返回路径上的最大阻挡类型
		int thread = 0
	) const;
	/// @brief 测试单位是否可以移动到单个目标方格。这是一个内联函数，调用 TestMoveSquareRange。
	bool TestMoveSquare(
		const MoveTypes::CheckCollisionQuery& collider,
		const float3 testMovePos,
		const float3 testMoveDir,
		bool testTerrain = true,
		bool testObjects = true,
		bool centerOnly = false,
		float* minSpeedModPtr = nullptr,
		int* maxBlockBitPtr = nullptr,
		int thread = 0
	) const {
		return (TestMoveSquareRange(collider, testMovePos, testMovePos, testMoveDir, testTerrain, testObjects, centerOnly, minSpeedModPtr, maxBlockBitPtr, thread));
	}
	/// @brief 在指定位置测试是否存在物体碰撞。
	bool TestMovePositionForObjects(
		const MoveTypes::CheckCollisionQuery* collider,
		const float3 testMovePos,
		int magicNum,
		int thread
	) const;

	/// @brief 检查一个世界坐标位置是否在“仅允许离开”区域内。
	bool IsInExitOnly(float3 testMovePos) const;
	/// @brief 检查一个地图方格坐标是否在“仅允许离开”区域内。
	bool IsInExitOnly(int x, int z) const;

	/// @brief 判断此移动类型是否像船一样浮在水面上。飞行器和建筑的此属性取决于 UnitDef::floatOnWater。
	bool FloatOnWater() const { return (speedModClass == MoveDef::Hover || speedModClass == MoveDef::Ship); }

	/// @brief 获取单位占地面积（footprint）的尺寸。
	float2 GetFootPrint(float scale) const { return {xsize * scale, zsize * scale}; }

	/// @brief 计算最小外接圆半径（刚好能包围整个 footprint 的圆）。
	float CalcFootPrintMinExteriorRadius(float scale = 1.0f) const;
	/// @brief 计算最大内切圆半径（被 footprint 完整包裹的最大圆）。
	float CalcFootPrintMaxInteriorRadius(float scale = 1.0f) const;
	/// @brief 计算 footprint 的轴向拉伸因子（方形为0，理论上的线形为1）。
	float CalcFootPrintAxisStretchFactor() const;

	/// @brief 根据当前高度（水深）计算速度修正系数。
	float GetDepthMod(float height) const;

	/// @brief 计算此 MoveDef 的校验和，用于同步检查。
	unsigned int CalcCheckSum() const;

	/// @brief 判断单位是否为“复杂潜水单位”，即可以潜水并使用自定义的水线。
	bool IsComplexSubmersible() const {
		return isSubmersible && overrideUnitWaterline;
	};

	/// @brief 获取默认的最小下潜深度。
	static float GetDefaultMinWaterDepth() { return -1e6f; }
	/// @brief 获取默认的最大涉水深度。
	static float GetDefaultMaxWaterDepth() { return +1e6f; }

	/// @brief 决定此 MoveDef 从地形类型中接收哪种速度修正（{tank, kbot, hover, ship}Speed）。
	enum SpeedModClass {
		Tank  = 0, // 坦克
		KBot  = 1, // 机器人
		Hover = 2, // 气垫船
		Ship  = 3  // 舰船
	};
	/// @brief 地形类别，定义单位的活动区域。
	enum TerrainClass {
		Land  = 0, /// 限制在“陆地”（高度 >= 0 的地形）
		Water = 1, /// 限制在“水域”（高度 < 0 的地形）
		Mixed = 2, /// 可以同时存在于高度大于和小于 0 的区域
	};
	/// @brief 深度修正参数的索引。
	enum DepthModParams {
		DEPTHMOD_MIN_HEIGHT = 0, // 开始应用修正的最小深度（正值）
		DEPTHMOD_MAX_HEIGHT = 1, // 达到最大修正的最大深度（正值）
		DEPTHMOD_MAX_SCALE  = 2, // 速度缩放的最大值
		DEPTHMOD_QUA_COEFF  = 3, // 二次多项式系数
		DEPTHMOD_LIN_COEFF  = 4, // 线性多项式系数
		DEPTHMOD_CON_COEFF  = 5, // 常数项系数
		DEPTHMOD_NUM_PARAMS = 6, // 参数总数
	};
	/// @brief 针对移动单位阻挡的寻路速度修正乘数的索引。
	enum SpeedModMults {
		SPEEDMOD_MOBILE_IDLE_MULT = 0, // 对“空闲”单位的乘数
		SPEEDMOD_MOBILE_BUSY_MULT = 1, // 对“繁忙”（有指令但未移动）单位的乘数
		SPEEDMOD_MOBILE_MOVE_MULT = 2, // 对“移动中”单位的乘数
		SPEEDMOD_MOBILE_NUM_MULTS = 3, // 乘数总数
	};

	/// 移动定义的名称
	std::string name;

// 使用1字节对齐，确保结构体紧凑，便于计算校验和
#pragma pack(push, 1)
	/// 速度修正类型
	SpeedModClass speedModClass = MoveDef::Tank;
	/// 地形类别
	TerrainClass terrainClass = MoveDef::Mixed;

	/// 路径类型ID，由 MoveDefHandler 分配
	unsigned int pathType = 0;

	/// footprint（单位占地面积）的尺寸
	int xsize = 0, xsizeh = 0; // x轴尺寸 和 半尺寸
	int zsize = 0, zsizeh = 0; // z轴尺寸 和 半尺寸

	/// 对船是最小下潜深度，对其他单位是最大涉水深度。
	/// 控制移动和（卸载）单位的约束。
	float depth = 0.0f;
	/// 深度修正参数数组
	float depthModParams[DEPTHMOD_NUM_PARAMS];
	/// 单位的高度
	float height = 0.0f;
	/// 可通行的最大坡度
	float maxSlope = 1.0f;
	/// 坡度对速度的修正系数
	float slopeMod = 0.0f;
	/// 碾压强度，决定能摧毁哪些物体
	float crushStrength = 0.0f;
	/// 单位的吃水线高度
	float waterline = 0.0f;

	// 针对被移动单位阻挡的方格，路径规划器使用的速度修正乘数。
	// 这些单位可能分别是 "idle"（静止且无指令）、"busy"（静止但有指令）或 "moving"（移动中）。
	// 注意：
	//     包含一个额外的填充元素，以确保 moveMath 成员在64位平台上以8字节对齐。
	float speedModMults[SPEEDMOD_MOBILE_NUM_MULTS + 1];

	/// 热图路径成本修正系数
	float heatMod = 0.05f;
	/// 流场（flow field）修正系数
	float flowMod = 1.0f;

	/// 每 tick（游戏逻辑帧）产生的热量
	int heatProduced = 30;

	/// 在水中时是否紧贴地面移动？
	bool followGround = true;
	/// 是否是纯粹的水下舰船（潜艇）？
	bool isSubmarine = false;

	// 该单位能否完全潜入水中？
	bool isSubmersible = false;

	/// 如果为 false，将强制使用简单的水下碰撞检测，这可能导致两栖单位出现一些寻路问题。
	/// 例如，无论高度如何，它们都会被水上和水下的障碍物阻挡。
	bool overrideUnitWaterline = true;

	/// 寻路时是否尝试绕开被移动单位占据的方格？
	///
	/// 这个布尔值也作为一个对齐填充字节，这样编译器就不会插入自己的填充字节
	/// (否则 GetCheckSum 将需要跳过这些字节，因为它们从未被初始化)。
	bool avoidMobilesOnPath = true;
	/// 是否允许与地形发生碰撞（例如，撞山）
	bool allowTerrainCollisions = true;
	/// 是否允许方向性寻路（寻路成本考虑单位朝向）
	bool allowDirectionalPathing = false;
	/// 是否允许不经过寻路系统的原始移动
	bool allowRawMovement = false;
	/// 是否偏好最短路径而非最快路径
	bool preferShortestPath = false;

	/// 是否在寻路时留下热图并避开他人留下的热图？
	bool heatMapping = true;
	/// 是否使用流场寻路？
	bool flowMapping = true;
#pragma pack(pop)
};



// 移动定义管理器类
class LuaParser;
class MoveDefHandler
{
	// 用于 Creg 序列化系统
	CR_DECLARE_STRUCT(MoveDefHandler)
public:
	/// 定义了最大移动定义的数量
	constexpr static size_t MAX_MOVE_DEFS = 256;

	/// 从 Lua 解析器加载并初始化所有移动定义
	void Init(LuaParser* defsParser);
	/// 在主仿真循环开始后执行的额外初始化
	void PostSimInit();
	/// 清理并重置 MoveDefHandler 的状态
	void Kill() {
		nameMap.clear(); // 从不迭代，直接清空

		mdCounter = 0;
		mdChecksum = 0;
	}

	/// 通过路径类型ID获取 MoveDef
	MoveDef* GetMoveDefByPathType(unsigned int pathType) { assert(pathType < mdCounter); return &moveDefs[pathType]; }
	/// 通过名称获取 MoveDef
	MoveDef* GetMoveDefByName(const std::string& name);

	/// 获取当前已加载的移动定义数量
	unsigned int GetNumMoveDefs() const { return mdCounter; }
	/// 获取所有移动定义的总校验和
	unsigned int GetCheckSum() const { return mdChecksum; }

	/// 获取所有移动定义中最大的 footprint X 尺寸
	int GetLargestFootPrintXSize() { return largestSize; };
	/// 获取所有移动定义中最大的 footprint 半尺寸
	int GetLargestFootPrintSizeH() { return largestSizeH; };

private:
	/// 存储所有移动定义的数组
	std::array<MoveDef, MAX_MOVE_DEFS> moveDefs;
	/// 从名称哈希到路径类型ID的映射
	spring::unordered_map<unsigned int, int> nameMap;

	/// 当前已加载的移动定义计数器
	unsigned int mdCounter = 0;
	/// 所有移动定义的总校验和
	unsigned int mdChecksum = 0;

	/// 所有移动定义中最大的 footprint 尺寸
	int largestSize = 0;
	/// 所有移动定义中最大的 footprint 半尺寸
	int largestSizeH = 0;
};

// 全局唯一的 MoveDefHandler 实例
extern MoveDefHandler moveDefHandler;

#endif // MOVEDEF_HANDLER_H