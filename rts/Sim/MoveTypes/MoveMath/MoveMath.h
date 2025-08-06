/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#ifndef MOVEMATH_H
#define MOVEMATH_H

#include "Map/ReadMap.h"
#include "Sim/Misc/YardmapStatusEffectsMap.h"
#include "Sim/Objects/SolidObject.h"
#include "System/float3.h"
#include "System/Misc/BitwiseEnum.h"

struct MoveDef;
class CSolidObject;

/*
rts/Sim/MoveTypes/
  ├── MoveMath/                           # 移动数学核心模块
  │   ├── MoveMath.h                     # 主要接口定义
  │   ├── MoveMath.cpp                   # 核心实现（你已经看过）
  │   ├── GroundMoveMath.cpp             # 地面移动实现
  │   ├── HoverMoveMath.cpp              # 悬浮移动实现
  │   └── ShipMoveMath.cpp               # 舰船移动实现
  │
  ├── MoveDefHandler.h/cpp               # 移动定义管理器
  ├── MoveType.h/cpp                     # 移动类型基类
  ├── GroundMoveType.h/cpp               # 地面移动类型
  │
  ├── Systems/                           # 系统级组件
  │   ├── GeneralMoveSystem.h/cpp        # 通用移动系统
  │   ├── GroundMoveSystem.h/cpp         # 地面移动系统
  │   └── UnitTrapCheckSystem.h/cpp      # 单位困住检测系统
  │
  ├── Components/                        # ECS组件
  │   ├── MoveTypesComponents.h          # 移动组件定义
  │   └── MoveTypesEvents.h              # 移动事件定义
  │
  └── Utils/                             # 工具类
      └── UnitTrapCheckUtils.h/cpp       # 单位困住检测工具

  关键依赖文件

  rts/Sim/Misc/
  ├── GroundBlockingObjectMap.h/cpp      # 地面阻挡对象图
  ├── YardmapStatusEffectsMap.h/cpp      # 庭院图状态效果
  └── GlobalSynced.h                     # 全局同步状态

  rts/Map/
  ├── ReadMap.h/cpp                      # 地图读取接口
  ├── Ground.h/cpp                       # 地形查询接口
  └── MapInfo.h/cpp                      # 地图信息（你已经看过）

  rts/System/
  ├── float3.h                           # 3D向量
  ├── type2.h                            # 2D整数向量
  └── Misc/BitwiseEnum.h                 # 位运算枚举
*/

namespace MoveTypes {
	/**
	 * @brief 碰撞查询对象 - 统一的移动碰撞检测接口
	 * 
	 * 这个结构体封装了进行移动碰撞检测所需的所有信息，是整个移动系统的核心查询接口。
	 * 它支持两种使用模式：
	 * 1. 基于现有单位的真实查询（用于实际移动）
	 * 2. 基于MoveDef的虚拟查询（用于路径规划和假设性测试）
	 */
	struct CheckCollisionQuery {
		/// 表示Y坐标不可用的特殊值，用于禁用高度检查
		static constexpr float POS_Y_UNAVAILABLE = std::numeric_limits<float>::infinity();

		// ======================== 构造函数 ========================
		
		/**
		 * @brief 基于现有单位创建碰撞查询对象
		 * @param ref 要查询的单位对象，会复制其位置、移动定义、物理状态等信息
		 * 
		 * 用途：检查现有单位是否能移动到新位置，常用于实际的单位移动逻辑
		 */
		CheckCollisionQuery(const CSolidObject* ref);
		
		/**
		 * @brief 基于MoveDef创建纯虚拟查询对象
		 * @param refMoveDef 移动定义，定义了单位的移动能力和限制
		 * 
		 * 用途：检查某种类型的单位是否能在指定条件下存在，常用于预计算和缓存
		 */
		CheckCollisionQuery(const MoveDef* refMoveDef)
			: moveDef(refMoveDef)
		{}

		/**
		 * @brief 基于MoveDef和测试位置创建虚拟查询对象
		 * @param refMoveDef 移动定义
		 * @param testPos 要测试的世界坐标位置
		 * 
		 * 用途：检查某种类型的单位是否能在特定位置存在，会自动更新高度和物理状态
		 */
		CheckCollisionQuery(const MoveDef* refMoveDef, float3 testPos);

		// ======================== 位置和高度更新方法 ========================
		
		/**
		 * @brief 根据世界坐标更新查询对象的高度信息
		 * @param newPos 新的世界坐标位置
		 * 
		 * 功能：自动将世界坐标转换为地图方格坐标，然后调用重载版本
		 */
		void UpdateElevationForPos(float3 newPos) { UpdateElevationForPos({int(pos.x / SQUARE_SIZE), int(pos.z / SQUARE_SIZE)}); };
		
		/**
		 * @brief 根据地图方格坐标更新查询对象的高度和物理状态
		 * @param sqr 地图方格坐标(x, z)
		 * 
		 * 功能：
		 * - 从高度图获取地形高度
		 * - 根据高度和水线计算正确的Y坐标
		 * - 判断并设置是否在水中的物理状态位
		 */
		void UpdateElevationForPos(int2 sqr);

		// ======================== 物理状态查询和操作方法 ========================
		
		/**
		 * @brief 检查是否设置了指定的物理状态位
		 * @param bit 要检查的物理状态位（如PSTATE_BIT_INWATER）
		 * @return true表示该状态位已设置
		 */
		bool HasPhysicalStateBit(unsigned int bit) const { return ((physicalState & bit) != 0); }
		
		/**
		 * @brief 设置指定的物理状态位
		 * @param bit 要设置的物理状态位
		 * 
		 * 功能：通过位运算安全地设置物理状态，不影响其他状态位
		 */
		void SetPhysicalStateBit(unsigned int bit) { 
			unsigned int ps = physicalState; 
			ps |= ( bit); 
			physicalState = static_cast<CSolidObject::PhysicalState>(ps); 
		}
		
		/**
		 * @brief 清除指定的物理状态位
		 * @param bit 要清除的物理状态位
		 * 
		 * 功能：通过位运算安全地清除物理状态，不影响其他状态位
		 */
		void ClearPhysicalStateBit(unsigned int bit) { 
			unsigned int ps = physicalState; 
			ps &= (~bit); 
			physicalState = static_cast<CSolidObject::PhysicalState>(ps); 
		}
		
		/**
		 * @brief 检查查询对象是否在水中
		 * @return true表示在水中
		 * 
		 * 功能：这是一个便利函数，等价于HasPhysicalStateBit(PSTATE_BIT_INWATER)
		 */
		bool IsInWater() const { return (HasPhysicalStateBit(CSolidObject::PhysicalState::PSTATE_BIT_INWATER)); }
		
		/**
		 * @brief 禁用高度检查功能
		 * 
		 * 功能：将Y坐标设置为特殊值，告诉系统忽略垂直分离检查
		 * 用途：用于某些不需要考虑高度差异的碰撞检测场景
		 */
		void DisableHeightChecks() { pos.y = POS_Y_UNAVAILABLE; }
		
		/**
		 * @brief 检查是否启用了高度检查
		 * @return true表示启用高度检查，false表示已禁用
		 */
		bool IsHeightChecksEnabled() const { return pos.y != MoveTypes::CheckCollisionQuery::POS_Y_UNAVAILABLE; }

		// ======================== 成员变量 ========================
		
		/**
		 * @brief 关联的单位对象指针（可选）
		 * 
		 * 用途：
		 * - 非nullptr：表示这是一个基于真实单位的查询，用于实际移动检测
		 * - nullptr：表示这是一个虚拟查询，仅用于假设性测试
		 * 
		 * 作用：在碰撞检测中用于排除自身，避免单位与自己发生碰撞
		 */
		const CSolidObject* unit = nullptr;
		
		/**
		 * @brief 移动定义指针（必需）
		 * 
		 * 包含的关键信息：
		 * - 占地面积大小 (xsize, zsize)
		 * - 地形限制 (maxSlope, depth, slopeMod)
		 * - 移动类型 (Tank, KBot, Hover, Ship)
		 * - 特殊能力 (isSubmarine, followGround等)
		 * 
		 * 用途：定义了单位的移动能力和各种限制条件
		 */
		const MoveDef* moveDef = nullptr;
		
		/**
		 * @brief 查询位置的3D坐标
		 * 
		 * 坐标系说明：
		 * - X: 地图的X轴方向（通常是东西方向）
		 * - Y: 高度轴（垂直方向），POS_Y_UNAVAILABLE表示禁用高度检查
		 * - Z: 地图的Z轴方向（通常是南北方向）
		 * 
		 * 特殊值：当Y = POS_Y_UNAVAILABLE时，表示禁用高度相关的碰撞检查
		 */
		float3 pos = {0.f, POS_Y_UNAVAILABLE, 0.f};
		
		/**
		 * @brief 物理状态标志位集合
		 * 
		 * 常用状态位：
		 * - PSTATE_BIT_ONGROUND: 在地面上
		 * - PSTATE_BIT_INWATER: 在水中
		 * - PSTATE_BIT_UNDERWATER: 完全在水下
		 * - PSTATE_BIT_FLYING: 在飞行中
		 * 
		 * 用途：影响碰撞检测逻辑，例如潜艇可以穿过水面建筑
		 */
		CSolidObject::PhysicalState physicalState = CSolidObject::PhysicalState(CSolidObject::PhysicalState::PSTATE_BIT_ONGROUND);
		
		/**
		 * @brief 是否在"仅允许离开"区域内
		 * 
		 * 用途：
		 * - 工厂出口区域管理：新生产的单位被允许离开，但其他单位不能进入
		 * - 实现"单行道"逻辑，防止单位堵塞工厂出口
		 * - 支持复杂的区域访问控制策略
		 * 
		 * true: 当前在出口专用区域内，false: 不在此类区域内
		 */
		bool inExitOnlyZone = false;
	};
}


class CMoveMath {
	CR_DECLARE(CMoveMath)

protected:
	static float GroundSpeedMod(const MoveDef& moveDef, float height, float slope);
	static float GroundSpeedMod(const MoveDef& moveDef, float height, float slope, float dirSlopeMod);
	static float HoverSpeedMod(const MoveDef& moveDef, float height, float slope);
	static float HoverSpeedMod(const MoveDef& moveDef, float height, float slope, float dirSlopeMod);
	static float ShipSpeedMod(const MoveDef& moveDef, float height, float slope);
	static float ShipSpeedMod(const MoveDef& moveDef, float height, float slope, float dirSlopeMod);

public:
	// gives the y-coordinate the unit will "stand on"
	static float yLevel(const MoveDef& moveDef, const float3& pos);
	static float yLevel(const MoveDef& moveDef, int xSquare, int zSquare);

public:
	enum BlockTypes {
		/*
			- BLOCK_NONE - 不阻挡
			- BLOCK_MOVING - 移动中的单位
			- BLOCK_MOBILE - 闲置可推动单位
			- BLOCK_MOBILE_BUSY - 忙碌可推动单位
			- BLOCK_STRUCTURE - 结构性阻挡（建筑或不可推动单位）
		*/
		BLOCK_NONE        = 0,
		BLOCK_MOVING      = 1,
		BLOCK_MOBILE      = 2,
		BLOCK_MOBILE_BUSY = 4,
		BLOCK_STRUCTURE   = 8,
		BLOCK_IMPASSABLE  = 24 // := 16 | BLOCK_STRUCTURE;
	};
	typedef Bitwise::BitwiseEnum<BlockTypes> BlockType;


	// returns a speed-multiplier for given position or data
	static float GetPosSpeedMod(const MoveDef& moveDef, unsigned xSquare, unsigned zSquare);
	static float GetPosSpeedMod(const MoveDef& moveDef, unsigned xSquare, unsigned zSquare, float3 moveDir);
	static float GetPosSpeedMod(const MoveDef& moveDef, const float3& pos){
		return (GetPosSpeedMod(moveDef, pos.x / SQUARE_SIZE, pos.z / SQUARE_SIZE));
	}
	static float GetPosSpeedMod(const MoveDef& moveDef, const float3& pos, const float3& moveDir){
		return (GetPosSpeedMod(moveDef, pos.x / SQUARE_SIZE, pos.z / SQUARE_SIZE, moveDir));
	}
	static float GetPosSpeedMod(const MoveDef& moveDef, unsigned squareIndex);

	// tells whether a position is blocked (inaccessible for a given object's MoveDef)
	static inline BlockType IsBlocked(const MoveDef& moveDef, const float3& pos, const CSolidObject* collider, int thread);
	static inline BlockType IsBlocked(const MoveDef& moveDef, int xSquare, int zSquare, const CSolidObject* collider, int thread);
	static BlockType IsBlockedNoSpeedModCheck(const MoveDef& moveDef, int xSquare, int zSquare, const CSolidObject* collider, int thread);
	static BlockType IsBlockedNoSpeedModCheckDiff(const MoveDef& moveDef, int2 prevSqr, int2 newSqr, const CSolidObject* collider, int thread = 0);
	static inline BlockType IsBlockedStructure(const MoveDef& moveDef, int xSquare, int zSquare, const CSolidObject* collider, int thread);

	// checks whether an object (collidee) is non-crushable by the given MoveDef
	static bool CrushResistant(const MoveDef& colliderMD, const CSolidObject* collidee);
	// checks whether an object (collidee) is non-blocking for the given MoveDef
	// (eg. would return true for a submarine's moveDef vs. a surface ship object)
	static bool IsNonBlocking(const CSolidObject* collidee, const MoveTypes::CheckCollisionQuery* collider);

	// check how this unit blocks its squares
	static BlockType ObjectBlockType(const CSolidObject* collidee, const MoveTypes::CheckCollisionQuery* collider);

	// checks if a single square is accessible for any object which uses the given MoveDef
	static BlockType SquareIsBlocked(const MoveDef& moveDef, int xSquare, int zSquare, MoveTypes::CheckCollisionQuery* collider);
	static BlockType SquareIsBlocked(const MoveDef& moveDef, const float3& pos, MoveTypes::CheckCollisionQuery* collider) {
		return (SquareIsBlocked(moveDef, pos.x / SQUARE_SIZE, pos.z / SQUARE_SIZE, collider));
	}
	static BlockType RangeIsBlocked(int xmin, int xmax, int zmin, int zmax, const MoveTypes::CheckCollisionQuery* collider, int thread = 0);
	static BlockType RangeIsBlockedTempNum(int xmin, int xmax, int zmin, int zmax, const MoveTypes::CheckCollisionQuery* collider, int tempNum, int thread);

	static BlockType RangeIsBlockedSt(int xmin, int xmax, int zmin, int zmax, const MoveTypes::CheckCollisionQuery* collider, int magicNumber);
	static BlockType RangeIsBlockedMt(int xmin, int xmax, int zmin, int zmax, const MoveTypes::CheckCollisionQuery* collider, int thread, int magicNumber);

	static void InitRangeIsBlockedHashes();
	static BlockType RangeIsBlockedHashedSt(int xmin, int xmax, int zmin, int zmax, const MoveTypes::CheckCollisionQuery* collider, int magicNumber);
	static BlockType RangeIsBlockedHashedMt(int xmin, int xmax, int zmin, int zmax, const MoveTypes::CheckCollisionQuery* collider, int magicNumber, int thread);

	static void FloodFillRangeIsBlocked(const MoveDef& moveDef, const CSolidObject* collider, const SRectangle& areaToSample, std::vector<std::uint8_t>& results, int thread);

	static bool RangeHasExitOnly(int xmin, int xmax, int zmin, int zmax, const ObjectCollisionMapHelper& object);

public:
	static bool noHoverWaterMove;
	static float waterDamageCost;
};


/* Check if a given square-position is accessible by the MoveDef footprint. */
inline CMoveMath::BlockType CMoveMath::IsBlocked(const MoveDef& moveDef, int xSquare, int zSquare, const CSolidObject* collider, int thread)
{
	if (GetPosSpeedMod(moveDef, xSquare, zSquare) == 0.0f)
		return BLOCK_IMPASSABLE;

	return (IsBlockedNoSpeedModCheck(moveDef, xSquare, zSquare, collider, thread));
}

/* Converts a point-request into a square-positional request. */
inline CMoveMath::BlockType CMoveMath::IsBlocked(const MoveDef& moveDef, const float3& pos, const CSolidObject* collider, int thread)
{
	return (IsBlocked(moveDef, pos.x / SQUARE_SIZE, pos.z / SQUARE_SIZE, collider, thread));
}

inline CMoveMath::BlockType CMoveMath::IsBlockedStructure(const MoveDef& moveDef, int xSquare, int zSquare, const CSolidObject* collider, int thread) {
	return (IsBlockedNoSpeedModCheck(moveDef, xSquare, zSquare, collider, thread) & BLOCK_STRUCTURE);
}

#endif

