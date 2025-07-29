/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#ifndef SOLID_OBJECT_H
#define SOLID_OBJECT_H

#include <bit>

#include "WorldObject.h"
#include "Lua/LuaRulesParams.h"
#include "Rendering/Models/3DModel.h"
#include "Sim/Misc/CollisionVolume.h"
#include "System/Matrix44f.h"
#include "System/type2.h"
#include "System/Ecs/EcsMain.h"
#include "System/Misc/BitwiseEnum.h"
#include "System/Sync/SyncedFloat3.h"
#include "System/Sync/SyncedPrimitive.h"

struct MoveDef;
struct LocalModelPiece;
struct SolidObjectDef;

class DamageArray;
class CUnit;

// 地形改变类型枚举
enum TerrainChangeTypes {
	TERRAINCHANGE_DAMAGE_RECALCULATION = 0, // 常规爆炸或地形改造事件后更新
	TERRAINCHANGE_SQUARE_TYPEMAP_INDEX = 1, // 地图方格的地形类型索引改变后更新 (通过 Lua)
	TERRAINCHANGE_TYPEMAP_SPEED_VALUES = 2, // 地形类型的速度值改变后更新 (通过 Lua)
	TERRAINCHANGE_OBJECT_INSERTED      = 3, // 对象被插入（创建）到地图上时
	TERRAINCHANGE_OBJECT_INSERTED_YM   = 4, // 对象被插入到地图上时（与 yardmap 相关）
	TERRAINCHANGE_OBJECT_DELETED       = 5, // 对象从地图上被删除时
};

// 庭院地图（Yardmap）状态枚举。Yardmap 定义了建筑的占地格（footprint）如何与世界交互。
enum YardmapStates {
	YARDMAP_OPEN         = 0,    // 总是开放 (可通行，可建造)
	YARDMAP_STACKABLE    = 1,    // 可以建造在 YARDMAP_BLOCKED 状态的格子上
	YARDMAP_GEOSTACKABLE = 2,    // 可以建造在 YARDMAP_BLOCKED 状态的格子上，并且需要地热（GEO）
	YARDMAP_YARD         = 4,    // 当“庭院”开放时可通行（例如工厂的门打开）
	YARDMAP_YARDINV      = 8,    // 当“庭院”关闭时可通行
	YARDMAP_UNBUILDABLE  = 16,   // 对移动开放 (可通行，但不可建造)
	YARDMAP_BUILDONLY	 = 32,	 // 对建造开放 (不可通行，但可建造)	
	YARDMAP_EXITONLY     = 64,   // 对进入关闭，对建造关闭（单位只能从这里出来，不能进去）
	YARDMAP_BLOCKED      = 0xFF & ~(YARDMAP_YARDINV|YARDMAP_EXITONLY|YARDMAP_UNBUILDABLE), // 总是阻塞 (不可通行，不可建造)

	// 辅助组合状态
	YARDMAP_YARDBLOCKED  = (YARDMAP_YARD|YARDMAP_EXITONLY|YARDMAP_UNBUILDABLE), // “庭院”相关的阻塞状态组合
	YARDMAP_YARDFREE     = ~(YARDMAP_YARD|YARDMAP_EXITONLY|YARDMAP_UNBUILDABLE), // “庭院”相关的开放状态组合
	YARDMAP_GEO          = YARDMAP_BLOCKED, // 地热点（GEO）在功能上等同于阻塞
};
// 使用 BitwiseEnum 使 YardmapStates 可以进行按位操作
typedef Bitwise::BitwiseEnum<YardmapStates> YardMapStatus;


// CSolidObject 类是游戏中所有具有物理实体和碰撞体积的对象的基类（如单位、建筑、地形特征）。
// 它继承自 CWorldObject。
class CSolidObject: public CWorldObject {
public:
	// 声明该类派生自 CR_BASE，用于引擎的反射和序列化系统。
	CR_DECLARE_DERIVED(CSolidObject)

	// 虚函数，获取该对象的定义（SolidObjectDef）。派生类应重写此函数。
	virtual const SolidObjectDef* GetDef() const { return nullptr; };

	// 物理状态枚举，使用位域表示，一个对象可以同时拥有多个状态。
	enum PhysicalState {
		// NOTE:
		//   {ONGROUND,*WATER} and INAIR are mutually exclusive
		//   {UNDERGROUND,UNDERWATER} are not (and are the only
		//   bits to take radius into account)
		// TODO:
		//   should isDead be on this list for spatial queries?
		// 注意:
		//   {ONGROUND, *WATER} 和 INAIR 是互斥的。
		//   {UNDERGROUND, UNDERWATER} 不是互斥的（并且是唯一考虑了半径的状态）。
		// TODO:
		//   isDead 是否应该也在这里，以便于空间查询？
		PSTATE_BIT_ONGROUND    = (1 << 0), // 在地面上
		PSTATE_BIT_INWATER     = (1 << 1), // 在水中（部分或全部浸没）
		PSTATE_BIT_UNDERWATER  = (1 << 2), // 在水下（完全浸没）
		PSTATE_BIT_UNDERGROUND = (1 << 3), // 在地下
		PSTATE_BIT_INAIR       = (1 << 4), // 在空中
		PSTATE_BIT_INVOID      = (1 << 5), // 在“虚空”中（例如，地图外）

		// special bits for impulse-affected objects that do
		// not get set automatically by UpdatePhysicalState;
		// also used by aircraft to control block / unblock
		// behavior
		// NOTE: FLYING DOES NOT ALWAYS IMPLY INAIR!
		// 用于受冲量影响的对象的特殊位，
		// 不会由 UpdatePhysicalState 自动设置；
		// 也被飞机用来控制其阻塞行为。
		// 注意: FLYING 并不总是意味着 INAIR！
		PSTATE_BIT_MOVING   = (1 <<  6), // 正在移动
		PSTATE_BIT_FLYING   = (1 <<  7), // 正在飞行（特指飞机等具有飞行能力的单位）
		PSTATE_BIT_FALLING  = (1 <<  8), // 正在下落
		PSTATE_BIT_SKIDDING = (1 <<  9), // 正在打滑/漂移
		PSTATE_BIT_CRASHING = (1 << 10), // 正在坠毁
		PSTATE_BIT_BLOCKING = (1 << 11), // 正在阻塞地图（物理上占据空间）
	};
	// 可碰撞状态枚举，使用位域表示。
	enum CollidableState {
		CSTATE_BIT_SOLIDOBJECTS = (1 << 0), // 可与其它固体对象碰撞（即使 PSTATE_BIT_BLOCKING 未设置也可以设置此位！）
		CSTATE_BIT_PROJECTILES  = (1 << 1), // 可与投射物碰撞
		CSTATE_BIT_QUADMAPRAYS  = (1 << 2), // 可与四叉树射线碰撞（用于鼠标选择等）
	};
	// 伤害类型枚举，用于区分对象受到伤害或被摧毁的原因。
	enum DamageType {
		DAMAGE_EXPLOSION_WEAPON    = 0,  // 由武器投射物引发的爆炸 (weaponDefID >= 0)
		DAMAGE_EXPLOSION_DEBRIS    = 1,  // 由碎片投射物引发的爆炸 (weaponDefID < 0)
		DAMAGE_COLLISION_GROUND    = 2,  // 与地面碰撞
		DAMAGE_COLLISION_OBJECT    = 3,  // 与其它对象碰撞
		DAMAGE_EXTSOURCE_FIRE      = 4,  // 外部来源：火
		DAMAGE_EXTSOURCE_WATER     = 5,  // 外部来源：水（岩浆/酸液等）
		DAMAGE_EXTSOURCE_KILLED    = 6,  // 外部来源：被直接杀死
		DAMAGE_EXTSOURCE_CRUSHED   = 7,  // 外部来源：被压碎
		DAMAGE_AIRCRAFT_CRASHED    = 8,  // 飞机坠毁
		DAMAGE_NEGATIVE_HEALTH     = 9,  // 生命值变为负数
		DAMAGE_SELFD_EXPIRED       = 10, // 自毁倒计时结束
		DAMAGE_KILLED_CHEAT        = 11, // 通过作弊码被杀死
		DAMAGE_RECLAIMED           = 12, // 被回收
		DAMAGE_KILLED_OOB          = 13, // 因出界（Out of Bounds）被杀死
		DAMAGE_TRANSPORT_KILLED    = 14, // 因搭载的运输工具被摧毁而被杀死
		DAMAGE_FACTORY_KILLED      = 15, // 因所属的工厂被摧毁而被杀死
		DAMAGE_FACTORY_CANCEL      = 16, // 因工厂取消建造而被销毁
		DAMAGE_UNIT_SCRIPT         = 17, // 由单位脚本造成的伤害
		DAMAGE_KAMIKAZE_ACTIVATED  = 18, // 神风（自杀式攻击）激活
		DAMAGE_CONSTRUCTION_DECAY  = 19, // 建造过程中生命值衰减
		DAMAGE_TURNED_INTO_FEATURE = 20, // 转变为地形特征（例如，残骸）

		// Keep killed by Lua as last index here. This will be exposed as
		// lowest index for games. As we keep killed by Lua as lowest index,
		// games can introduce their own damage types by doing code like
		//
		//      envTypes.CullingStrike      = envTypes.KilledByLua - 1
		//      envTypes.SummonTimerExpired = envTypes.KilledByLua - 2
		//
		// 将 Lua 造成的击杀作为此处的最后一个索引。这将作为最低索引暴露给游戏。
		// 由于我们将其作为最低索引，游戏可以通过如下代码引入自己的伤害类型：
		//
		//      envTypes.CullingStrike      = envTypes.KilledByLua - 1
		//      envTypes.SummonTimerExpired = envTypes.KilledByLua - 2
		//
		DAMAGE_KILLED_LUA = 21
	};
	// 虚析构函数，确保派生类的析构函数能被正确调用。
	virtual ~CSolidObject() {}
	// 在对象加载（例如从存档文件加载）后调用的函数。
	void PostLoad();
	// 为该对象（通常是建筑）增加建造进度。返回 false 表示无法接受建造。
	virtual bool AddBuildPower(CUnit* builder, float amount) { return false; }
	// 对该对象施加伤害。
	virtual void DoDamage(const DamageArray& damages, const float3& impulse, CUnit* attacker, int weaponDefID, int projectileID) {}
	// 对该对象施加一个冲量（例如，来自爆炸）。默认实现是直接增加到速度上。
	virtual void ApplyImpulse(const float3& impulse) { SetVelocityAndSpeed(speed + impulse); }
	// 杀死该对象。
	virtual void Kill(CUnit* killer, const float3& impulse, bool crushed);
	// 获取该对象在地面阻塞图中的 ID。
	virtual int GetBlockingMapID() const { return -1; }
	// 获取该对象的庭院地图（YardMap）指针，用于判断占地格的属性。
	virtual const YardMapStatus* GetBlockMap() const { return nullptr; }
	// 强制移动对象到新位置。
	virtual void ForcedMove(const float3& newPos) {}
	// 强制旋转对象到新的朝向。
	virtual void ForcedSpin(const float3& newDir);
	// 强制旋转对象，使用新的前向和右向向量。
	virtual void ForcedSpin(const float3& newFrontDir, const float3& newRightDir);
	// 根据当前的位置和速度更新对象的物理状态（如在空中、在水里等）。
	virtual void UpdatePhysicalState(float eps);
	// 移动对象。relative 为 true 时，v 是基于对象当前朝向的相对位移。
	void Move(const float3& v, bool relative);

	// this should be called whenever the direction
	// vectors are changed (ie. after a rotation) in
	// eg. movetype code
	// 当方向向量改变时（例如旋转后），应调用此函数来更新 midPos 和 aimPos。
	// 主要在移动类型（movetype）代码中调用。
	void UpdateMidAndAimPos() {
		midPos = GetMidPos();
		aimPos = GetAimPos();
	}
	// 设置 midPos 和 aimPos。relative 为 true 时，mp 和 ap 是相对于对象位置的偏移量。
	void SetMidAndAimPos(const float3& mp, const float3& ap, bool relative) {
		SetMidPos(mp, relative);
		SetAimPos(ap, relative);
	}

	// 使用欧拉角设置方向向量。
	void SetDirVectorsEuler(const float3 angles);
	// 使用 4x4 矩阵设置方向向量。
	void SetDirVectors(const CMatrix44f& matrix) {
		rightdir.x = -matrix[0]; updir.x = matrix[4]; frontdir.x = matrix[ 8];
		rightdir.y = -matrix[1]; updir.y = matrix[5]; frontdir.y = matrix[ 9];
		rightdir.z = -matrix[2]; updir.z = matrix[6]; frontdir.z = matrix[10];
	}
	// 增加航向角。
	void AddHeading(short deltaHeading, bool useGroundNormal, bool useObjectNormal, float dirSmoothing) { SetHeading(heading + deltaHeading, useGroundNormal, useObjectNormal, dirSmoothing); }
	// 设置航向角。
	void SetHeading(short worldHeading, bool useGroundNormal, bool useObjectNormal, float dirSmoothing) {
		heading = worldHeading;

		UpdateDirVectors(useGroundNormal, useObjectNormal, dirSmoothing);
		UpdateMidAndAimPos();
	}

	// update object's <heading> from current frontdir
	// should always be called after a SetDirVectors()
	// 从当前的前向向量 (frontdir) 更新对象的航向角 (heading)。
	// 在调用 SetDirVectors() 之后应该总是调用此函数。
	void SetHeadingFromDirection();
	// 从当前的航向角 (heading) 更新对象的建造朝向 (buildFacing)。
	// update object's <buildFacing> from current heading
	void SetFacingFromHeading();
	// update object's local coor-sys from current <heading>
	// (unlike ForcedSpin which updates from given <updir>)
	// NOTE: movetypes call this directly
	// 从当前的航向角 (heading) 更新对象的本地坐标系。
	// (与 ForcedSpin 不同，后者是从给定的 updir 更新)。
	// 注意: 移动类型（movetypes）会直接调用此函数。
	void UpdateDirVectors(bool useGroundNormal, bool useObjectNormal, float dirSmoothing);
	// 使用给定的上方向向量更新对象的方向向量。
	void UpdateDirVectors(const float3& uDir);
	// 更新上一帧的变换矩阵，用于插值渲染。派生类必须实现。
	virtual void UpdatePrevFrameTransform() = 0;
	// 根据给定的位置 p 和对象的方向向量构造一个变换矩阵。
	CMatrix44f ComposeMatrix(const float3& p) const { return (CMatrix44f(p, -rightdir, updir, frontdir)); }
	// 获取对象的变换矩阵。synced=true 获取同步状态的矩阵。派生类必须实现。
	virtual CMatrix44f GetTransformMatrix(bool synced = false, bool fullread = false) const = 0;
	// 获取碰撞体。如果 lmp 为空，则返回对象的主碰撞体。
	// 否则，如果主碰撞体允许，则返回模型部件 lmp 的碰撞体。
	const CollisionVolume* GetCollisionVolume(const LocalModelPiece* lmp) const {
		if (lmp == nullptr)
			return &collisionVolume;
		if (!collisionVolume.DefaultToPieceTree())
			return &collisionVolume;

		return (lmp->GetCollisionVolume());
	}
	// 获取 Lua 对象材质数据。
	const LuaObjectMaterialData* GetLuaMaterialData() const { return (localModel.GetLuaMaterialData()); }
	      LuaObjectMaterialData* GetLuaMaterialData()       { return (localModel.GetLuaMaterialData()); }
	// 获取在指定帧被最后击中的模型部件。
	const LocalModelPiece* GetLastHitPiece(int frame, int synced = true) const {
		if (frame == pieceHitFrames[synced])
			return hitModelPieces[synced];

		return nullptr;
	}
	// 设置最后被击中的模型部件和帧。
	void SetLastHitPiece(const LocalModelPiece* piece, int frame, int synced = true) {
		hitModelPieces[synced] = piece;
		pieceHitFrames[synced] = frame;
	}


	/**
	 * adds this object to the GroundBlockingMap if and only
	 * if HasCollidableStateBit(CSTATE_BIT_SOLIDOBJECTS), else
	 * does nothing
	 */
	/**
	 * @brief 将此对象添加到地面阻塞图中，当且仅当 HasCollidableStateBit(CSTATE_BIT_SOLIDOBJECTS) 为真时。
	 */
	void Block();
	/**
	 * Removes this object from the GroundBlockingMap if it
	 * is currently marked on it, does nothing otherwise.
	 */
	/**
	 * @brief 如果此对象当前被标记在地面阻塞图中，则将其移除。
	 */
	void UnBlock();
	// 设置对象在阻塞图上的位置。
	void SetMapPos(const int2 mp) {
		mapPos = mp;
		groundBlockPos = pos;
	}


	// these transform a point or vector to object-space
	// 将一个向量从世界空间转换到对象本地空间。
	float3 GetObjectSpaceVec(const float3& v) const { return (      (frontdir * v.z) + (rightdir * v.x) + (updir * v.y)); }
	// 将一个点从世界空间转换到对象本地空间。
	float3 GetObjectSpacePos(const float3& p) const { return (pos + (frontdir * p.z) + (rightdir * p.x) + (updir * p.y)); }
	// 注意：需要先设置 drawPos。将一个点转换到用于绘制的对象本地空间。
	// note: requires drawPos to have been set first
	float3 GetObjectSpaceDrawPos(const float3& p) const { return (drawPos + GetObjectSpaceVec(p)); }
	// 获取用于绘制的模型中点位置（非同步）。
	// unsynced mid-{position,vector}s
	float3 GetMdlDrawMidPos() const { return (GetObjectSpaceDrawPos(WORLD_TO_OBJECT_SPACE * localModel.GetRelMidPos())); }
	// 获取用于绘制的对象中点位置（非同步）。
	float3 GetObjDrawMidPos() const { return (GetObjectSpaceDrawPos(WORLD_TO_OBJECT_SPACE *               relMidPos  )); }

	// 获取对象在地图上的二维坐标（基于 pos）。
	int2 GetMapPos() const { return (GetMapPos(pos)); }
	// 根据给定的世界坐标计算地图二维坐标。
	int2 GetMapPos(const float3& position) const { return GetMapPosStatic(pos, xsize, zsize); }
	// 静态函数：根据给定的世界坐标和尺寸计算地图二维坐标。
	static int2 GetMapPosStatic(const float3& position, int xsize, int zsize);
	// 获取对象的占地面积尺寸。
	float2 GetFootPrint(float scale) const { return {xsize * scale, zsize * scale}; }
	// 计算阻力加速度向量。
	float3 GetDragAccelerationVec(float atmosphericDensity, float waterDensity, float dragCoeff, float frictionCoeff) const;
	// 获取期望的上方向向量，用于调整对象姿态（如贴合地面）。
	float3 GetWantedUpDir(bool useGroundNormal, bool useObjectNormal, float dirSmoothing) const;
	// 获取绘制半径（覆盖基类方法）。
	float GetDrawRadius() const override { return (localModel.GetDrawRadius()); }
	// 计算占地面积的最小外接圆半径。
	float CalcFootPrintMinExteriorRadius(float scale = 1.0f) const;
	// 计算占地面积的最大内切圆半径。
	float CalcFootPrintMaxInteriorRadius(float scale = 1.0f) const;
	// 计算占地面积在坐标轴上的拉伸因子。
	float CalcFootPrintAxisStretchFactor() const;
	// 获取在给定世界坐标位置的地面阻塞掩码。
	YardMapStatus GetGroundBlockingMaskAtPos(float3 gpos) const;
	// 检查对象的占地面积是否在地面上。
	bool FootPrintOnGround() const;
	// 检查对象在阻塞图上的位置是否已改变。
	bool BlockMapPosChanged() const { return (groundBlockPos != pos); }
	// 检查物理状态的便捷函数
	bool IsOnGround   () const { return (HasPhysicalStateBit(PSTATE_BIT_ONGROUND   )); } // 是否在地面上
	bool IsInAir      () const { return (HasPhysicalStateBit(PSTATE_BIT_INAIR      )); } // 是否在空中
	bool IsInWater    () const { return (HasPhysicalStateBit(PSTATE_BIT_INWATER    )); } // 是否在水中
	bool IsUnderWater () const { return (HasPhysicalStateBit(PSTATE_BIT_UNDERWATER )); } // 是否在水下
	bool IsUnderGround() const { return (HasPhysicalStateBit(PSTATE_BIT_UNDERGROUND)); } // 是否在地下
	bool IsInVoid     () const { return (HasPhysicalStateBit(PSTATE_BIT_INVOID     )); } // 是否在虚空中

	bool IsMoving  () const { return (HasPhysicalStateBit(PSTATE_BIT_MOVING  )); } // 是否在移动
	bool IsFlying  () const { return (HasPhysicalStateBit(PSTATE_BIT_FLYING  )); } // 是否在飞行
	bool IsFalling () const { return (HasPhysicalStateBit(PSTATE_BIT_FALLING )); } // 是否在下落
	bool IsSkidding() const { return (HasPhysicalStateBit(PSTATE_BIT_SKIDDING)); } // 是否在打滑
	bool IsCrashing() const { return (HasPhysicalStateBit(PSTATE_BIT_CRASHING)); } // 是否在坠毁
	bool IsBlocking() const { return (HasPhysicalStateBit(PSTATE_BIT_BLOCKING)); } // 是否在阻塞地图
	// 物理状态位操作函数
	// 检查是否拥有某个物理状态位
	bool    HasPhysicalStateBit(unsigned int bit) const { return ((physicalState & bit) != 0); }
	// 设置一个物理状态位
	void    SetPhysicalStateBit(unsigned int bit) { unsigned int ps = physicalState; ps |= ( bit); physicalState = static_cast<PhysicalState>(ps); }
	// 清除一个物理状态位
	void  ClearPhysicalStateBit(unsigned int bit) { unsigned int ps = physicalState; ps &= (~bit); physicalState = static_cast<PhysicalState>(ps); }
	// 压入一个物理状态位（保存到高位）
	void   PushPhysicalStateBit(unsigned int bit) { UpdatePhysicalStateBit(1u << (31u - std::countr_zero(bit)), HasPhysicalStateBit(bit)); }
	// 弹出物理状态位（从高位恢复）
	void    PopPhysicalStateBit(unsigned int bit) { UpdatePhysicalStateBit(bit, HasPhysicalStateBit(1u << (31u - std::countr_zero(bit)))); }
	// 弹出物理状态位（从高位恢复）
	bool UpdatePhysicalStateBit(unsigned int bit, bool set) {
		if (set) {
			SetPhysicalStateBit(bit);
		} else {
			ClearPhysicalStateBit(bit);
		}
		return (HasPhysicalStateBit(bit));
	}
	// 可碰撞状态位操作函数
	// 检查是否拥有某个可碰撞状态位
	bool    HasCollidableStateBit(unsigned int bit) const { return ((collidableState & bit) != 0); }
	// 设置一个可碰撞状态位
	void    SetCollidableStateBit(unsigned int bit) { unsigned int cs = collidableState; cs |= ( bit); collidableState = static_cast<CollidableState>(cs); }
	// 清除一个可碰撞状态位
	void  ClearCollidableStateBit(unsigned int bit) { unsigned int cs = collidableState; cs &= (~bit); collidableState = static_cast<CollidableState>(cs); }
	// 压入一个可碰撞状态位
	void   PushCollidableStateBit(unsigned int bit) { UpdateCollidableStateBit(1u << (31u - std::countr_zero(bit)), HasCollidableStateBit(bit)); }
	// 弹出一个可碰撞状态位
	void    PopCollidableStateBit(unsigned int bit) { UpdateCollidableStateBit(bit, HasCollidableStateBit(1u << (31u - std::countr_zero(bit)))); }
	// 根据布尔值更新可碰撞状态位
	bool UpdateCollidableStateBit(unsigned int bit, bool set) {
		if (set) {
			SetCollidableStateBit(bit);
		} else {
			ClearCollidableStateBit(bit);
		}
		return (HasCollidableStateBit(bit));
	}
	// 设置虚空状态。如果对象在地图外，返回 true。
	bool SetVoidState();
	// 清除虚空状态。
	bool ClearVoidState();
	// 根据布尔值更新虚空状态。
	void UpdateVoidState(bool set);
	// 设置对象的质量。
	virtual void SetMass(float newMass);

private:
	// 私有辅助函数，用于设置中点位置。
	void SetMidPos(const float3& mp, bool relative) {
		if (relative) {
			relMidPos = mp; midPos = GetMidPos();
		} else {
			midPos = mp; relMidPos = midPos - pos;
		}
	}
	// 私有辅助函数，用于设置瞄准点位置。
	void SetAimPos(const float3& ap, bool relative) {
		if (relative) {
			relAimPos = ap; aimPos = GetAimPos();
		} else {
			aimPos = ap; relAimPos = aimPos - pos;
		}
	}
	// 私有辅助函数，获取世界坐标系下的中点位置。
	float3 GetMidPos() const { return (GetObjectSpacePos(relMidPos)); }
	// 私有辅助函数，获取世界坐标系下的瞄准点位置。
	float3 GetAimPos() const { return (GetObjectSpacePos(relAimPos)); }

public:
	// 成员变量
	float health = 0.0f;    // 当前生命值
	float maxHealth = 1.0f; // 最大生命值
 	// 在实体组件系统（ECS）中的实体引用
	entt::entity entityReference = entt::null;
	///< 对象的物理质量（可以通过 SetMass 改变）
	///< the physical mass of this object (can be changed by SetMass)
	float mass = DEFAULT_MASS;
	///< how much MoveDef::crushStrength is required to crush this object (run-time constant)
	///< 摧毁此对象所需的 crushStrength（运行时常量）
	float crushResistance = 0.0f;

	///< whether this object can potentially be crushed during a collision with another object
	///< 此对象是否可能在与其他对象碰撞时被压碎
	bool crushable = false;
	///< whether this object can be moved or not (except perhaps along y-axis, to make it stay on ground)
	///< 此对象是否不可移动（但可能沿 y 轴移动以保持在地面上）
	bool immobile = false;
	///< “庭院”是否开放（例如工厂大门是否打开）
	bool yardOpen = false;

	///< if false, object can be pushed during enemy collisions even when modrules forbid it
	///< 如果为 false，即使 mod 规则禁止，对象在与敌人碰撞时也可以被推动
	bool blockEnemyPushing = true;
	///< if true, map height cannot change under this object (through explosions, etc.)
	///< 如果为 true，此对象下方的地图高度不能改变（通过爆炸等）
	bool blockHeightChanges = false;

	///< if true, object will not be drawn at all (neither as model nor as icon/fartex)
	///< 如果为 true，对象将完全不被绘制（既不作为模型也不作为图标）
	bool noDraw = false;
	///< if true, LuaRules::Draw{Unit, Feature} will be called for this object (UNSYNCED)
	///< 如果为 true，将为此对象调用 LuaRules::Draw{Unit, Feature} (非同步)
	bool luaDraw = false;
	///< if true, unit/feature can not be selected/mouse-picked by a player (UNSYNCED)
	///< 如果为 true，单位/特征不能被玩家选择/鼠标拾取 (非同步)
	bool noSelect = false;
	///< if true, unsynced matrices (transformation + pieceSpaceMat/modelSpaceMat) will be updated unconditionally
	///< 如果为 true，非同步矩阵（变换矩阵 + 部件空间矩阵/模型空间矩阵）将被无条件更新
	bool alwaysUpdateMat = false;

	///< specifies which draw passes will be drawn by the engine
	///< 指定此对象将在哪些绘制通道中由引擎绘制
	uint8_t engineDrawMask = uint8_t(-1);

	///< x-size of this object, according to its footprint (note: rotated depending on buildFacing)
	///< 对象占地面积的 x 尺寸（根据 buildFacing 旋转）
	int xsize = 1;
	///< z-size of this object, according to its footprint (note: rotated depending on buildFacing)
	///< 对象占地面积的 z 尺寸（根据 buildFacing 旋转）
	int zsize = 1;

	///< unrotated x-/z-size of this object, according to its footprint
	///< 对象占地面积的未旋转的 x/z 尺寸
	int2 footprint = {1, 1};

	///< contains the same information as frontdir, but in a short signed integer
	///< 存储与 frontdir 相同的信息，但使用一个有符号短整型（同步）
	SyncedSshort heading = 0;
	///< orientation of footprint, 4 different states
	///< 占地面积的方向，有4种不同的状态（同步）
	SyncedSshort buildFacing = 0;


	///< objects start out non-blocking but fully collidable
	///< SolidObjectDef::collidable controls only the SO-bit
	///<
	///< bitmask indicating current state of this object within the game world
	///< 对象初始时是非阻塞的，但完全可碰撞
	///< SolidObjectDef::collidable 只控制 CSTATE_BIT_SOLIDOBJECTS 位
	///<
	///< 位掩码，指示对象在游戏世界中的当前物理状态
	PhysicalState physicalState = PhysicalState(PSTATE_BIT_ONGROUND);
	///< bitmask indicating which types of objects this object can collide with
	///< 位掩码，指示此对象可以与哪些类型的对象碰撞
	CollidableState collidableState = CollidableState(CSTATE_BIT_SOLIDOBJECTS | CSTATE_BIT_PROJECTILES | CSTATE_BIT_QUADMAPRAYS);


	///< team that "owns" this object
	///< “拥有”此对象的队伍
	int team = 0;
	///< allyteam that this->team is part of
	///< 此对象所属队伍的盟友队伍
	int allyteam = 0;

	// the object could be spawned before the frame start (via cheats) or during the normal sim frame
	// 对象可能在帧开始前（通过作弊）或在正常模拟帧期间生成
	bool prevFrameNeedsUpdate = true;

	///< [i] := frame on which hitModelPieces[i] was last hit
	///< [i] := hitModelPieces[i] 最后被击中的帧
	int pieceHitFrames[2] = {-1, -1};

	///< mobility information about this object (if NULL, object is either static or aircraft)
	///< 此对象的移动信息（如果为 NULL，则对象是静态的或飞机）
	MoveDef* moveDef = nullptr;

	LocalModel localModel; // 对象的本地模型数据（包括部件、动画等）
	CollisionVolume collisionVolume; // 对象的主要碰撞体
	CollisionVolume selectionVolume; // 对象的选择体积（通常比碰撞体大）

	///< pieces that were last hit by a {[0] := unsynced, [1] := synced} projectile
	///< 最近被 {[0] := 非同步, [1] := 同步} 投射物击中的模型部件
	const LocalModelPiece* hitModelPieces[2];

	///< object-local {z,x,y}-axes (in WS)
	///< 对象本地坐标系的 {z,x,y} 轴（在世界坐标系中表示）（同步）
	SyncedFloat3 frontdir =  FwdVector; // 前方向量
	SyncedFloat3 rightdir = -RgtVector; // 右方向量
	SyncedFloat3    updir =   UpVector; // 上方向量

	///< local-space vector from pos to midPos (read from model, used to initialize midPos)
	///< 从 pos 到 midPos 的本地空间向量（从模型读取，用于初始化 midPos）（同步）
	SyncedFloat3 relMidPos;
	///< local-space vector from pos to aimPos (read from model, used to initialize aimPos)
	///< 从 pos 到 aimPos 的本地空间向量（从模型读取，用于初始化 aimPos）（同步）
	SyncedFloat3 relAimPos;
	///< mid-position of model in WS, used as center of mass (etc)
	///< 模型在世界坐标系中的中点位置，用作质心等（同步）
	SyncedFloat3 midPos;
	///< aim-position of model in WS, used by weapons
	///< 模型在世界坐标系中的瞄准点位置，供武器使用（同步）
	SyncedFloat3 aimPos;

	///< current position on GroundBlockingObjectMap
	///< 在地面阻塞对象图上的当前位置
	int2 mapPos;
	///< 对象在设置阻塞时的世界坐标位置
	float3 groundBlockPos;
	///< 阻力缩放系数
	float3 dragScales = OnesVector;

	///< pos + speed * timeOffset (unsynced)
	float3 drawPos;
	///< drawPos + relMidPos (unsynced)
	float3 drawMidPos;
	///< 对象是否可用（例如，未处于建造暂停或失能状态）
	bool objectUsable = true;

	/**
	 * @brief mod 控制的参数
	 * 这是一组在 CreateUnitRulesParams() 中初始化并在游戏过程中可能改变的参数。
	 * 每个参数仅由其 id（即在向量中的索引）唯一标识。
	 * 参数可能有也可能没有名称。
	 */
	LuaRulesParams::Params  modParams;

public:
	static constexpr float DEFAULT_MASS = 1e5f;
	static constexpr float MINIMUM_MASS = 1e0f; // 1.0f
	static constexpr float MAXIMUM_MASS = 1e6f;

	static int deletingRefID;

	static void SetDeletingRefID(int id) { deletingRefID = id; }
	// 返回当前正在被删除的对象的引用ID，
	// 对于单位，这等于 unit->id；对于特征，等于 feature->id + unitHandler.MaxUnits()
	static int GetDeletingRefID() { return deletingRefID; }
};

#endif // SOLID_OBJECT_H
