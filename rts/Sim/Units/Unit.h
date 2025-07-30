/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#ifndef UNIT_H
#define UNIT_H

#include <vector>

#include "Sim/Objects/SolidObject.h"// 包含了单位的基类 CSolidObject。
#include "Sim/Misc/Resource.h"// 包含了资源相关的结构体，如 SResourcePack。
#include "Sim/Weapons/WeaponTarget.h"// 包含了武器目标结构体 SWeaponTarget。
#include "System/Matrix44f.h"// 包含了 4x4 浮点数矩阵的定义。
#include "System/type2.h"// 包含了 int2, float2 等二维向量类型的定义。

// Added only to calculate the size of unit memory buffers.
// change as appropriate if the largest type changes.

// for amtMemBuffer
#include "Sim/MoveTypes/GroundMoveType.h"

// for smtMemBuffer
#include "Sim/MoveTypes/ScriptMoveType.h"

// for caiMemBuffer
#include "Sim/Units/CommandAI/BuilderCAI.h"

// for usMemBuffer
#include "Sim/Units/Scripts/LuaUnitScript.h"


class CPlayer;               // 代表一个玩家。
class CCommandAI;            // 代表单位的指令AI，处理命令队列。
class CGroup;                // 代表一个单位组。
class CMissileProjectile;    // 代表导弹类投射物。
class AMoveType;             // 移动类型的抽象基类。
class CWeapon;               // 代表一个武器。
class CUnitScript;           // 单位脚本的基类，处理动画和特殊逻辑。
class DamageArray;           // 伤害数组。
class DynDamageArray;        // 动态伤害数组。
struct SolidObjectDef;       // 固体对象的静态定义。
struct UnitDef;              // 单位的静态定义（蓝图）。
struct UnitLoadParams;       // 单位加载时使用的参数结构体。
struct SLosInstance;         // 视野实例的结构体。

namespace icon {
	class CIconData;
}

// 视野（LOS）状态位
static constexpr uint8_t LOS_INLOS     = (1 << 0);  // 单位当前在盟友队伍的视野内
static constexpr uint8_t LOS_INRADAR   = (1 << 1);  // 单位当前在盟友队伍的雷达范围内
static constexpr uint8_t LOS_PREVLOS   = (1 << 2);  // 单位先前曾进入过盟友队伍的视野
static constexpr uint8_t LOS_CONTRADAR = (1 << 3);  // 单位自上次进入视野后，一直持续在雷达范围内

static constexpr uint8_t LOS_MASK_SHIFT = 4; // 掩码的位移量

// 视野掩码位 (被掩码的位不会被自动更新)
static constexpr uint8_t LOS_INLOS_MASK     = (LOS_INLOS     << LOS_MASK_SHIFT);  // 不更新 LOS_INLOS
static constexpr uint8_t LOS_INRADAR_MASK   = (LOS_INRADAR   << LOS_MASK_SHIFT);  // 不更新 LOS_INRADAR
static constexpr uint8_t LOS_PREVLOS_MASK   = (LOS_PREVLOS   << LOS_MASK_SHIFT);  // 不更新 LOS_PREVLOS
static constexpr uint8_t LOS_CONTRADAR_MASK = (LOS_CONTRADAR << LOS_MASK_SHIFT);  // 不更新 LOS_CONTRADAR
// 所有状态位的组合
static constexpr uint8_t LOS_ALL_BITS = \
	(LOS_INLOS      | LOS_INRADAR      | LOS_PREVLOS      | LOS_CONTRADAR);
// 所有掩码位的组合
static constexpr uint8_t LOS_ALL_MASK_BITS = \
	(LOS_INLOS_MASK | LOS_INRADAR_MASK | LOS_PREVLOS_MASK | LOS_CONTRADAR_MASK);

/**
 * @brief CUnit 类
 * 这是游戏中所有单位的核心类，无论是建筑、坦克还是飞机。
 * 它继承自 CSolidObject，并添加了战斗、建造、资源管理、运输等复杂的逻辑。
 */
class CUnit : public CSolidObject
{
public:
	// 用于引擎的反射和序列化系统
	CR_DECLARE(CUnit)

	CUnit();  // 构造函数
	virtual ~CUnit(); // 虚析构函数

	static void InitStatic(); // 初始化静态成员

	void SanityCheck() const; // 健全性检查，用于调试
	void UpdatePrevFrameTransform() override; // 更新上一帧的变换矩阵，用于渲染插值

	virtual void PreInit(const UnitLoadParams& params); // 初始化之前调用
	virtual void PostInit(const CUnit* builder); // 初始化之后调用

	virtual void Update(); // 每帧调用的主更新函数
	virtual void SlowUpdate(); // 较慢频率调用的更新函数

	const SolidObjectDef* GetDef() const { return ((const SolidObjectDef*) unitDef); } // 获取单位定义
	virtual void DoDamage(const DamageArray& damages, const float3& impulse, CUnit* attacker, int weaponDefID, int projectileID); // 对单位施加伤害
	virtual void DoWaterDamage(); // 对单位施加来自水（如酸液）的伤害
	virtual void FinishedBuilding(bool postInit); // 建造完成时调用

	void ApplyDamage(CUnit* attacker, const DamageArray& damages, float& baseDamage, float& experienceMod); // 应用伤害的内部逻辑
	void ApplyImpulse(const float3& impulse); // 应用冲量

	bool AttackUnit(CUnit* unit, bool isUserTarget, bool wantManualFire, bool fpsMode = false); // 攻击一个单位
	bool AttackGround(const float3& pos, bool isUserTarget, bool wantManualFire, bool fpsMode = false); // 攻击一个地面位置
	void DropCurrentAttackTarget(); // 取消当前的攻击目标

	int GetBlockingMapID() const { return id; } // 获取在阻塞图中的ID，就是单位ID

	void ChangeLos(int losRad, int airRad); // 改变视野和空中视野半径

	void TurnIntoNanoframe(); // 将单位（通常是建造中的）变为纳米框架

	// negative amount=reclaim, return= true -> build power was successfully applied
	// 负数 amount=回收, 返回 true -> 建造能量被成功应用
	bool AddBuildPower(CUnit* builder, float amount);

	virtual void Activate(); // 激活单位（例如，从关闭状态开启）
	virtual void Deactivate(); // 关闭单位

	void ForcedMove(const float3& newPos); // 强制移动到新位置

	void DeleteScript();// 删除单位脚本
	void EnableScriptMoveType();// 启用脚本移动类型
	void DisableScriptMoveType();// 禁用脚本移动类型

	CMatrix44f GetTransformMatrix(bool synced = false, bool fullread = false) const override final; // 获取变换矩阵

	void DependentDied(CObject* o);// 当依赖的对象死亡时调用

	bool AllowedReclaim(CUnit* builder) const;// 判断是否允许被指定的建造者回收
	// 资源管理函数
	bool UseMetal(float metal);// 消耗金属
	void AddMetal(float metal, bool useIncomeMultiplier = true);// 增加金属
	bool UseEnergy(float energy); // 消耗能量
	void AddEnergy(float energy, bool useIncomeMultiplier = true);// 增加能量
	bool AddHarvestedMetal(float metal);// 增加采集到的金属

	void SetStorage(const SResourcePack& newstorage);// 设置资源存储量
	bool HaveResources(const SResourcePack& res) const;// 检查是否有足够的资源
	bool UseResources(const SResourcePack& res);// 消耗一组资源
	void AddResources(const SResourcePack& res, bool useIncomeMultiplier = true);// 增加一组资源
	bool IssueResourceOrder(SResourceOrder* order);// 发出资源订单

	// 将新的风力信息推送给脚本
	void UpdateWind(float x, float z, float strength);

	void UpdateTransportees();// 更新被运输的单位
	void ReleaseTransportees(CUnit* attacker, bool selfDestruct, bool reclaimed);// 释放所有被运输的单位
	void TransporteeKilled(const CObject* o);// 当被运输的单位被摧毁时调用

	void AddExperience(float exp);// 增加经验值

	void SetMass(float newMass);// 设置质量

	void DoSeismicPing(float pingSize);// 发出地震波 ping

	void CalculateTerrainType();// 计算地形类型
	void UpdateTerrainType();// 更新地形类型
	void UpdatePhysicalState(float eps); // 更新物理状态

	float3 GetErrorVector(int allyteam) const;// 获取误差向量，用于雷达不精确等
	// 获取带误差的位置f
	float3 GetErrorPos(int allyteam, bool aiming = false) const { return (aiming? aimPos: midPos) + GetErrorVector(allyteam); }
	// 获取用于绘制的带误差的位置
	float3 GetObjDrawErrorPos(int allyteam) const { return (GetObjDrawMidPos() + GetErrorVector(allyteam)); }
	// 获取给Lua脚本使用的误差向量
	float3 GetLuaErrorVector(int allyteam, bool fullRead) const { return (fullRead? ZeroVector: GetErrorVector(allyteam)); }
	// 获取给Lua脚本使用的带误差的位置
	float3 GetLuaErrorPos(int allyteam, bool fullRead) const { return (midPos + GetLuaErrorVector(allyteam, fullRead)); }
 	// 更新位置误差参数
	void UpdatePosErrorParams(bool updateError, bool updateDelta);
	// 状态检查函数
	bool UsingScriptMoveType() const { return (prevMoveType != nullptr); } // 是否正在使用脚本移动类型
	bool UnderFirstPersonControl() const { return (fpsControlPlayer != nullptr); } // 是否处于第一人称控制下

	bool FloatOnWater() const; // 单位是否能浮在水上

	bool IsNeutral() const { return neutral; } // 是否是中立单位
	bool IsCloaked() const { return isCloaked; } // 是否已隐形
	bool IsStunned() const { return stunned; } // 是否被眩晕
	bool IsIdle() const; // 是否处于空闲状态

	bool HaveTarget() const { return (curTarget.type != Target_None); } // 是否有目标
	bool CanUpdateWeapons() const { // 是否可以更新武器状态（开火等）
		return (forceUseWeapons || (allowUseWeapons && !onTempHoldFire && !isDead && !beingBuilt && !IsStunned()));
	}

	void SetNeutral(bool b); // 设置中立状态
	void SetStunned(bool stun); // 设置眩晕状态
	// 获取位置误差位
	bool GetPosErrorBit(int at) const {
		return (posErrorMask[at / 32] & (1 << (at % 32)));
	}
	// 设置位置误差位
	void SetPosErrorBit(int at, int bit) {
		posErrorMask[at / 32] |=  ((1 << (at % 32)) * (bit == 1));
		posErrorMask[at / 32] &= ~((1 << (at % 32)) * (bit == 0));
	}
 	// 检查对于指定盟友队伍是否在视野内
	bool IsInLosForAllyTeam(int allyTeam) const { return ((losStatus[allyTeam] & LOS_INLOS) != 0); }
	// 设置视野状态
	void SetLosStatus(int allyTeam, unsigned short newStatus);
	// 计算视野状态
	unsigned short CalcLosStatus(int allyTeam);
	// 更新视野状态
	void UpdateLosStatus(int allyTeam);
 	// 设置是否留下残影
	void SetLeavesGhost(bool newLeavesGhost, bool leaveDeadGhost);

	void UpdateWeapons(); // 更新武器
	void UpdateWeaponVectors(); // 更新武器向量

	void SlowUpdateWeapons();// 慢速更新武器
	void SlowUpdateKamikaze(bool scanForTargets);// 慢速更新神风（自杀攻击）逻辑
	void SlowUpdateCloak(bool stunCheck);// 慢速更新隐形逻辑

	bool ScriptCloak();// 脚本控制的隐形
	bool ScriptDecloak(const CSolidObject* object, const CWeapon* weapon);// 脚本控制的现形
	bool GetNewCloakState(bool checkStun);// 获取新的隐形状态

	enum ChangeType {
		ChangeGiven,   // 被赠予
		ChangeCaptured // 被捕获
	};
	virtual bool ChangeTeam(int team, ChangeType type); // 改变队伍
	virtual void StopAttackingAllyTeam(int ally); // 停止攻击指定的盟友队伍

	// --- 运输相关 ---
	//Transporter stuff
	CR_DECLARE_SUB(TransportedUnit) // 序列化子结构
	struct TransportedUnit {
		CR_DECLARE_STRUCT(TransportedUnit) // 序列化结构体
		CUnit* unit; // 被运输的单位
		int piece; // 附着在运输机的哪个部件上
	};
	// 设置唯一的建造者
	bool SetSoloBuilder(CUnit* builder, const UnitDef* buildeeDef);
	void SetLastAttacker(CUnit* attacker); // 设置最后的攻击者

	void SetTransporter(CUnit* trans) { transporter = trans; } // 设置运输此单位的运输机
	CUnit* GetTransporter() const { return transporter; } // 获取运输此单位的运输机

	bool AttachUnit(CUnit* unit, int piece, bool force = false); // 将一个单位装载到本机上
	bool CanTransport(const CUnit* unit) const; // 判断是否可以运输指定的单位

	bool DetachUnit(CUnit* unit); // 从本机卸载一个单位
	bool DetachUnitCore(CUnit* unit);  // 卸载单位的核心逻辑
	// 从空中卸载单位，并命令其移动到指定位置
	bool DetachUnitFromAir(CUnit* unit, const float3& pos); // orders <unit> to move to <pos> after detach 
	// 判断是否能在指定位置装载/卸载
	bool CanLoadUnloadAtPos(const float3& wantedPos, const CUnit* unit, float* wantedHeightPtr = nullptr) const;
	// 获取被运输单位期望的高度
	float GetTransporteeWantedHeight(const float3& wantedPos, const CUnit* unit, bool* ok = nullptr) const;
	// 获取被运输单位期望的朝向
	short GetTransporteeWantedHeading(const CUnit* unit) const;

public:
	// 死亡脚本执行完毕时调用
	void KilledScriptFinished(int wreckLevel) { deathScriptFinished = true; delayedWreckLevel = wreckLevel; }
	// 强制杀死单位（无死亡动画）
	void ForcedKillUnit(CUnit* attacker, bool selfDestruct, bool reclaimed, int weaponDefID = 0);
	// 杀死单位
	virtual void KillUnit(CUnit* attacker, bool selfDestruct, bool reclaimed, int weaponDefID = 0);
	// 当有导弹飞来时调用
	virtual void IncomingMissile(CMissileProjectile* missile);
	// 创建残骸
	CFeature* CreateWreck(int wreckLevel, int smokeTime);
	// 临时停火
	void TempHoldFire(int cmdID);
	// 设置停火状态
	void SetHoldFire(bool b) { onTempHoldFire = b; }

	// 让此单位从父单位上开始自由下落
	// start this unit in free fall from parent unit
	void Drop(const float3& parentPos, const float3& parentDir, CUnit* parent);
	void PostLoad(); // 从存档加载后调用
protected:
	void ChangeTeamReset();// 改变队伍后的重置逻辑
	void UpdateResources();  // 更新资源
	float GetFlankingDamageBonus(const float3& attackDir); // 获取侧翼攻击伤害加成

public: // unsynced methods
	bool SetGroup(CGroup* newGroup, bool fromFactory = false, bool autoSelect = true); // 设置所属单位组
	// 获取所属单位组
	const CGroup* GetGroup() const;
	      CGroup* GetGroup();
	// 是否被绘制为图标
	bool GetIsIcon() const { return HasDrawFlag(DrawFlags::SO_DRICON_FLAG); }
	void SetIsIcon(bool b) {  // 设置是否被绘制为图标
		if (b)
			AddDrawFlag(DrawFlags::SO_DRICON_FLAG);
		else
			DelDrawFlag(DrawFlags::SO_DRICON_FLAG);
	}
public:
	// 计算经验值缩放
	static float ExperienceScale(float limExperience, float experienceWeight) {
		// limExperience ranges from 0.0 to 0.9999...
		return std::max(0.0f, 1.0f - (limExperience * experienceWeight));
	}

public:
	// --- 成员变量 ---
	const UnitDef* unitDef = nullptr; // 指向单位定义（蓝图）的指针

	// Our shield weapon, NULL if we have none
	CWeapon* shieldWeapon = nullptr; // 单位的护盾武器，如果没有则为 NULL
	// Our weapon with stockpiled ammo, NULL if we have none
	CWeapon* stockpileWeapon = nullptr; // 单位的“储备弹药”类武器（如核弹），如果没有则为 NULL

	const DynDamageArray* selfdExpDamages = nullptr; // 自毁时产生的爆炸伤害
	const DynDamageArray* deathExpDamages = nullptr; // 死亡时产生的爆炸伤害

	CUnit* soloBuilder = nullptr;  // 唯一的建造者（用于某些建造逻辑）
	CUnit* lastAttacker = nullptr; // 最后一个攻击本单位的单位
	CUnit* transporter = nullptr;  // 正在运输本单位的运输机

	// player who is currently FPS'ing this unit
	// 正在第一人称控制此单位的玩家
	CPlayer* fpsControlPlayer = nullptr;

	AMoveType* moveType = nullptr;     // 当前的移动类型实例
	AMoveType* prevMoveType = nullptr; // 之前的移动类型实例（用于脚本移动类型切换）

	CCommandAI* commandAI = nullptr;   // 指令AI实例
	CUnitScript* script = nullptr;     // 单位脚本实例

	// current attackee
	SWeaponTarget curTarget; // 当前的武器目标


	// sufficient for the largest UnitScript (CLuaUnitScript)
	// 为最大的 UnitScript (CLuaUnitScript) 预留的内存缓冲区，避免频繁的堆分配
	uint8_t usMemBuffer[sizeof(CLuaUnitScript)];
	// sufficient for the largest AMoveType (CGroundMoveType)
	// need two buffers since ScriptMoveType might be enabled
	// 为最大的 AMoveType (CGroundMoveType) 预留的内存缓冲区
	// 需要两个，因为脚本移动类型可能会启用
	uint8_t amtMemBuffer[sizeof(CGroundMoveType)];
	uint8_t smtMemBuffer[sizeof(CScriptMoveType)];
	// sufficient for the largest CommandAI type (CBuilderCAI)
	// knowing the exact CAI object size here is not required;
	// static asserts will catch any overflow
	// 为最大的 CommandAI (CBuilderCAI) 预留的内存缓冲区
	// 这里不需要知道精确的CAI对象大小；静态断言会捕捉任何溢出
	uint8_t caiMemBuffer[sizeof(CBuilderCAI)];


	std::vector<CWeapon*> weapons; // 单位拥有的所有武器

	// which squares the unit can currently observe, per los-type
	// 单位当前可观察到的方格，按视野类型划分
	std::array<SLosInstance*, /*ILosType::LOS_TYPE_COUNT*/ 7> los{{nullptr}};

	// indicates the los/radar status each allyteam has on this unit
	// should technically be MAX_ALLYTEAMS, but #allyteams <= #teams
	// 指示每个盟友队伍对此单位的视野/雷达状态
	// 理论上应为 MAX_ALLYTEAMS，但盟友队伍数 <= 队伍数
	std::array<uint8_t, /*MAX_TEAMS*/ 255> losStatus{{0}};
	// bit-mask indicating which allyteams see this unit with positional error
	// 位掩码，指示哪些盟友队伍看到的此单位带有位置误差
	std::array<uint32_t, /*MAX_TEAMS/32*/ 8> posErrorMask{{1}};

	// quads the unit is part of
	// 单位所在的四叉树区域
	std::vector<int> quads;

	// 本单位运输的其他单位列表
	std::vector<TransportedUnit> transportedUnits;
	// 飞向本单位的、可以被曳光弹引开的导弹
	// incoming projectiles for which flares can cause retargeting
	std::array<CMissileProjectile*, /*MAX_INCOMING_MISSILES*/ 8> incomingMissiles{{nullptr}};

	// 上一次开火的炮口火焰方向
	float3 lastMuzzleFlameDir = UpVector;
	// units take less damage when attacked from this dir (encourage flanking fire)
	// 从此方向攻击单位受到的伤害较少（鼓励侧翼攻击）
	float3 flankingBonusDir = RgtVector;

	// used for radar inaccuracy etc
	// 用于雷达不精确等
	float3 posErrorVector;// 位置误差向量
	float3 posErrorDelta;// 位置误差变化量


	// 死亡后生成的残骸的 FeatureDef ID
	int featureDefID = -1; // FeatureDef id of the wreck we spawn on death

	// 指示单位的相对战斗力，用于经验值计算等
	// indicate the relative power of the unit, used for experience calculations etc
	float power = 100.0f;

	// 0.0-1.0
	float buildProgress = 0.0f; // 建造进度, 0.0-1.0
	// if (health - this) is negative the unit is stunned
	// 麻痹伤害量，如果 (health - this) < 0，单位被眩晕
	float paralyzeDamage = 0.0f;
	// how close this unit is to being captured
	// 捕获进度
	float captureProgress = 0.0f;
	// 经验值
	float experience = 0.0f;
	// approaches 1 as experience approaches infinity
	// 极限经验值，当经验趋于无穷时，此值趋于1
	float limExperience = 0.0f;


	// how much terraforming is left to do
	// 剩余的地形改造量
	float terraformLeft = 0.0f;
	// How much reapir power has been added to this recently
	// 最近被施加的修理量
	float repairAmount = 0.0f;

	// last frame unit was attacked by other unit
	// 上一次被攻击的帧
	int lastAttackFrame = -200;
	// last time this unit fired a weapon
	// 上一次开火的帧
	int lastFireWeapon = 0;

	// if we arent built on for a while start decaying
	// 上一次被纳米（建造/维修）的帧，如果长时间未被建造，会开始衰变
	int lastNanoAdd = 0;
	// 上一次投下曳光弹的帧
	int lastFlareDrop = 0;

	// id of transport that the unit is about to be {un}loaded by
	// 正在准备装载本单位的运输机ID
	int loadingTransportId = -1; 
	int unloadingTransportId = -1; // 正在准备卸载本单位的运输机ID
	bool requestRemoveUnloadTransportId = false; // 请求移除卸载运输机ID的标志

	int transportCapacityUsed = 0; // 已使用的运输容量
	float transportMassUsed = 0.0f; // 已使用的运输质量


	// the wreck level the unit will eventually create when it has died
	 // 单位死亡后最终将创建的残骸等级
	int delayedWreckLevel = -1;

	// how long the unit has been inactive
	// 单位处于非活动状态的时间
	unsigned int restTime = 0;
	// 装填速度乘数
	float reloadSpeed = 1.0f;
	// 最大射程
	float maxRange = 0.0f;

	// used to determine muzzle flare size
	// 用于确定炮口火焰大小
	float lastMuzzleFlameSize = 0.0f;

	int armorType = 0; // 装甲类型
	// what categories the unit is part of (bitfield)
	 // 单位所属的类别（位域）
	unsigned int category = 0;
	// 所在的地图方格索引
	int mapSquare = -1;
	// 建造完成后实际的视野半径
	// set los to this when finished building
	int realLosRadius = 0;
	// 建造完成后实际的空中视野半径
	int realAirLosRadius = 0;

	// 视野半径
	int losRadius = 0;
	// 空中视野半径
	int airLosRadius = 0;

	int radarRadius = 0; // 雷达半径
	int sonarRadius = 0; // 声呐半径
	int jammerRadius = 0; // 雷达干扰半径
	int sonarJamRadius = 0; // 声呐干扰半径
	int seismicRadius = 0; // 地震波半径

	float seismicSignature = 0.0f; // 地震信号特征值
	float decloakDistance = 0.0f; // 自动现形的距离


	// --- 资源相关 ---
	SResourcePack resourcesCondUse;  // 条件性资源消耗（激活时）
	SResourcePack resourcesCondMake; // 条件性资源产出（激活时）

	SResourcePack resourcesUncondUse;  // 无条件资源消耗
	SResourcePack resourcesUncondMake; // 无条件资源产出

	SResourcePack resourcesUse;  // 每 UNIT_SLOWUPDATE_RATE 帧的总资源消耗
	SResourcePack resourcesMake; // 每 UNIT_SLOWUPDATE_RATE 帧的总资源产出

	// variables used for calculating unit resource usage
	// 用于计算单位资源使用量的变量
	SResourcePack resourcesUseI;
	SResourcePack resourcesMakeI;
	SResourcePack resourcesUseOld;
	SResourcePack resourcesMakeOld;

	// the amount of storage the unit contributes to the team
	// 此单位为团队提供的资源存储量
	SResourcePack storage;

	// per unit storage (gets filled on reclaim and needs then to be unloaded at some storage building -> 2nd part is lua's job)
	// 单位自身的采集存储（回收时填满，需在存储建筑卸载 -> 后半部分由Lua负责）
	SResourcePack harvestStorage;
	// 已采集的资源
	SResourcePack harvested;
	// 建造此单位的成本
	SResourcePack cost = {100.0f, 0.0f};

	// how much metal the unit currently extracts from the ground
	// 单位当前从地面提取的金属量
	float metalExtract = 0.0f;
	// 建造时间
	float buildTime = 100.0f;

	// decaying value of how much damage the unit has taken recently (for severity of death)
	// 最近受到的伤害量的衰减值（用于决定死亡的严重程度）
	float recentDamage = 0.0f;

	int fireState = 0; // 开火状态
	int moveState = 0; // 移动状态

	// for units being dropped from transports (parachute drops)
	// 从运输机上被空投下来时的下落速度
	float fallSpeed = 0.2f;

	/**
	 * 0 = no flanking bonus
	 * 1 = global coords, mobile
	 * 2 = unit coords, mobile
	 * 3 = unit coords, locked
	 */
	/**
	 * 0 = 无侧翼加成
	 * 1 = 全局坐标，可移动
	 * 2 = 单位坐标，可移动
	 * 3 = 单位坐标，锁定
	 */
	int flankingBonusMode = 0;

	float flankingBonusMobility = 10.0f;   // 侧翼加成的最弱伤害方向在受攻击时可转动的程度（受击时归零，缓慢增加）
	float flankingBonusMobilityAdd = 0.01f; // 侧翼加成方向的移动能力每帧增加量
	float flankingBonusAvgDamage = 1.4f;   // 平均伤害乘数因子
	float flankingBonusDifDamage = 0.5f;   // (最大伤害 - 最小伤害) / 2

	float armoredMultiple = 1.0f; // 装甲状态下的伤害乘数
	float curArmorMultiple = 1.0f; // 当前的伤害乘数

	int nextPosErrorUpdate = 1; // 下一次位置误差更新的时间

	int lastTerrainType = -1; // 上一个地形类型
	int curTerrainType = 0;   // 当前地形类型 (用于调用TA脚本需要的 setSFXoccupy)

	int selfDCountdown = 0; // 自毁倒计时

	int cegDamage = 0; // 由此单位脚本生成的自定义爆炸图形(CEG)的伤害值


	// --- 布尔状态标志 ---
	// if the unit is in it's 'on'-state 
	// 单位是否处于激活状态
	bool activated = false;
	// prevent damage from hitting an already dead unit (causing multi wreck etc)
	bool isDead = false; // 单位是否已死亡，防止伤害鞭尸导致多重残骸等问题

	bool armoredState = false; // 是否处于装甲状态

	bool stealth = false;  // 是否具有潜行能力
	bool sonarStealth = false; // 是否具有声呐潜行能力

	// used by constructing units
	// 是否处于建造姿态（用于建造单位）
	bool inBuildStance = false; 
	// tells weapons that support it to try to use a high trajectory
	 // 是否使用高弹道（告诉支持此功能的武器）
	bool useHighTrajectory = false; 
	// used by landed gunships to block weapon Update()'s, also by builders to
	// prevent weapon SlowUpdate()'s and Attack{Unit,Ground}()'s during certain
	// commands
	// 是否临时停火，用于降落的武装直升机或建造者在执行特定命令时
	bool onTempHoldFire = false;

	// Lua overrides for CanUpdateWeapons
	// Lua覆盖：强制使用武器
	bool forceUseWeapons = false;
	// Lua覆盖：允许使用武器
	bool allowUseWeapons =  true;

	// signals if script has finished executing Killed and the unit can be deleted
	// 死亡脚本是否执行完毕，是则单位可被删除
	bool deathScriptFinished = false;

	// if true, unit will not be automatically fired upon unless attacker's fireState is set to > FIREATWILL
	// 是否是中立单位，是则不会被自动攻击
	bool neutral = false;
	// if unit is currently incompletely constructed (implies buildProgress < 1)
	// 是否正在被建造（意味着 buildProgress < 1）
	bool beingBuilt = true;
	// if the updir is straight up or align to the ground vector
	// updir是笔直向上还是贴合地面法线
	bool upright = true;
	// whether the ground below this unit has been terraformed
	// 此单位下方的地面是否已被平整
	bool groundLevelled = true;

	// true if the unit is currently cloaked (has enough energy etc)
	// 单位当前是否已隐形（有足够能量等）
	bool isCloaked = false;
	// true if the unit currently wants to be cloaked
	// 单位当前是否想要隐形
	bool wantCloak = false;
	// true if the unit leaves static ghosts
	// 单位是否留下静态残影
	bool leavesGhost = false;

	// unsynced vars
	// --- 非同步变量 ---
	bool noMinimap = false; // 是否在小地图上不可见
	bool leaveTracks = false; // 是否留下履带痕迹

	bool isSelected = false; // 是否被选中
	// if true, unit can not be added to groups by a player (UNSYNCED)
	// 如果为 true，此单位不能被玩家编组 (非同步)
	bool noGroup = false;
	// 图标半径
	float iconRadius = 0.0f;
	// 图标数据指针
	icon::CIconData* myIcon = nullptr;
	// 是否绘制图标
	bool drawIcon = true;
private:
	// if we are stunned by a weapon or for other reason, access via IsStunned/SetStunned(bool)
	// 因武器或其他原因被眩晕，通过 IsStunned/SetStunned(bool) 访问
	bool stunned = false;
};

// 全局单位参数结构体
struct GlobalUnitParams {
	CR_DECLARE_STRUCT(GlobalUnitParams) // 序列化

	float empDeclineRate = 0.0f; // EMP（电磁脉冲）效果的衰减速率
	float expMultiplier  = 0.0f; // 经验值乘数
	float expPowerScale  = 0.0f; // 经验对攻击力的影响尺度
	float expHealthScale = 0.0f; // 经验对生命值的影响尺度
	float expReloadScale = 0.0f; // 经验对装填速度的影响尺度
	float expGrade       = 0.0f; // 经验等级参数
};

// 全局单位参数实例
extern GlobalUnitParams globalUnitParams;

#endif // UNIT_H
