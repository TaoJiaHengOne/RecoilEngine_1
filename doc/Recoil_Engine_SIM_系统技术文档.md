# Recoil Engine SIM 系统完整技术文档

## 目录

1. [系统概览](#1-系统概览)
2. [核心对象层次结构](#2-核心对象层次结构)
3. [单位管理系统 (Units)](#3-单位管理系统-units)
4. [物理系统和碰撞检测 (Physics & Collision)](#4-物理系统和碰撞检测-physics--collision)
5. [路径寻找系统 (PathFinding)](#5-路径寻找系统-pathfinding)
6. [武器和伤害系统 (Weapons & Damage)](#6-武器和伤害系统-weapons--damage)
7. [地形特征系统 (Features)](#7-地形特征系统-features)
8. [移动系统 (MoveTypes)](#8-移动系统-movetypes)
9. [命令AI系统 (CommandAI)](#9-命令ai系统-commandai)
10. [投射物系统 (Projectiles)](#10-投射物系统-projectiles)
11. [ECS架构集成](#11-ecs架构集成)
12. [性能优化技术](#12-性能优化技术)
13. [数据结构和类关系总结](#13-数据结构和类关系总结)

---

## 1. 系统概览

Recoil Engine 的 SIM (Simulation) 文件夹包含了游戏模拟的核心系统，实现了一个复杂的实时策略游戏引擎。该架构采用了分层设计，结合了**实体组件系统(ECS)**、**传统面向对象**和**数据驱动**的设计模式。

### 1.1 核心设计理念

- **模块化架构**: 每个系统职责明确，接口清晰
- **性能优先**: 支持数千单位的60FPS实时模拟
- **多线程并行**: 关键系统支持多线程优化
- **内存效率**: 使用对象池和内存缓冲技术
- **可扩展性**: 支持Lua脚本和数据驱动配置

### 1.2 文件夹结构

```
rts/Sim/
├── Objects/          # 基础对象层次结构
├── Units/            # 单位管理系统
├── Weapons/          # 武器系统
├── Projectiles/      # 投射物系统
├── MoveTypes/        # 移动类型系统
├── Path/             # 路径寻找系统
├── Features/         # 地形特征系统
├── Misc/             # 工具和辅助系统
└── Ecs/              # 实体组件系统
```

---

## 2. 核心对象层次结构

### 2.1 基础对象体系

整个SIM系统的对象继承体系如下：

```cpp
CObject (System/Object.h)
└── CWorldObject (Objects/WorldObject.h)
    ├── pos, speed, radius, height          // 基础物理属性
    ├── drawRadius, drawFlag                // 渲染相关
    └── tempNum, mtTempNum                  // 多线程临时标记
    
    └── CSolidObject (Objects/SolidObject.h)
        ├── CUnit (Units/Unit.h)            // 单位对象
        ├── CFeature (Features/Feature.h)   // 地形特征
        └── CProjectile (Projectiles/Projectile.h) // 投射物
```

### 2.2 CSolidObject 核心特性

**物理状态管理** (位域系统):
```cpp
enum PhysicalState {
    PSTATE_BIT_ONGROUND    = (1 << 0),  // 在地面
    PSTATE_BIT_INWATER     = (1 << 1),  // 在水中
    PSTATE_BIT_UNDERWATER  = (1 << 2),  // 完全水下
    PSTATE_BIT_UNDERGROUND = (1 << 3),  // 地下
    PSTATE_BIT_INAIR       = (1 << 4),  // 空中
    PSTATE_BIT_MOVING      = (1 << 6),  // 移动中
    PSTATE_BIT_FLYING      = (1 << 7),  // 飞行中
    PSTATE_BIT_BLOCKING    = (1 << 11), // 阻塞地图
};
```

**变换系统**:
- `frontdir, rightdir, updir`: 本地坐标系
- `heading, buildFacing`: 朝向和建造方向
- `relMidPos, relAimPos`: 相对位置

**碰撞和阻塞**:
- `collisionVolume`: 主碰撞体
- `selectionVolume`: 选择体积  
- `YardMapStatus`: 庭院地图状态

---

## 3. 单位管理系统 (Units)

### 3.1 CUnit 核心架构

```cpp
CUnit : CSolidObject {
    // 核心组件
    const UnitDef* unitDef;          // 静态定义
    AMoveType* moveType;             // 移动系统
    CCommandAI* commandAI;           // 指令AI
    CUnitScript* script;             // 脚本系统
    
    // 战斗系统
    std::vector<CWeapon*> weapons;   // 武器列表
    CWeapon* shieldWeapon;           // 护盾武器
    CWeapon* stockpileWeapon;        // 储备武器
    SWeaponTarget curTarget;         // 当前目标
    
    // 状态管理
    float health, maxHealth;         // 生命值
    float buildProgress;             // 建造进度
    float experience, limExperience; // 经验系统
    float paralyzeDamage;            // 麻痹伤害
    
    // 运输系统
    CUnit* transporter;              // 运输机
    std::list<CUnit*> transportedUnits; // 被运输单位
    short transportCapacity;         // 运输容量
    
    // 资源管理
    std::array<float, MAX_RESOURCES> resourcesUse; // 资源消耗
    std::array<float, MAX_RESOURCES> resourcesMake; // 资源产出
    std::array<float, MAX_RESOURCES> storage;      // 存储容量
    SResourcePack cost;              // 建造成本
    
    // 内存管理优化
    CLuaUnitScript::UnitMemBuffer usMemBuffer;    // 单位脚本缓冲
    CGroundMoveType::GMTMemBuffer amtMemBuffer;   // 移动类型缓冲
    CBuilderCAI::BuilderCAIMemBuffer caiMemBuffer; // 指令AI缓冲
};
```

### 3.2 单位子系统

#### CommandAI 层次结构
```cpp
CCommandAI (基类)
├── CMobileCAI    // 移动单位AI
├── CBuilderCAI   // 建造单位AI  
├── CFactoryCAI   // 工厂AI
└── CAirCAI       // 空中单位AI
```

**核心功能**:
- **命令队列管理** (`CCommandQueue commandQue`)
- **目标跟踪** (`CUnit* orderTarget`)
- **命令描述缓存** (`possibleCommands`)
- **死亡依赖管理** (`commandDeathDependences`)

#### Scripts 系统
支持两种脚本类型:

1. **COB Scripts** (传统二进制脚本):
   - `CobEngine`, `CobInstance`, `CobThread`
   - 操作码执行引擎
   - 支持传统TA/Spring脚本

2. **Lua Scripts** (现代脚本系统):
   - `LuaUnitScript`
   - 与引擎更紧密集成
   - 更强的表达能力

### 3.3 UnitHandler 全局管理

```cpp
CUnitHandler 核心职责:
├── ID管理
│   └── SimObjectIDPool idPool       // ID池
├── 集合管理
│   ├── vector<CUnit*> units        // ID索引
│   ├── unitsByDefs[team][unitDef]  // 按队伍和类型分组
│   └── activeUnits                 // 活跃单位
├── 更新流水线
│   ├── UpdatePreFrame()            // 预更新
│   ├── Update()                    // 主更新
│   └── UpdatePostFrame()           // 后更新
└── 多线程支持
    ├── UpdateUnitPathing()         // 路径更新
    └── UpdateUnitWeapons()         // 武器更新
```

**性能优化**:
- **批量更新**: 单位按类型分组批量处理
- **LOD系统**: 基于距离的细节层次
- **多线程**: 路径规划和武器更新并行化

---

## 4. 物理系统和碰撞检测 (Physics & Collision)

### 4.1 分层碰撞检测架构

引擎采用三层碰撞检测体系:

**第一层：空间索引 (Broad Phase)**
```cpp
// QuadField.h - 四叉树空间分割
class CQuadField {
    constexpr static unsigned int BASE_QUAD_SIZE = 128; // 基础四叉树大小128x128
    std::vector<Quad> baseQuads;                        // 四叉树节点数组
    
    void GetUnitsExact(QuadFieldQuery& qfq, const float3& pos, float radius);
    void GetFeaturesExact(QuadFieldQuery& qfq, const float3& pos, float radius);
};
```

**第二层：AABB预检测 (Narrow Phase Pre-filter)**
```cpp
// CollisionHandler.cpp - 边界盒早期剔除
bool CCollisionHandler::Collision(const CollisionVolume* v, const CMatrix44f& m, const float3& p) {
    // 早期边界球检测
    if ((v->GetWorldSpacePos(o) - p).SqLength() > v->GetBoundingRadiusSq())
        return false;
    // 进入精确检测
}
```

**第三层：精确几何检测 (Narrow Phase)**
支持四种基本碰撞体类型:
- **球体 (Sphere)**: `pi.dot(pi) <= radius²`
- **椭球体 (Ellipsoid)**: `(x²/a²) + (y²/b²) + (z²/c²) <= 1`
- **圆柱体 (Cylinder)**: 轴向范围检查 + 截面椭圆检测
- **包围盒 (Box)**: 三轴范围检查

### 4.2 空间索引优化

#### QuadField 设计特点

- **固定网格**: 地图按128x128单位划分四叉树节点
- **对象类型分离**: 每个节点分别存储Units、Features、Projectiles
- **团队优化**: 按盟友团队分组存储，加速友军/敌军查询
- **多线程支持**: 每线程独立的临时向量池

**性能优化**:
```cpp
// 预分配向量池避免动态分配
std::array<QueryVectorCache<CUnit*>, ThreadPool::MAX_THREADS> tempUnits;
std::array<QueryVectorCache<CFeature*>, ThreadPool::MAX_THREADS> tempFeatures;

// 圆形区域查询优化
void GetQuads(QuadFieldQuery& qfq, float3 pos, float radius) {
    const int2 min = WorldPosToQuadField(pos - radius);
    const int2 max = WorldPosToQuadField(pos + radius);
    const float maxSqLength = (radius + quadSizeX * 0.72f)²; // 优化的圆形检测
}
```

#### GroundBlockingObjectMap 混合存储

```cpp
template<typename T, uint32_t S = 8> 
struct ArrayCell {
    std::array<T*, S> arr;        // 固定8个对象的数组
    uint32_t numObjs = 0;         // 当前对象数量
    uint32_t vecIndx = 0;         // 溢出向量索引
};
```

**设计优势**:
- **常见情况优化**: 80%网格包含≤8个对象，用数组避免堆分配
- **溢出处理**: 超过8个对象时溢出到动态向量
- **内存局部性**: 热数据在数组中，减少缓存未命中

### 4.3 多线程碰撞检测

**线程安全设计**:
```cpp
// 每线程独立的临时向量缓存
std::array<QueryVectorCache<CUnit*>, ThreadPool::MAX_THREADS> tempUnits;

// 线程安全的对象访问
inline int GetMtTempNum() const { return mtTempNum[ThreadPool::GetThreadNum()]; }
inline void SetMtTempNum(int value) { mtTempNum[ThreadPool::GetThreadNum()] = value; }
```

**无锁算法**:
- 使用原子操作更新物理状态位
- 分离读写操作避免竞争条件
- 预分配内存池减少动态分配

---

## 5. 路径寻找系统 (PathFinding)

### 5.1 双寻路器架构

引擎支持两种寻路系统:

#### HAPFS (分层A*寻路系统)

```cpp
HAPFS::CPathManager
├── 多分辨率寻路             // Multi-Resolution Pathfinding
│   ├── CPathFinder (最高精度)      // 详细路径
│   ├── CPathEstimator (中精度)     // 估算路径
│   └── CPathEstimator (低精度)     // 粗略路径  
├── 路径协调               // Path Coordination
│   ├── MultiPath                   // 多层路径结构
│   ├── ArrangePath()               // 路径整合
│   └── LowRes2MaxRes()            // 精度提升
├── 热力图和流场图               // Heat & Flow Maps
│   ├── PathHeatMap                // 路径热力图
│   └── PathFlowMap                // 路径流场图
└── 缓存系统                 // Caching System
    └── PathCache                  // 路径缓存
```

#### QTPFS (四叉树寻路系统)

```cpp  
QTPFS::PathManager
├── 四叉树结构            // Quad-Tree Structure
│   └── NodeLayer                  // 节点层
├── 实体组件系统       // Entity-Component System
│   ├── Path Components           // 路径组件
│   └── Speed Mod Components      // 速度修改组件
├── 多线程                     // Threading
│   ├── SearchThreadData          // 搜索线程数据
│   └── UpdateThreadData          // 更新线程数据
└── 路径共享                  // Path Sharing
    ├── SharedPathMap             // 共享路径图
    └── PartialSharedPathMap      // 部分共享路径
```

### 5.2 算法实现细节

#### A*搜索核心
```cpp
// 方向成本常量 - 优化对角线移动
static constexpr float PF_DIRECTION_COSTS[] = {
    1.0f,        // 直线移动
    math::SQRT2, // 对角线移动 (√2 ≈ 1.414)
};

// 启发式函数 - 曼哈顿距离 + 对角线优化
float GetHeuristic(const MoveDef& moveDef, const CPathFinderDef& pfDef, const int2& square) const;
```

#### 自适应四叉树节点
```cpp
struct QTNode {
    // 节点边界（16位整数优化内存）
    unsigned short _xmin, _xmax, _zmin, _zmax;
    
    // 平均移动成本
    float moveCostAvg = -1.0f;
    
    // 子节点索引
    unsigned int childBaseIndex = -1u;
    
    // 邻居节点连接点
    std::vector<NeighbourPoints> neighbours;
};
```

### 5.3 性能对比

| 特性 | HAPFS | QTPFS |
|------|-------|-------|
| 算法基础 | 分层A* | 自适应四叉树 |
| 内存使用 | 中等 | 较低（动态分配） |
| 计算精度 | 高（1x1格精度） | 可变（自适应细分） |
| 地形适应性 | 预计算+缓存 | 实时自适应 |
| 多线程支持 | 部分支持 | 全面支持 |
| 路径质量 | 精确但可能重叠 | 智能避让，分散性好 |

---

## 6. 武器和伤害系统 (Weapons & Damage)

### 6.1 武器系统架构

```cpp
CWeapon 基础架构:
├── 目标系统                     // Target System
│   ├── currentTarget                // 当前目标
│   ├── targetWeight()               // 目标权重计算
│   └── AutoTarget()                 // 自动目标选择
├── 开火控制                   // Firing Control
│   ├── reloadTime, reloadStatus     // 装填系统
│   ├── salvoSize, salvoDelay        // 齐射控制
│   └── angleGood, weaponDir         // 瞄准系统
├── 弹道系统                       // Ballistics
│   ├── range, projectileSpeed       // 射程和速度
│   ├── accuracyError, sprayAngle    // 精度控制
│   └── heightBoostFactor            // 高度加成
└── 脚本集成               // Script Integration
    ├── AimScriptFinished()          // 瞄准完成
    └── hasBlockShot                 // 阻挡射击检查
```

### 6.2 武器类型层次

```cpp
Weapon Type Hierarchy:
CWeapon (base)
├── BeamLaser          // 光束激光
├── Cannon             // 加农炮  
├── MissileLauncher    // 导弹发射器
├── FlameThrower       // 火焰喷射器
├── LightningCannon    // 闪电炮
├── PlasmaRepulser     // 等离子体反射器
└── NoWeapon           // 空武器
```

### 6.3 弹道计算算法

**火炮弹道计算**:
```cpp
float3 CCannon::CalcWantedDir(const float3& targetVec) const {
    const float Dsq = targetVec.SqLength();     // 目标距离平方
    const float DFsq = targetVec.SqLength2D();  // 水平距离平方
    const float g = gravity;                    // 重力加速度
    const float v = projectileSpeed;            // 初始速度
    const float dy = targetVec.y;               // 高度差
    
    // 弹道方程求解：考虑高/低弹道选择
    const float vsq = v * v;
    const float root1 = vsq * vsq + 2.0f * vsq * g * dy - g * g * DFsq;
    if (root1 >= 0.0f) {
        const float root2 = 2.0f * DFsq * Dsq * 
            (vsq + g * dy + (highTrajectory ? -1.0f : 1.0f) * math::sqrt(root1));
        if (root2 >= 0.0f) {
            Vxz = math::sqrt(root2) / (2.0f * Dsq);  // 水平速度分量
            Vy = (Vxz * dy / dxz - dxz * g / (2.0f * Vxz)); // 垂直速度分量
        }
    }
}
```

### 6.4 伤害系统

**伤害数组设计**:
```cpp
class DamageArray {
    std::vector<float> damages;        // 各装甲类型伤害值
    int paralyzeDamageTime;           // 瘫痪持续时间
    float impulseFactor, impulseBoost; // 冲击力参数
    float craterMult, craterBoost;     // 弹坑参数
};
```

**动态伤害计算**:
```cpp
DamageArray GetDynamicDamages(const float3& startPos, const float3& curPos) const {
    const float travDist = min(dynDamageRange, curPos.distance2D(startPos));
    const float damageMod = 1.0f - pow(travDist / dynDamageRange, dynDamageExp);
    
    // 根据飞行距离衰减伤害
    for (int i = 0; i < damageTypes; ++i) {
        if (dynDamageInverted) {
            damage[i] = baseDamage[i] - damageMod * baseDamage[i];
        } else {
            damage[i] = damageMod * baseDamage[i];
        }
        // 应用最小伤害限制
        damage[i] = max(baseDamage[i] * dynDamageMin, damage[i]);
    }
}
```

---

## 7. 地形特征系统 (Features)

### 7.1 CFeature 架构

```cpp
CFeature : CSolidObject {
    // 资源系统
    SResourcePack defResources;      // 默认资源
    SResourcePack resources;         // 当前资源
    int reclaimTime;                 // 回收时间
    
    // 物理控制
    MoveCtrl* moveCtrl;              // 移动控制
    unsigned int velocityMask;       // 速度掩码
    unsigned int movementMask;       // 移动掩码
    
    // 生命周期
    float resurrectProgress;         // 复活进度
    float reclaimLeft;              // 剩余回收量
    bool deleteMe;                  // 删除标志
    
    // 视觉效果
    float fireTime, smokeTime;       // 火焰和烟雾时间
    float alphaFade;                // 透明度渐变
};
```

### 7.2 特征生命周期

**创建流程**:
```cpp
CFeature* LoadFeature(const FeatureLoadParams& params) {
    // 1. 从对象池分配内存
    // 2. 初始化物理属性和变换矩阵  
    // 3. 注册到四叉树 (QuadField)
    // 4. 加入更新队列
}
```

**资源回收机制**:
- **渐进式回收**: `reclaimLeft` 从 1.0 递减到 0
- **防作弊保护**: `isRepairingBeforeResurrect` 标志位
- **自动回收**: 支持 `autoreclaim` 特征自动消失

### 7.3 FeatureHandler 管理

全局特征管理器负责:
- **特征生命周期管理**: 创建、更新、销毁
- **资源回收处理**: 回收进度和资源计算
- **四叉树空间索引**: 快速空间查询
- **与单位系统交互**: 碰撞检测和建造清理

---

## 8. 移动系统 (MoveTypes)

### 8.1 移动类型架构

```cpp
AMoveType 层次结构:
AMoveType (抽象基类)
├── 核心接口                   // Core Interface
│   ├── StartMoving()               // 开始移动
│   ├── StopMoving()                // 停止移动
│   ├── Update()                    // 更新
│   └── KeepPointingTo()            // 保持指向
├── 状态管理                 // State Management
│   ├── progressState               // 进度状态
│   ├── maxSpeed, maxWantedSpeed    // 速度控制
│   └── goalPos                     // 目标位置
└── 物理集成             // Physics Integration
    ├── CanApplyImpulse()          // 冲量应用
    └── BrakingDistance()          // 刹车距离
```

### 8.2 具体移动类型

```cpp
Movement Type Hierarchy:
AMoveType
├── GroundMoveType      // 地面移动
│   ├── 路径跟随
│   ├── 转向控制
│   ├── 碰撞避免
│   └── 地形适应
├── HoverAirMoveType    // 悬停飞行
├── StrafeAirMoveType   // 战斗机飞行
├── StaticMoveType      // 静态(建筑)
└── ScriptMoveType      // 脚本控制移动
```

### 8.3 地面移动核心机制

**多线程移动更新流水线**:
```cpp
// 主线程调用顺序
Update() {
    UpdateTraversalPlan();      // 多线程安全：路径规划和障碍躲避
    UpdateUnitPosition();       // 多线程安全：位置和物理状态更新  
    UpdatePreCollisions();      // 单线程：碰撞前预处理
    UpdateCollisionDetections(); // 多线程安全：碰撞检测
    // 碰撞响应在后续的单线程阶段处理
}
```

**路径跟随算法**:
- **双路径点系统**: `currWayPoint` 和 `nextWayPoint`
- **支持倒车机制**: 复杂转角时自动判断是否倒车更优
- **早期同步**: `earlyCurrWayPoint` 用于多线程计算

**物理模拟特性**:
- **打滑系统**: 基于速度与方向夹角的物理打滑判定
- **碰撞响应**: 分离静态碰撞和动态碰撞处理
- **转向惯性**: `turnAccel` 和 `turnSpeed` 实现逼真转向

### 8.4 MoveMath 统一接口

**碰撞查询系统**:
```cpp
struct CheckCollisionQuery {
    const CSolidObject* unit;     // 实际单位（可选）
    const MoveDef* moveDef;       // 移动定义（必需）
    float3 pos;                   // 查询位置
    PhysicalState physicalState;  // 物理状态位
    bool inExitOnlyZone;         // 出口专用区域标志
};
```

**地形评估算法**:
- **速度修正系数**: 基于坡度、高度、移动类型
- **阻挡类型分级**: `BLOCK_NONE` 到 `BLOCK_IMPASSABLE`
- **多线程安全**: 范围检测函数支持并行调用

---

## 9. 命令AI系统 (CommandAI)

### 9.1 命令层次结构

**AI类继承体系**:
```cpp
CCommandAI (基础命令AI)
├── CMobileCAI (移动单位AI)
├── CBuilderCAI (建造单位AI) 
├── CFactoryCAI (工厂AI)
└── CAirCAI (空军AI)
```

### 9.2 命令数据结构

**Command 类设计**:
```cpp
struct Command {
    int id[2];                    // [0]:命令ID, [1]:AI回调ID
    int timeOut;                  // 命令超时机制
    unsigned int tag;             // 命令队列内唯一标识
    unsigned char options;        // 选项位标志
    float params[MAX_COMMAND_PARAMS]; // 内联参数存储
    
    // 支持大参数命令的池化存储
    bool IsPooledCommand() const { return (pageIndex != -1u); }
};
```

**命令类型系统**:
- **位置命令**: `CMD_MOVE`, `CMD_PATROL`, `CMD_FIGHT`
- **目标命令**: `CMD_ATTACK`, `CMD_GUARD`, `CMD_REPAIR`
- **区域命令**: `CMD_AREA_ATTACK`, `CMD_RECLAIM`
- **状态命令**: `CMD_FIRE_STATE`, `CMD_MOVE_STATE`

### 9.3 智能决策机制

**目标选择算法**:
```cpp
// 多因素评估系统
bool IsValidTarget(const CUnit* enemy, CWeapon* weapon) const {
    // 1. 射程检查
    // 2. 火力状态检查  
    // 3. 友军识别
    // 4. 特殊目标过滤
}
```

**路径优化**:
- **BuggerOff**: 智能规避机制，避免单位聚集
- **ReturnFight**: 战斗后返回原路径的智能算法
- **SlowGuard**: 护卫时的缓慢跟随模式

### 9.4 建造系统集成

**CBuilderCAI 高级特性**:
```cpp
// 智能建造位置检测
bool IsBuildPosBlocked(const BuildInfo& bi) {
    // 1. 地形适宜性检查
    // 2. 建造间隔验证
    // 3. 资源点评估
    // 4. 敌方干扰评估
}

// 区域命令处理
bool FindReclaimTargetAndReclaim(const float3& pos, float radius, 
                                unsigned char cmdopt, ReclaimOption options) {
    // 支持敌方单位、特征、残骸的智能筛选回收
}
```

---

## 10. 投射物系统 (Projectiles)

### 10.1 投射物继承层次

```cpp
CProjectile : CExpGenSpawnable {
    // 状态管理                  
    bool synced, weapon, piece;      // 同步和类型标志
    bool deleteMe, createMe;         // 生命周期
    bool checkCol, ignoreWater;      // 碰撞设置
    
    // 物理属性  
    float3 dir, speed;               // 方向和速度
    float myrange, mygravity;        // 射程和重力
    
    // 碰撞系统
    virtual void Collision() = 0;              // 基础碰撞
    virtual void Collision(CUnit*) = 0;        // 单位碰撞
    virtual void Collision(CFeature*) = 0;     // 特征碰撞
    
    // 渲染系统
    float3 drawPos;              // 绘制位置
    float sortDist;              // 排序距离
    bool castShadow, drawSorted; // 阴影和排序
};
```

### 10.2 投射物类型

#### WeaponProjectiles 子类

- **BeamLaserProjectile** - 光束激光
- **MissileProjectile** - 导弹
- **ExplosiveProjectile** - 爆炸性投射物
- **TorpedoProjectile** - 鱼雷
- **StarburstProjectile** - 星爆导弹

#### 导弹制导算法

```cpp
void CMissileProjectile::Update() {
    if (--ttl > 0) {
        // 1. 速度加速
        speed.w += (weaponDef->weaponacceleration * (speed.w < maxSpeed));
        
        // 2. 目标跟踪更新
        const float3& targetVel = UpdateTargeting();
        
        // 3. 飞行轨迹干扰
        UpdateWobble();  // 摆动
        UpdateDance();   // 舞蹈模式
        
        // 4. 高度调整 (弹道导弹模式)
        if (extraHeightTime > 0) {
            extraHeight -= extraHeightDecay;
            targetPos.y += extraHeight;
        }
        
        // 5. 转向计算
        float3 targetDiff = targetPos - pos;
        if (targetDiff.SqLength() > 4.0f) {
            const float turnSpeed = weaponDef->turnrate * TAANG2RAD / 30.0f;
            dir = targetDiff.ANormalize() * turnSpeed + dir * (1.0f - turnSpeed);
        }
    }
}
```

### 10.3 爆炸生成系统

**ExplosionGenerator 特性**:
- **自定义爆炸效果** (CEG系统)
- **粒子系统集成**
- **音效和震屏效果**
- **地形破坏计算**

**ExpGenSpawnable 基类**:
- 支持生成爆炸效果的对象基类
- 统一的爆炸参数接口
- 与脚本系统集成

---

## 11. ECS架构集成

### 11.1 ECS系统概览

Recoil Engine 正在逐步向现代ECS架构迁移:

```cpp
// 使用entt库实现组件系统
namespace Sim {
    extern entt::basic_registry<entity_t> registry;
}

// ECS Helper功能
namespace Sim::ECS {
    void RegisterComponents();          // 注册所有组件
    void SaveComponents(/*...*/);       // 保存组件状态
    void LoadComponents(/*...*/);       // 加载组件状态
}
```

### 11.2 组件系统设计

**移动系统组件**:
```cpp
// 组件定义宏
ALIAS_COMPONENT(GeneralMoveType, int);  // 单线程移动
ALIAS_COMPONENT(GroundMoveType, int);   // 多线程地面移动

// 事件组件
ALIAS_COMPONENT_LIST(FeatureCollisionEvents, std::vector<FeatureCollisionEvent>);
ALIAS_COMPONENT_LIST(UnitCollisionEvents, std::vector<UnitCollisionEvent>);
```

**路径寻找组件**:
```cpp
// QTPFS系统ECS组件
VOID_COMPONENT(PathIsTemp);         // 临时路径
VOID_COMPONENT(PathIsDirty);        // 路径需要更新
VOID_COMPONENT(PathIsToBeUpdated);  // 待更新路径
```

### 11.3 系统更新管道

**ECS系统注册**:
```cpp
// Systems/目录下的ECS系统
- GeneralMoveSystem      // 通用移动系统
- GroundMoveSystem       // 地面移动系统  
- UnitTrapCheckSystem    // 单位陷阱检测
- PathSpeedModInfoSystem // 路径速度修改
- RemoveDeadPathsSystem  // 删除无效路径
```

### 11.4 传统OOP与ECS混合

**过渡策略**:
- **保持向后兼容**: 现有OOP接口不变
- **渐进式迁移**: 新功能优先使用ECS
- **性能关键路径**: 先迁移性能敏感系统
- **数据驱动**: 组件系统支持更好的数据驱动设计

---

## 12. 性能优化技术

### 12.1 内存管理优化

#### 对象池系统

```cpp
// 单位系统内存池
class CUnit {
    // 预分配内存缓冲区避免频繁分配
    CLuaUnitScript::UnitMemBuffer usMemBuffer;    
    CGroundMoveType::GMTMemBuffer amtMemBuffer;   
    CBuilderCAI::BuilderCAIMemBuffer caiMemBuffer; 
};
```

#### ID池管理

```cpp
// SimObjectIDPool - 高效ID分配回收
class SimObjectIDPool {
    std::vector<int> freeIDs;        // 回收ID队列
    std::bitset<MAX_UNITS> usedIDs;  // 已使用ID位图
    
    int ExtractID();                 // 分配ID
    void RecycleID(int id);          // 回收ID
};
```

#### 内存池设计模式

**Features系统**:
```cpp
// FeatureMemPool - 特征对象内存池
static FixedSizeMemPool<sizeof(CFeature), MAX_FEATURES> featureMemPool;
```

**投射物系统**:
```cpp  
// ProjectileMemPool - 投射物内存池
static FixedSizeMemPool<sizeof(CProjectile), MAX_PROJECTILES> projectileMemPool;
```

### 12.2 空间索引优化

#### QuadField四叉树

**设计要点**:
- **固定网格**: 128x128单位的固定分割
- **类型分离**: Units/Features/Projectiles分别存储
- **团队分组**: 按盟友关系分组，加速查询
- **预分配**: 避免查询过程中的内存分配

**查询优化**:
```cpp
// 圆形区域查询优化算法
void GetUnitsExact(QuadFieldQuery& qfq, const float3& pos, float radius) {
    // 1. 计算影响的四叉树节点范围
    const int2 min = WorldPosToQuadField(pos - radius);
    const int2 max = WorldPosToQuadField(pos + radius);
    
    // 2. 遍历节点，精确距离检测
    for (int z = min.y; z <= max.y; ++z) {
        for (int x = min.x; x <= max.x; ++x) {
            const Quad& quad = baseQuads[z * numQuadsX + x];
            
            // 3. 按团队分组处理
            for (const CUnit* unit : quad.teamUnits[allyTeam]) {
                if (pos.SqDistance2D(unit->pos) <= radiusSq) {
                    qfq.units->emplace_back(const_cast<CUnit*>(unit));
                }
            }
        }
    }
}
```

### 12.3 多线程优化

#### 线程安全设计

**无锁数据结构**:
```cpp
// 每线程独立的临时缓存
template<typename T>
struct QueryVectorCache {
    std::array<std::vector<T>, ThreadPool::MAX_THREADS> cache;
    
    std::vector<T>& GetVector(int threadNum) {
        return cache[threadNum];
    }
};
```

**原子操作优化**:
```cpp
// 物理状态位的原子更新
std::atomic<unsigned int> physicalState;

bool UpdatePhysicalStateBit(unsigned int bit, bool set) {
    unsigned int expected, desired;
    do {
        expected = physicalState.load();
        desired = set ? (expected | bit) : (expected & ~bit);
    } while (!physicalState.compare_exchange_weak(expected, desired));
    
    return (desired & bit) != 0;
}
```

#### 批处理系统

**移动系统批处理**:
```cpp
// 地面移动批量更新
void CGroundMoveType::UpdateBatch(std::vector<CGroundMoveType*>& moveTypes) {
    // 并行处理移动类型数组
    #pragma omp parallel for
    for (size_t i = 0; i < moveTypes.size(); ++i) {
        moveTypes[i]->Update();
    }
}
```

**武器系统并行更新**:
```cpp
// 武器批量目标搜索
void CWeapon::BatchTargetSearch(std::vector<CWeapon*>& weapons) {
    #pragma omp parallel for
    for (size_t i = 0; i < weapons.size(); ++i) {
        weapons[i]->AutoTarget();
    }
}
```

### 12.4 缓存优化

#### 数据结构优化 (SOA vs AOS)

**Structure of Arrays设计**:
```cpp
// 武器系统热数据分离
struct WeaponArrays {
    std::vector<float3> positions;       // 连续的位置数据
    std::vector<float3> directions;      // 连续的方向数据
    std::vector<float> reloadTimes;      // 连续的装填时间
    std::vector<SWeaponTarget> targets;  // 连续的目标数据
};
```

#### 计算结果缓存

**弹道计算缓存**:
```cpp
// 火炮弹道计算缓存
float3 CCannon::GetWantedDir(const float3& targetVec) {
    const float3 tgtDif = targetVec - lastTargetVec;
    
    // 目标变化小于阈值时使用缓存结果
    if (fabs(tgtDif.x) < (SQUARE_SIZE/4) && 
        fabs(tgtDif.z) < (SQUARE_SIZE/4)) {
        return lastLaunchDir;
    }
    
    // 重新计算并缓存
    lastLaunchDir = CalcWantedDir(targetVec);
    lastTargetVec = targetVec;
    return lastLaunchDir;
}
```

**路径缓存系统**:
```cpp
class CPathCache {
    struct CacheItem {
        IPath::SearchResult result;
        IPath::Path path;
        int2 strtBlock, goalBlock;
        float goalRadius;
        int pathType;
    };
    
    spring::unordered_map<std::uint64_t, CacheItem> cachedPaths;
    std::deque<CacheQueItem> cacheQue; // LRU管理
    
    // 缓存键生成
    std::uint64_t GetHash(/*...*/) const;
};
```

### 12.5 更新频率优化

#### 分层更新系统

**SlowUpdate机制**:
```cpp
// 降低非关键系统更新频率
void CUnit::SlowUpdate() {
    // 每UNIT_SLOWUPDATE_RATE帧更新一次
    if ((gs->frameNum % UNIT_SLOWUPDATE_RATE) != (id % UNIT_SLOWUPDATE_RATE))
        return;
        
    // 执行低频更新逻辑
    UpdateExperience();
    UpdateResources();
    CheckForPlayerControlChanges();
}
```

**基于距离的LOD**:
```cpp
// 基于摄像机距离的细节层次
float GetLODDistance(const float3& pos) {
    const float camDist = camera->GetPos().distance(pos);
    
    if (camDist < 200.0f) return 1.0f;        // 高细节
    if (camDist < 800.0f) return 0.5f;        // 中细节  
    return 0.25f;                             // 低细节
}
```

#### 智能休眠系统

**单位休眠机制**:
```cpp
class CUnit {
    bool isIdle;                    // 空闲状态
    int idleFrames;                 // 空闲帧数
    
    void CheckIdleState() {
        // 无命令且不移动的单位进入空闲
        if (commandAI->commandQue.empty() && !isMoving) {
            if (++idleFrames > IDLE_THRESHOLD) {
                isIdle = true;
                // 降低更新频率
            }
        } else {
            isIdle = false;
            idleFrames = 0;
        }
    }
};
```

---

## 13. 数据结构和类关系总结

### 13.1 核心类继承关系图

```
CObject
└── CWorldObject
    └── CSolidObject
        ├── CUnit
        │   ├── CBuilding (UnitTypes/Building.h)
        │   ├── CBuilder (UnitTypes/Builder.h) 
        │   └── CFactory (UnitTypes/Factory.h)
        ├── CFeature
        └── CProjectile
            ├── CWeaponProjectile
            │   ├── CMissileProjectile
            │   ├── CBeamLaserProjectile
            │   ├── CExplosiveProjectile
            │   └── 其他武器投射物
            ├── CFireProjectile
            ├── CFlareProjectile
            └── CPieceProjectile
```

### 13.2 管理器类关系

```cpp
核心管理器系统:
├── UnitHandler          // 单位全局管理器
├── FeatureHandler       // 特征全局管理器  
├── ProjectileHandler    // 投射物全局管理器
├── WeaponDefHandler     // 武器定义管理器
├── UnitDefHandler       // 单位定义管理器
├── FeatureDefHandler    // 特征定义管理器
├── TeamHandler          // 队伍管理器
├── LosHandler           // 视野管理器
├── ResourceHandler      // 资源管理器
├── CategoryHandler      // 类别管理器
├── InterceptHandler     // 拦截管理器
├── CollisionHandler     // 碰撞管理器
└── PathManager          // 路径管理器 (HAPFS/QTPFS)
```

### 13.3 组合关系

**CUnit 组合结构**:
```cpp
CUnit {
    const UnitDef* unitDef;           // 引用定义数据
    AMoveType* moveType;              // 拥有移动类型
    CCommandAI* commandAI;            // 拥有命令AI
    CUnitScript* script;              // 拥有脚本实例
    std::vector<CWeapon*> weapons;    // 拥有武器列表
    CUnit* transporter;               // 引用运输机
    std::list<CUnit*> transportedUnits; // 拥有被运输单位列表
}
```

**CWeapon 组合结构**:
```cpp
CWeapon {
    WeaponDef* weaponDef;            // 引用武器定义
    CUnit* owner;                    // 引用拥有者单位
    SWeaponTarget currentTarget;     // 拥有目标信息
    DynDamageArray* damages;         // 拥有伤害数组
}
```

### 13.4 接口关系

**移动类型接口**:
```cpp
IPathManager* pathManager;           // 路径管理器接口
AMoveType {
    // 统一的移动接口
    virtual void StartMoving(/*...*/);
    virtual void StopMoving();
    virtual void Update();
    virtual void SetGoal(/*...*/);
}
```

**脚本系统接口**:
```cpp
CUnitScript {
    // 脚本引擎接口
    virtual void Create() = 0;
    virtual void Killed() = 0;
    virtual void StartMoving() = 0;
    virtual void StopMoving() = 0;
}
```

### 13.5 事件系统

**依赖关系管理**:
```cpp
// 对象依赖死亡通知系统
class CSolidObject {
    std::vector<CSolidObject*> dependents;    // 依赖者列表
    
    void DependentDied(CSolidObject* dependent);
    void AddDeathDependence(CSolidObject* dependee);
    void DeleteDeathDependence(CSolidObject* dependee);
};
```

**ECS事件系统**:
```cpp
// 碰撞事件
struct FeatureCollisionEvent {
    int unitId;
    int featureId;
    float3 collisionPos;
};

struct UnitCollisionEvent {
    int unit1Id;
    int unit2Id;
    float3 collisionPos;
    float3 collisionVel;
};
```

### 13.6 设计模式应用

1. **工厂模式**:
   - `UnitLoader::LoadUnit()` - 单位创建工厂
   - `WeaponLoader::LoadWeapon()` - 武器创建工厂
   - `MoveTypeFactory::CreateMoveType()` - 移动类型工厂

2. **策略模式**:
   - `AMoveType` 层次 - 不同移动策略
   - `CWeapon` 层次 - 不同武器行为
   - `CCommandAI` 层次 - 不同AI策略

3. **观察者模式**:
   - 死亡依赖系统 - 对象生命周期监听
   - 碰撞事件系统 - 碰撞监听器

4. **单例模式**:
   - 各种Handler - 全局管理器单例
   - `QuadField` - 全局空间索引单例

5. **对象池模式**:
   - `MemPool` 系列 - 对象内存池
   - `SimObjectIDPool` - ID池化管理

6. **状态模式**:
   - `PhysicalState` 位域 - 物理状态机
   - `CommandAI` 状态 - 命令执行状态机

7. **组合模式**:
   - `CUnit` 组合武器、移动类型、AI等组件
   - ECS组件系统 - 灵活的组件组合

这套复杂的类关系和数据结构设计，构成了Recoil Engine强大的RTS游戏模拟能力，能够支持大规模、高性能的实时战略游戏。

---

## 结语

本文档详细分析了Recoil Engine SIM系统的各个组件，从基础对象层次到复杂的AI系统，从物理模拟到路径寻找，展现了一个成熟RTS引擎的技术深度。

该引擎的设计体现了以下技术特点:

- **高性能架构**: 支持数千单位的实时模拟
- **模块化设计**: 清晰的职责分离和接口定义
- **现代化技术**: ECS架构、多线程并行、缓存优化
- **可扩展性**: 脚本系统集成和数据驱动配置
- **工程实践**: 内存管理、性能分析、调试支持

通过深入理解这些系统的实现原理，开发者可以学习到现代游戏引擎设计的精髓，为构建自己的游戏引擎提供宝贵的参考。