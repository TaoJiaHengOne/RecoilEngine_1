/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#ifndef PATHFINDERDEF_HDR
#define PATHFINDERDEF_HDR

#include "Sim/MoveTypes/MoveMath/MoveMath.h"
#include "System/float3.h"
#include "System/type2.h"
#include "System/Rectangle.h"

struct MoveDef;

/**
 * @brief 路径搜索定义基类 - HAPFS寻路系统的核心约束接口
 * 
 * CPathFinderDef是HAPFS(Hierarchical A* Pathfinding System)中所有路径搜索约束的抽象基类。
 * 它定义了路径搜索的基本参数、目标检测、启发式函数计算等核心功能。
 * 
 * 设计模式: 策略模式(Strategy Pattern)
 * - 通过继承实现不同的搜索约束策略(圆形、矩形等)
 * - 提供统一的接口供PathFinder和PathEstimator使用
 * - 支持运行时动态切换不同的约束策略
 * 
 * 核心职责:
 * 1. 定义搜索的起点、终点和目标区域
 * 2. 提供目标检测函数(IsGoal)
 * 3. 计算启发式距离(Heuristic)  
 * 4. 实现搜索范围约束(WithinConstraints)
 * 5. 管理各种搜索行为标志
 * 
 * 使用场景:
 * - PathFinder高精度1x1搜索
 * - PathEstimator宏观16x16/32x32搜索
 * - 各种特殊用途的路径搜索(逃跑、追击、巡逻等)
 */
class CPathFinderDef {
public:
	/**
	 * @brief 构造路径搜索定义对象
	 * 
	 * @param startPos        起始位置(世界坐标系)
	 * @param goalPos         目标位置(世界坐标系)
	 * @param goalRadius      目标区域半径(世界坐标单位)
	 * @param sqGoalDistance  目标距离的平方值(优化性能，避免开方运算)
	 * 
	 * 初始化流程:
	 * 1. 保存世界坐标系的起点和终点
	 * 2. 将世界坐标转换为地图方格坐标
	 * 3. 计算目标半径的平方值
	 * 4. 设置各种搜索行为的默认标志
	 * 5. 预计算一些优化用的数值
	 */
	CPathFinderDef(const float3& startPos, const float3& goalPos, float goalRadius, float sqGoalDistance);
	virtual ~CPathFinderDef() {}

	/**
	 * @brief 检查指定方格是否在搜索约束范围内 (重载版本)
	 * 
	 * @param square 方格坐标(int2格式)
	 * @return true 在约束范围内, false 超出约束范围
	 * 
	 * 这是便利接口，内部调用 WithinConstraints(square.x, square.y)
	 */
	virtual bool WithinConstraints(const int2 square) const { return (WithinConstraints(square.x, square.y)); }
	
	/**
	 * @brief 检查指定方格是否在搜索约束范围内 (纯虚函数)
	 * 
	 * @param xSquare X方格坐标
	 * @param zSquare Z方格坐标  
	 * @return true 在约束范围内, false 超出约束范围
	 * 
	 * 核心约束检测函数，由派生类实现具体的约束逻辑:
	 * - CCircularSearchConstraint: 圆形区域约束
	 * - CRectangularSearchConstraint: 矩形区域约束
	 * - 其他自定义约束类型
	 * 
	 * 性能影响:
	 * - 此函数在A*搜索中被频繁调用
	 * - 应保持O(1)时间复杂度
	 * - 良好的约束设计可减少50-90%的搜索空间
	 */
	virtual bool WithinConstraints(uint32_t xSquare, uint32_t zSquare) const = 0;

	/**
	 * @brief 禁用/启用搜索约束
	 * 
	 * @param b true=禁用约束, false=启用约束
	 * 
	 * 当禁用约束时，WithinConstraints将对所有位置返回true，
	 * 相当于在整个地图范围内进行无约束搜索。
	 * 
	 * 使用场景:
	 * - 调试模式下对比约束效果
	 * - 约束搜索失败后的备选策略
	 * - 特殊情况下需要全地图搜索
	 */
	void DisableConstraint(bool b) { constraintDisabled = b; }
	
	/**
	 * @brief 允许/禁止原始路径搜索
	 * 
	 * @param b true=允许原始路径, false=禁止原始路径
	 * 
	 * 原始路径(Raw Path)是指直线路径检测，用于优化短距离移动:
	 * - 检查起点到终点是否存在直线可通行路径
	 * - 如果可行，直接返回两点连线，跳过A*搜索
	 * - 可显著提升近距离路径搜索性能
	 */
	void AllowRawPathSearch(bool b) { allowRawPath = b; }
	
	/**
	 * @brief 允许/禁止默认路径搜索
	 * 
	 * @param b true=允许默认搜索, false=禁止默认搜索
	 * 
	 * 默认路径搜索是指标准的A*算法搜索。
	 * 禁用后可能只进行原始路径检测或其他特殊搜索策略。
	 */
	void AllowDefPathSearch(bool b) { allowDefPath = b; }

	/**
	 * @brief 检查指定位置是否为目标区域
	 * 
	 * @param squareX 方格X坐标
	 * @param squareZ 方格Z坐标
	 * @return true 是目标区域, false 不是目标区域
	 * 
	 * 目标检测逻辑:
	 * - 计算当前位置到目标位置的距离
	 * - 与目标半径比较，在半径内即为目标
	 * - 支持点目标(半径=0)和区域目标(半径>0)
	 * 
	 * 性能优化:
	 * - 使用平方距离比较避免开方运算
	 * - 预计算目标半径的平方值
	 */
	bool IsGoal(uint32_t squareX, uint32_t squareZ) const;
	
	/**
	 * @brief 检查目标是否被阻挡
	 * 
	 * @param moveDef   移动定义(单位移动能力)
	 * @param blockMask 阻挡类型掩码
	 * @param owner     路径请求的单位对象
	 * @param threadNum 线程编号
	 * @return true 目标被阻挡, false 目标未被阻挡
	 * 
	 * 阻挡检测考虑因素:
	 * - 建筑物阻挡(BLOCK_STRUCTURE)
	 * - 其他单位阻挡(BLOCK_MOBILE) 
	 * - 地形阻挡(不可通行地形)
	 * - 移动类型限制(陆地单位不能走水路等)
	 */
	bool IsGoalBlocked(const MoveDef& moveDef, const CMoveMath::BlockType& blockMask, const CSolidObject* owner, int threadNum) const;

	/**
	 * @brief 计算启发式距离 (4参数版本)
	 * 
	 * @param srcSquareX  源点X坐标
	 * @param srcSquareZ  源点Z坐标
	 * @param tgtSquareX  目标点X坐标
	 * @param tgtSquareZ  目标点Z坐标
	 * @param blockSize   块大小(坐标转换用)
	 * @return 启发式距离值
	 * 
	 * A*算法中的H函数，估算从源点到目标点的距离。
	 * 
	 * 特性要求:
	 * - 可接受性: h(n) ≤ h*(n) (不超估实际距离)
	 * - 一致性: h(n) ≤ c(n,n') + h(n') (三角不等式)
	 * - 信息性: 尽可能接近真实距离以提升搜索效率
	 * 
	 * 实现方式:
	 * - 基础版本使用欧几里得距离
	 * - 可根据移动类型进行调整(考虑地形、高度差等)
	 */
	float Heuristic(uint32_t srcSquareX, uint32_t srcSquareZ, uint32_t tgtSquareX, uint32_t tgtSquareZ, uint32_t blockSize) const;
	
	/**
	 * @brief 计算到预设目标的启发式距离 (2参数版本)
	 * 
	 * @param srcSquareX 源点X坐标
	 * @param srcSquareZ 源点Z坐标
	 * @param blockSize  块大小
	 * @return 启发式距离值
	 * 
	 * 便利函数，目标点使用构造时设定的goalSquareX/goalSquareZ。
	 * 内部调用4参数版本的Heuristic函数。
	 */
	float Heuristic(uint32_t srcSquareX, uint32_t srcSquareZ, uint32_t blockSize) const {
		return (Heuristic(srcSquareX, srcSquareZ, goalSquareX, goalSquareZ, blockSize));
	}

	/**
	 * @brief 计算目标位置的方格偏移量
	 * 
	 * @param blockSize 块大小
	 * @return 目标位置相对于某个基准点的偏移量
	 * 
	 * 用于PathEstimator等宏观搜索中的坐标转换。
	 * 将精细的方格坐标转换为粗粒度的块坐标偏移。
	 */
	int2 GoalSquareOffset(uint32_t blockSize) const;

public:
	// === 世界坐标系位置信息 ===
	
	/**
	 * @brief 世界坐标系起始位置
	 * 
	 * 原始的3D世界坐标，包含x,y,z三个分量。
	 * y分量表示高度，在地形适应性路径搜索中有重要作用。
	 */
	float3 wsStartPos;
	
	/**
	 * @brief 世界坐标系目标位置
	 * 
	 * 原始的3D世界坐标目标点。
	 * 与wsStartPos一起确定路径搜索的基本方向和距离。
	 */
	float3 wsGoalPos;

	// === 目标区域参数 ===
	
	/**
	 * @brief 目标区域半径的平方值
	 * 
	 * 为避免频繁的开方运算，存储半径的平方值。
	 * 在目标检测时直接与平方距离比较，提升性能。
	 * 
	 * 值的含义:
	 * - 0: 精确点目标，必须到达确切位置
	 * - >0: 区域目标，到达半径范围内即可
	 */
	float sqGoalRadius;
	
	/**
	 * @brief 最大原始路径长度
	 * 
	 * 定义什么距离内可以尝试原始路径(直线路径)检测。
	 * 超过此距离将直接使用A*搜索，避免无意义的直线检测。
	 * 
	 * 典型值: 50-200个世界单位，取决于地图复杂度
	 */
	float maxRawPathLen;
	
	/**
	 * @brief 最小原始路径速度修正值
	 * 
	 * 原始路径上各点的最小速度修正值阈值。
	 * 如果路径上任何点的速度修正低于此值，则放弃原始路径。
	 * 
	 * 作用:
	 * - 避免通过泥泞、陡坡等难通行区域的直线路径
	 * - 确保原始路径的通行质量
	 */
	float minRawSpeedMod;

	// === 搜索行为控制标志 ===
	
	/**
	 * @brief 起点是否已在目标半径内
	 * 
	 * 如果为true，表示单位已经在目标区域内，无需路径搜索。
	 * 这是一个重要的早期退出优化，避免不必要的计算。
	 */
	bool startInGoalRadius;
	
	/**
	 * @brief 约束是否被禁用
	 * 
	 * 当为true时，WithinConstraints对所有位置返回true。
	 * 用于调试或特殊情况下的全地图无约束搜索。
	 */
	bool constraintDisabled;
	
	/**
	 * @brief 是否跳过子搜索
	 * 
	 * 在分层路径系统中，控制是否跳过某些层级的搜索。
	 * 用于优化多层级路径细化过程。
	 */
	bool skipSubSearches;

	// === 碰撞和阻挡检测标志 ===
	
	/**
	 * @brief 是否检测移动单位碰撞
	 * 
	 * true: 路径搜索考虑其他移动单位的阻挡
	 * false: 忽略移动单位，只考虑建筑和地形阻挡
	 * 
	 * 性能权衡:
	 * - 启用可获得更准确的路径，但增加计算开销
	 * - 禁用可提升性能，适合静态路径预计算
	 */
	bool testMobile;
	
	/**
	 * @brief 是否需要生成具体路径点
	 * 
	 * true: 生成完整的路径点序列
	 * false: 只检查可达性，不生成具体路径
	 * 
	 * 用途:
	 * - 可达性查询时设为false，节省内存和计算
	 * - 需要跟随路径时设为true
	 */
	bool needPath;
	
	/**
	 * @brief 是否要求精确路径
	 * 
	 * true: 必须到达确切目标，失败时不接受近似解
	 * false: 允许启发式目标，接受"足够接近"的路径
	 * 
	 * 影响:
	 * - 精确路径质量更高但可能搜索失败
	 * - 非精确路径更容易找到解但可能不完美
	 */
	bool exactPath;
	
	/**
	 * @brief 是否允许原始路径搜索
	 * 
	 * 控制是否尝试直线路径检测。
	 * 原始路径是重要的性能优化，对短距离移动效果显著。
	 */
	bool allowRawPath;
	
	/**
	 * @brief 是否允许默认路径搜索
	 * 
	 * 控制是否进行标准A*搜索。
	 * 通常与allowRawPath配合使用，实现多策略路径搜索。
	 */
	bool allowDefPath;
	
	/**
	 * @brief 是否方向无关
	 * 
	 * true: 单位在各个方向的移动能力相同
	 * false: 单位移动具有方向性(如某些地形只能从特定方向通过)
	 * 
	 * 影响启发式函数和成本计算的精度。
	 */
	bool dirIndependent;
	
	/**
	 * @brief 是否为同步搜索
	 * 
	 * true: 同步搜索，用于游戏逻辑确定性
	 * false: 异步搜索，可能用于UI显示或预测
	 * 
	 * 影响:
	 * - 同步搜索结果在多人游戏中保持一致
	 * - 异步搜索允许更多优化但可能不确定
	 */
	bool synced;
	
	/**
	 * @brief 是否使用验证过的起始块
	 * 
	 * true: 起始位置已通过有效性验证
	 * false: 需要对起始位置进行有效性检查
	 * 
	 * 优化标志，避免重复验证已知有效的起始位置。
	 */
	bool useVerifiedStartBlock;

	// === 地图方格坐标 (heightmap coordinates) ===
	
	/**
	 * @brief 起始位置X方格坐标
	 * 
	 * 将wsStartPos.x转换为地图方格坐标系的X坐标。
	 * 转换公式: startSquareX = wsStartPos.x / SQUARE_SIZE
	 */
	uint32_t startSquareX;
	
	/**
	 * @brief 起始位置Z方格坐标
	 * 
	 * 将wsStartPos.z转换为地图方格坐标系的Z坐标。
	 * 转换公式: startSquareZ = wsStartPos.z / SQUARE_SIZE
	 */
	uint32_t startSquareZ;
	
	/**
	 * @brief 目标位置X方格坐标
	 * 
	 * 将wsGoalPos.x转换为地图方格坐标系的X坐标。
	 * 用于启发式函数计算和目标检测。
	 */
	uint32_t goalSquareX;
	
	/**
	 * @brief 目标位置Z方格坐标  
	 * 
	 * 将wsGoalPos.z转换为地图方格坐标系的Z坐标。
	 * 与goalSquareX一起确定目标在地图网格中的位置。
	 */
	uint32_t goalSquareZ;
};

#endif