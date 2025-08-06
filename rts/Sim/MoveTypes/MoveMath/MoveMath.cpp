/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#include "MoveMath.h"

#include "Map/Ground.h"
#include "Map/MapInfo.h"
#include "Sim/Misc/YardmapStatusEffectsMap.h"
#include "Sim/Misc/GlobalSynced.h"
#include "Sim/Misc/GroundBlockingObjectMap.h"
#include "Sim/MoveTypes/MoveDefHandler.h"
#include "Sim/MoveTypes/MoveType.h"
#include "Sim/Objects/SolidObject.h"
#include "Sim/Units/Unit.h"
#include "System/Platform/Threading.h"

#include "System/Misc/TracyDefs.h"

bool CMoveMath::noHoverWaterMove = false;
float CMoveMath::waterDamageCost = 0.0f;

static constexpr int FOOTPRINT_XSTEP = 2;
static constexpr int FOOTPRINT_ZSTEP = 2;

MoveTypes::CheckCollisionQuery::CheckCollisionQuery(const CSolidObject* ref)
	: unit(ref)
	, moveDef(ref->moveDef)
	, pos(ref->pos)
	, physicalState(ref->physicalState)
{
	if (moveDef != nullptr)
		inExitOnlyZone = moveDef->IsInExitOnly(ref->pos);
}

MoveTypes::CheckCollisionQuery::CheckCollisionQuery(const MoveDef* refMoveDef, float3 testPos)
	: moveDef(refMoveDef)
	, pos(testPos.cClampInBounds())
{
	UpdateElevationForPos(pos);
}
/**
 * 根据地图方格坐标更新碰撞查询对象的高度和水中状态
 * 判断是否在水里 增加或者删除一些flag
*/
void MoveTypes::CheckCollisionQuery::UpdateElevationForPos(int2 sqr) {
	const float mapHeight = readMap->GetMaxHeightMapSynced()[sqr.y * mapDims.mapx + sqr.x];
	pos.y = std::max(mapHeight, -moveDef->waterline);

	bool inWater = (pos.y < 0.f);
	if (inWater)
		SetPhysicalStateBit(CSolidObject::PhysicalState::PSTATE_BIT_INWATER);
	else
		ClearPhysicalStateBit(CSolidObject::PhysicalState::PSTATE_BIT_INWATER);
}

float CMoveMath::yLevel(const MoveDef& moveDef, int xSqr, int zSqr)
{
	RECOIL_DETAILED_TRACY_ZONE;
	switch (moveDef.speedModClass) {
		case MoveDef::Tank: // fall-through
		case MoveDef::KBot:  { return (CGround::GetHeightReal      (xSqr * SQUARE_SIZE, zSqr * SQUARE_SIZE) + 10.0f); } break;
		case MoveDef::Hover: { return (CGround::GetHeightAboveWater(xSqr * SQUARE_SIZE, zSqr * SQUARE_SIZE) + 10.0f); } break;
		case MoveDef::Ship:  { return (CGround::GetWaterLevel(xSqr * SQUARE_SIZE, zSqr * SQUARE_SIZE)); } break;
	}

	return 0.0f;
}

float CMoveMath::yLevel(const MoveDef& moveDef, const float3& pos)
{
	RECOIL_DETAILED_TRACY_ZONE;
	switch (moveDef.speedModClass) {
		case MoveDef::Tank: // fall-through
		case MoveDef::KBot:  { return (CGround::GetHeightReal      (pos.x, pos.z) + 10.0f); } break;
		case MoveDef::Hover: { return (CGround::GetHeightAboveWater(pos.x, pos.z) + 10.0f); } break;
		case MoveDef::Ship:  { return (CGround::GetWaterLevel(pos.x, pos.z)); } break;
	}

	return 0.0f;
}



/* calculate the local speed-modifier for this MoveDef */
float CMoveMath::GetPosSpeedMod(const MoveDef& moveDef, unsigned xSquare, unsigned zSquare)
{
	RECOIL_DETAILED_TRACY_ZONE;
	if (xSquare >= mapDims.mapx || zSquare >= mapDims.mapy)
		return 0.0f;

	const int accurateSquare = xSquare + (zSquare * mapDims.mapx);
	const int square = (xSquare >> 1) + ((zSquare >> 1) * mapDims.hmapx);
	const int squareTerrType = readMap->GetTypeMapSynced()[square];

	const float height = readMap->GetMaxHeightMapSynced()[accurateSquare];
	const float slope   = readMap->GetSlopeMapSynced()[square];

	const CMapInfo::TerrainType& tt = mapInfo->terrainTypes[squareTerrType];

	switch (moveDef.speedModClass) {
		case MoveDef::Tank:  { return (GroundSpeedMod(moveDef, height, slope) * tt.tankSpeed ); } break;
		case MoveDef::KBot:  { return (GroundSpeedMod(moveDef, height, slope) * tt.kbotSpeed ); } break;
		case MoveDef::Hover: { return ( HoverSpeedMod(moveDef, height, slope) * tt.hoverSpeed); } break;
		case MoveDef::Ship:  { return (  ShipSpeedMod(moveDef, height, slope) * tt.shipSpeed ); } break;
		default: {} break;
	}

	return 0.0f;
}
/**
 * 这个函数的核心目标是计算一个单位在地图上某个特定方格 (xSquare, zSquare) 上的地形速度修正系数。
 * 与另一个不带 moveDir 参数的版本不同，这个版本的高级之处在于，它不仅考虑了地形的陡峭程度，
 * 还精确地考虑了单位相对于斜坡的移动方向——即单位是在上坡、下坡还是横向坡移动，
 * 并以此对速度做出更精细的调整。这对于“方向性寻路”（Directional Pathing）功能至关重要。
 * 是函数的定义，接收单位的移动定义(moveDef)、要检查的方格坐标(xSquare, zSquare)，以及最重要的——单位的移动方向向量(moveDir)
*/
float CMoveMath::GetPosSpeedMod(const MoveDef& moveDef, unsigned xSquare, unsigned zSquare, float3 moveDir)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// 这是一个边界检查。如果查询的坐标超出了地图范围，直接返回0.0，表示该位置不可通行。
	if (xSquare >= mapDims.mapx || zSquare >= mapDims.mapy)
		return 0.0f;
	// 它计算了两种一维索引：accurateSquare 用于访问全分辨率的地图数据（如高度图），
	// square 用于访问半分辨率的数据（如地形类型图、坡度图），这是一种常见的内存优化。
	const int accurateSquare = xSquare + (zSquare * mapDims.mapx);
	const int square = (xSquare >> 1) + ((zSquare >> 1) * mapDims.hmapx);
	// 根据获取的地形类型的索引
	const int squareTerrType = readMap->GetTypeMapSynced()[square];
	// 高度和梯度
	const float height = readMap->GetMaxHeightMapSynced()[accurateSquare];
	const float slope  = readMap->GetSlopeMapSynced()[square];
	// 地形详情
	const CMapInfo::TerrainType& tt = mapInfo->terrainTypes[squareTerrType];
	// 是此函数的关键步骤之一。它获取了该方格的表面法线向量 (sqrNormal)。法线向量是一个垂直于地面的三维向量，
	// 它精确地描述了斜坡的朝向。例如，一个平地的法线是 (0, 1, 0)，而一个朝向正南方的斜坡，其法线会有一个指向 +z 方向的分量
	const float3 sqrNormal = readMap->GetCenterNormals2DSynced()[xSquare + zSquare * mapDims.mapx];

	// with a flat normal, only consider the normalized xz-direction
	// (the actual steepness is represented by the "slope" variable)
	// we verify that it was normalized in advance
	assert(float3(moveDir).SafeNormalize2D() == moveDir);

	// note: moveDir is (or should be) a unit vector in the xz-plane, y=0
	// scale is negative for "downhill" slopes, positive for "uphill" ones
	// 这是整个函数的核心计算，用于判断上坡还是下坡
	// 如果单位上坡，moveDir 与法线的方向夹角小于90度，点积为正值
	// 如果单位下坡，moveDir 与法线的方向夹角大于90度，点积为负值
	// 如果单位横向坡移动，两者垂直，点积为0
	// 前面的负号将结果反转。因此，dirSlopeMod 的值变为
	// 上坡时为正值 (代表一个速度惩罚)
	// 下坡时为负值 (代表一个速度加成)
	const float dirSlopeMod = -moveDir.dot(sqrNormal);
	// 将前面收集的所有信息——height, slope，以及最关键的 dirSlopeMod——传递给这个辅助函数，计算出综合的地形影响。
	// 最后再乘以该地形对该兵种的基础速度系数 tt.tankSpeed，得出最终的速度修正系数
	switch (moveDef.speedModClass) {
		case MoveDef::Tank:  { return (GroundSpeedMod(moveDef, height, slope, dirSlopeMod) * tt.tankSpeed ); } break;
		case MoveDef::KBot:  { return (GroundSpeedMod(moveDef, height, slope, dirSlopeMod) * tt.kbotSpeed ); } break;
		case MoveDef::Hover: { return ( HoverSpeedMod(moveDef, height, slope, dirSlopeMod) * tt.hoverSpeed); } break;
		case MoveDef::Ship:  { return (  ShipSpeedMod(moveDef, height, slope, dirSlopeMod) * tt.shipSpeed ); } break;
		default: {} break;
	}

	return 0.0f;
}

/* Check if a given square-position is accessible by the MoveDef footprint. */
/*
  函数目的： 检查给定方格位置是否可被MoveDef占地面积访问，不检查速度修正（即不考虑地形因素如坡度、水深等）

  参数说明：
  - moveDef: 移动定义，包含单位的占地面积信息
  - xSquare, zSquare: 要检查的中心方格坐标
  - collider: 碰撞检测对象（可以为nullptr）
  - thread: 线程ID，用于多线程安全
*/
CMoveMath::BlockType CMoveMath::IsBlockedNoSpeedModCheck(const MoveDef& moveDef, int xSquare, int zSquare, const CSolidObject* collider, int thread)
{
	/*
	逻辑分析：
	- 条件判断： 检查 collider 参数是否为空指针
	- 分支1（collider != nullptr）： 使用现有的固体对象创建碰撞查询
		- 会复制该对象的位置、物理状态、移动定义等信息
		- 用于检查现有单位是否能移动到新位置
	- 分支2（collider == nullptr）： 仅使用MoveDef创建碰撞查询
		- 创建虚拟的碰撞查询对象
		- 用于检查某种类型的单位是否能在该位置存在
	*/
	MoveTypes::CheckCollisionQuery collisionQuery = (collider != nullptr)
			? MoveTypes::CheckCollisionQuery(collider)
			: MoveTypes::CheckCollisionQuery(&moveDef);
	/*
		功能解析：
		- int2{xSquare, zSquare}： 创建二维整数坐标
		- UpdateElevationForPos： 更新碰撞查询对象在指定位置的高度和物理状态
			- 从高度图获取该位置的地形高度
			- 确定单位是否在水中（设置PSTATE_BIT_INWATER标志）
			- 计算正确的Y坐标（考虑waterline等因素）
	*/
	collisionQuery.UpdateElevationForPos(int2{xSquare, zSquare});

	return RangeIsBlocked(xSquare - moveDef.xsizeh, xSquare + moveDef.xsizeh, zSquare - moveDef.zsizeh, zSquare + moveDef.zsizeh, &collisionQuery, thread);
}

CMoveMath::BlockType CMoveMath::IsBlockedNoSpeedModCheckDiff(const MoveDef& moveDef, int2 prevSqr, int2 newSqr, const CSolidObject* collider, int thread)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// prev allows {-1, -1} so that the first check is always treated as a full test
	const int prev_xmin = std::max(prevSqr.x - moveDef.xsizeh,               -1);
	const int prev_zmin = std::max(prevSqr.y - moveDef.zsizeh,               -1);
	const int prev_xmax = std::min(prevSqr.x + moveDef.xsizeh, mapDims.mapx - 1);
	const int prev_zmax = std::min(prevSqr.y + moveDef.zsizeh, mapDims.mapy - 1);

	const int xmin = std::max(newSqr.x - moveDef.xsizeh,                0);
	const int zmin = std::max(newSqr.y - moveDef.zsizeh,                0);
	const int xmax = std::min(newSqr.x + moveDef.xsizeh, mapDims.mapx - 1);
	const int zmax = std::min(newSqr.y + moveDef.zsizeh, mapDims.mapy - 1);

	BlockType ret = BLOCK_NONE;

	MoveTypes::CheckCollisionQuery colliderInfo = (collider != nullptr)
			? MoveTypes::CheckCollisionQuery(collider)
			: MoveTypes::CheckCollisionQuery(&moveDef);
	colliderInfo.UpdateElevationForPos(int2{newSqr.x, newSqr.y});

	const int tempNum = gs->GetMtTempNum(thread);

	// footprints are point-symmetric around <xSquare, zSquare>
	for (int z = zmin; z <= zmax; z += FOOTPRINT_ZSTEP) {
		const int zOffset = z * mapDims.mapx;

		for (int x = xmin; x <= xmax; x += FOOTPRINT_XSTEP) {

			if (		z <= prev_zmax && z >= prev_zmin
			 		&& 	x <= prev_xmax && x >= prev_xmin)
				continue;

			const CGroundBlockingObjectMap::BlockingMapCell& cell = groundBlockingObjectMap.GetCellUnsafeConst(zOffset + x);

			for (size_t i = 0, n = cell.size(); i < n; i++) {
				CSolidObject* collidee = cell[i];

				if (collidee->mtTempNum[thread] == tempNum)
					continue;

				collidee->mtTempNum[thread] = tempNum;

				if (((ret |= ObjectBlockType(collidee, &colliderInfo)) & BLOCK_STRUCTURE) == 0)
					continue;

				return ret;
			}
		}
	}

	return ret;
}

bool CMoveMath::CrushResistant(const MoveDef& colliderMD, const CSolidObject* collidee)
{
	RECOIL_DETAILED_TRACY_ZONE;
	if (!collidee->HasCollidableStateBit(CSolidObject::CSTATE_BIT_SOLIDOBJECTS))
		return false;
	if (!collidee->crushable)
		return true;

	return (collidee->crushResistance > colliderMD.crushStrength);
}

bool CMoveMath::IsNonBlocking(const CSolidObject* collidee, const MoveTypes::CheckCollisionQuery* collider)
{
	RECOIL_DETAILED_TRACY_ZONE;
	if (collider->unit == collidee)
		return true;
	if (!collidee->HasCollidableStateBit(CSolidObject::CSTATE_BIT_SOLIDOBJECTS))
		return true;
	// if obstacle is out of map bounds, it cannot block us
	if (!collidee->pos.IsInBounds())
		return true;
	// same if obstacle is not currently marked on blocking-map
	if (!collidee->IsBlocking())
		return true;

	// remaining conditions under which obstacle does NOT block unit
	// only reachable from stand-alone PE invocations or GameHelper
	//   1.
	//      unit is a submarine, obstacle sticks out above-water
	//      (and not itself flagged as a submarine) *OR* unit is
	//      not a submarine and obstacle is (fully under-water or
	//      flagged as a submarine)
	//
	//      NOTE:
	//        do we want to allow submarines to pass underneath
	//        any obstacle even if it is 99% submerged already?
	//
	//        will cause stacking for submarines that are *not*
	//        explicitly flagged as such in their MoveDefs
	//
	// note that these condition(s) can lead to a certain degree of
	// clipping: for full 3D accuracy the height of the MoveDef's
	// owner would need to be accessible, but the path-estimator
	// defs are not tied to any collider instances
	//
	if ( !collider->IsHeightChecksEnabled() ) {
		const bool colliderIsSub = collider->moveDef->isSubmarine;
		const bool collideeIsSub = collidee->moveDef != nullptr && collidee->moveDef->isSubmarine;

		if (colliderIsSub)
			return (!collidee->IsUnderWater() && !collideeIsSub);

		// we don't have height information here so everything above and below water is going to be
		// considered blocking when the unit moveDef is amphibious.
		if (collider->moveDef->followGround)
			return false;

		return (collidee->IsUnderWater() || collideeIsSub);
	}

	// simple case: if unit and obstacle have non-zero
	// vertical separation as measured by their (model)
	// heights, unit can in theory always pass obstacle
	//
	// this allows (units marked as) submarines to both
	// *pass* and *short-range path* underneath floating
	// DT, or ships to P&SRP over underwater structures
	//
	// specifically restricted to units *inside* water
	// because it can have the unwanted side-effect of
	// enabling the PFS to generate paths for units on
	// steep slopes *through* obstacles, either higher
	// up or lower down
	//
	if (collider->IsInWater() && collidee->IsInWater()) {
		float colliderHeight = (collider->moveDef != nullptr) ? collider->moveDef->height : math::fabs(collider->unit->height);
		if ((collider->pos.y + colliderHeight) < collidee->pos.y)
			return true;

		float collideeHeight = (collidee->moveDef != nullptr) ? collidee->moveDef->height : math::fabs(collidee->height);
		if ((collidee->pos.y + collideeHeight) < collider->pos.y)
			return true;
	}
	return false;
}

/**
 * 功能：确定一个固体对象对碰撞查询单位的阻挡类型
 * 这是碰撞检测系统的核心函数，将对象分类为不同的阻挡类型，
 * 以便移动系统做出相应的路径规划决策
 * 
 * 参数说明：
 * - collidee: 被检测的固体对象（潜在的阻挡物）
 * - collider: 碰撞查询对象（要移动的单位的信息）
 * 
 * 返回值：BlockType枚举，表示阻挡类型
 */
CMoveMath::BlockType CMoveMath::ObjectBlockType(const CSolidObject* collidee, const MoveTypes::CheckCollisionQuery* collider)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// 第一步：检查是否为非阻挡对象
	// IsNonBlocking函数检查多种不阻挡的情况：
	// - 对象是查询单位自己
	// - 对象未启用碰撞
	// - 对象在地图边界外
	// - 对象未在阻挡图上标记
	// - 潜艇vs水面对象的特殊规则
	// - 垂直分离情况（如：潜艇穿过水面建筑）
	if (IsNonBlocking(collidee, collider))
		return BLOCK_NONE;

	// 第二步：处理不可移动对象（建筑物、地形特征等）
	if (collidee->immobile)
		// 检查碰撞单位是否有足够的碾压力来摧毁该对象
		// CrushResistant检查：collidee->crushResistance > collider->moveDef->crushStrength
		// 如果无法碾压，则视为结构性阻挡；如果可以碾压，则不阻挡
		return ((CrushResistant(*(collider->moveDef), collidee))? BLOCK_STRUCTURE: BLOCK_NONE);

	// 第三步：处理可移动对象（单位）
	// 将固体对象强制转换为单位对象，因为只有单位是可移动的
	const CUnit* u = static_cast<const CUnit*>(collidee);
	const AMoveType* mt = u->moveType;

	// 第四步：检查单位是否正在移动
	// 正在移动的单位可能正在执行移动命令或跟随路径
	// 这类单位通常会自主避让，所以标记为BLOCK_MOVING
	if (u->IsMoving())
		return BLOCK_MOVING;

	// 第五步：检查静止单位是否可推动
	// IsPushResistant检查单位是否拒绝被推动
	// 不可推动的静止单位应被视为结构性阻挡
	if (mt->IsPushResistant())
		return BLOCK_STRUCTURE;

	// 第六步：处理可推动的静止单位
	// 区分闲置单位和忙碌单位：
	// - 闲置单位(IsIdle)：没有任何命令，容易被推动 -> BLOCK_MOBILE
	// - 忙碌单位：正在执行命令但未移动（如建造、攻击），较难被推动 -> BLOCK_MOBILE_BUSY
	// 注意：正在建造的单位永远不会被认为是闲置的，
	// 但在当前实现中它们被归类为BLOCK_MOBILE_BUSY而不是BLOCK_STRUCTURE
	return ((u->IsIdle())? BLOCK_MOBILE: BLOCK_MOBILE_BUSY);
}

CMoveMath::BlockType CMoveMath::SquareIsBlocked(const MoveDef& moveDef, int xSquare, int zSquare, MoveTypes::CheckCollisionQuery* collider)
{
	RECOIL_DETAILED_TRACY_ZONE;
	if (static_cast<unsigned>(xSquare) >= mapDims.mapx || static_cast<unsigned>(zSquare) >= mapDims.mapy)
		return BLOCK_IMPASSABLE;

	collider->UpdateElevationForPos(int2(xSquare, zSquare));
	BlockType r = BLOCK_NONE;

	const CGroundBlockingObjectMap::BlockingMapCell& cell = groundBlockingObjectMap.GetCellUnsafeConst(zSquare * mapDims.mapx + xSquare);

	for (size_t i = 0, n = cell.size(); i < n; i++) {
		r |= ObjectBlockType(cell[i], collider);
	}

	return r;
}

/*
  功能：检查指定矩形范围内是否存在阻挡，这是单位占地面积碰撞检测的核心函数。

  参数说明：
  - xmin, xmax, zmin, zmax: 要检查的矩形范围（地图方格坐标）
  - collider: 碰撞查询对象，包含单位信息
  - thread: 线程ID，用于多线程安全
*/
CMoveMath::BlockType CMoveMath::RangeIsBlocked(int xmin, int xmax, int zmin, int zmax, const MoveTypes::CheckCollisionQuery* collider, int thread)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// 边界范围修正
	xmin = std::max(xmin,                0);
	zmin = std::max(zmin,                0);
	xmax = std::min(xmax, mapDims.mapx - 1);
	zmax = std::min(zmax, mapDims.mapy - 1);
	// 功能：初始化阻挡类型为无阻挡，后续会通过位运算累积各种阻挡类型。
	BlockType ret = BLOCK_NONE;
	// - ThreadPool::inMultiThreadedSection：全局标志，指示当前是否在多线程环境中
	if (ThreadPool::inMultiThreadedSection) { // 指示当前是否在多线程环境中
		const int tempNum = gs->GetMtTempNum(thread);
		ret = CMoveMath::RangeIsBlockedMt(xmin, xmax, zmin, zmax, collider, thread, tempNum);
	} else {
		const int tempNum = gs->GetTempNum();
		ret = CMoveMath::RangeIsBlockedSt(xmin, xmax, zmin, zmax, collider, tempNum);
	}

	return ret;
}

CMoveMath::BlockType CMoveMath::RangeIsBlockedTempNum(int xmin, int xmax, int zmin, int zmax, const MoveTypes::CheckCollisionQuery* collider, int tempNum, int thread)
{
	RECOIL_DETAILED_TRACY_ZONE;
	xmin = std::max(xmin,                0);
	zmin = std::max(zmin,                0);
	xmax = std::min(xmax, mapDims.mapx - 1);
	zmax = std::min(zmax, mapDims.mapy - 1);

	BlockType ret = BLOCK_NONE;
	if (ThreadPool::inMultiThreadedSection) {
		const int tempNum = gs->GetMtTempNum(thread);
		ret = CMoveMath::RangeIsBlockedHashedMt(xmin, xmax, zmin, zmax, collider, tempNum, thread);
	} else {
		const int tempNum = gs->GetTempNum();
		ret = CMoveMath::RangeIsBlockedHashedSt(xmin, xmax, zmin, zmax, collider, tempNum);
	}

	return ret;
}

/*
  功能：单线程版本的范围阻挡检测，检查指定矩形区域内的所有阻挡对象。

  参数说明：
  - xmin, xmax, zmin, zmax: 要检查的矩形范围
  - collider: 碰撞查询对象
  - tempNum: 临时编号，用于避免重复检查同一对象
*/
CMoveMath::BlockType CMoveMath::RangeIsBlockedSt(int xmin, int xmax, int zmin, int zmax, const MoveTypes::CheckCollisionQuery* collider, int tempNum)
{
	RECOIL_DETAILED_TRACY_ZONE;
	//  功能：初始化阻挡类型为无阻挡，后续通过位运算累积各种阻挡类型
	BlockType ret = BLOCK_NONE;

	// footprints are point-symmetric around <xSquare, zSquare>
	for (int z = zmin; z <= zmax; z += FOOTPRINT_ZSTEP) {
		const int zOffset = z * mapDims.mapx;

		for (int x = xmin; x <= xmax; x += FOOTPRINT_XSTEP) {
			// 获取碰撞单元格
			/*
			1. groundBlockingObjectMap：全局地面阻挡对象图
				- 将地图分割为网格，每个网格存储该位置的所有阻挡对象
				- 空间分割优化，避免检查整个地图的所有对象
			2. zOffset + x：计算1D索引
				- 等价于 z * mapWidth + x
				- 将2D坐标(x,z)转换为1D数组索引
			3. GetCellUnsafeConst()：
				- "Unsafe"：不进行边界检查，假定调用者已确保索引有效
				- "Const"：返回只读引用
				- 返回该位置的所有阻挡对象列表
			4. BlockingMapCell：
				- 通常是 std::vector<CSolidObject*>
				- 存储该网格中的所有固体对象
			*/
			const CGroundBlockingObjectMap::BlockingMapCell& cell = groundBlockingObjectMap.GetCellUnsafeConst(zOffset + x);

			for (size_t i = 0, n = cell.size(); i < n; i++) {
				// 功能：遍历当前网格中的所有固体对象
				CSolidObject* collidee = cell[i];
				// 第454-455行：重复检查避免机制
				/*
				// 为什么需要避免重复检查？
				// 考虑一个3x3的坦克占地面积：
				//
				//   [A][B][C]
				//   [D][E][F]  ← 坦克占地
				//   [G][H][I]
				//
				// 如果有一个大建筑跨越多个网格：
				//   [建筑][建筑]
				//   [建筑][建筑] ← 同一建筑出现在4个网格中
				//
				// 没有tempNum机制：会检查同一建筑4次
				// 有tempNum机制：只检查第一次遇到时

				tempNum的工作原理：
				- 每次范围检查开始时，系统分配一个新的唯一tempNum
				- 首次遇到对象时，对象的tempNum不等于当前tempNum
				- 检查该对象，然后将其tempNum设置为当前值
				- 再次遇到该对象时，tempNum相等，直接跳过
				*/
				if (collidee->tempNum == tempNum)
					continue;

				collidee->tempNum = tempNum;

				if (((ret |= ObjectBlockType(collidee, collider)) & BLOCK_STRUCTURE) == 0)
					continue;

				return ret;
			}
		}
	}

	return ret;
}


CMoveMath::BlockType CMoveMath::RangeIsBlockedMt(int xmin, int xmax, int zmin, int zmax, const MoveTypes::CheckCollisionQuery* collider, int thread, int tempNum)
{
	RECOIL_DETAILED_TRACY_ZONE;
	BlockType ret = BLOCK_NONE;

	// footprints are point-symmetric around <xSquare, zSquare>
	for (int z = zmin; z <= zmax; z += FOOTPRINT_ZSTEP) {
		const int zOffset = z * mapDims.mapx;

		for (int x = xmin; x <= xmax; x += FOOTPRINT_XSTEP) {
			const CGroundBlockingObjectMap::BlockingMapCell& cell = groundBlockingObjectMap.GetCellUnsafeConst(zOffset + x);

			for (size_t i = 0, n = cell.size(); i < n; i++) {
				CSolidObject* collidee = cell[i];

				if (collidee->mtTempNum[thread] == tempNum)
					continue;

				collidee->mtTempNum[thread] = tempNum;

				if (((ret |= ObjectBlockType(collidee, collider)) & BLOCK_STRUCTURE) == 0)
					continue;

				return ret;
			}
		}
	}

	return ret;
}

CMoveMath::BlockType CMoveMath::RangeIsBlockedHashedSt(int xmin, int xmax, int zmin, int zmax, const MoveTypes::CheckCollisionQuery* collider, int tempNum)
{
	RECOIL_DETAILED_TRACY_ZONE;
	BlockType ret = BLOCK_NONE;

	static spring::unordered_map<CSolidObject*, CMoveMath::BlockType> blockMap(10);
	static int lastTempNum = -1;

	if (lastTempNum != tempNum){
		blockMap.clear();
		lastTempNum = tempNum;
	}

	// footprints are point-symmetric around <xSquare, zSquare>
	for (int z = zmin; z <= zmax; z += FOOTPRINT_ZSTEP) {
		const int zOffset = z * mapDims.mapx;

		for (int x = xmin; x <= xmax; x += FOOTPRINT_XSTEP) {
			const CGroundBlockingObjectMap::BlockingMapCell& cell = groundBlockingObjectMap.GetCellUnsafeConst(zOffset + x);

			for (size_t i = 0, n = cell.size(); i < n; i++) {
				CSolidObject* collidee = cell[i];

				auto blockMapResult = blockMap.find(collidee);
				if (blockMapResult == blockMap.end()) {
					blockMapResult = blockMap.emplace(collidee, ObjectBlockType(collidee, collider)).first;
				}

				ret |= blockMapResult->second;

				if ((ret & BLOCK_STRUCTURE) == 0)
					continue;

				return ret;
			}
		}
	}

	return ret;
}

static std::array<spring::unordered_map<CSolidObject*, CMoveMath::BlockType>, ThreadPool::MAX_THREADS> blockMaps;
static std::array<int, ThreadPool::MAX_THREADS> lastTempNums;

// Called by GeneralMoveSystem::Init()
void CMoveMath::InitRangeIsBlockedHashes() {
	for (auto& blockMap : blockMaps) {
		blockMap.reserve(10);
	}
	for (auto& lastTempNum : lastTempNums) {
		lastTempNum = -1;
	}
}

CMoveMath::BlockType CMoveMath::RangeIsBlockedHashedMt(int xmin, int xmax, int zmin, int zmax, const MoveTypes::CheckCollisionQuery* collider, int tempNum, int thread)
{
	RECOIL_DETAILED_TRACY_ZONE;
	BlockType ret = BLOCK_NONE;

	spring::unordered_map<CSolidObject*, CMoveMath::BlockType>& blockMap = blockMaps[thread];
	int& lastTempNum = lastTempNums[thread];

	if (lastTempNum != tempNum){
		blockMap.clear();
		lastTempNum = tempNum;
	}

	// footprints are point-symmetric around <xSquare, zSquare>
	for (int z = zmin; z <= zmax; z += FOOTPRINT_ZSTEP) {
		const int zOffset = z * mapDims.mapx;

		for (int x = xmin; x <= xmax; x += FOOTPRINT_XSTEP) {
			const CGroundBlockingObjectMap::BlockingMapCell& cell = groundBlockingObjectMap.GetCellUnsafeConst(zOffset + x);

			for (size_t i = 0, n = cell.size(); i < n; i++) {
				CSolidObject* collidee = cell[i];
  
				auto blockMapResult = blockMap.find(collidee);
				if (blockMapResult == blockMap.end()) {
					blockMapResult = blockMap.emplace(collidee, ObjectBlockType(collidee, collider)).first;
				}

				ret |= blockMapResult->second;

				if ((ret & BLOCK_STRUCTURE) == 0)
					continue;

				return ret;
			}
		}
	}

	return ret;
}

void CMoveMath::FloodFillRangeIsBlocked(const MoveDef& moveDef, const CSolidObject* collider, const SRectangle& areaToSample, std::vector<std::uint8_t>& results, int thread)
{
	RECOIL_DETAILED_TRACY_ZONE;
	spring::unordered_map<CSolidObject*, CMoveMath::BlockType>& blockMap = blockMaps[thread];
	blockMap.clear();

	results.clear();
	results.reserve(areaToSample.GetArea());

	MoveTypes::CheckCollisionQuery colliderInfo = (collider != nullptr)
			? MoveTypes::CheckCollisionQuery(collider)
			: MoveTypes::CheckCollisionQuery(&moveDef, {float(areaToSample.x1*SQUARE_SIZE), 0.f, float(areaToSample.z1*SQUARE_SIZE)});

	for (int z = areaToSample.z1; z < areaToSample.z2; ++z) {
		const int zOffset = z * mapDims.mapx;

		for (int x = areaToSample.x1; x < areaToSample.x2; ++x) {
			const CGroundBlockingObjectMap::BlockingMapCell& cell = groundBlockingObjectMap.GetCellUnsafeConst(zOffset + x);
			BlockType ret = BLOCK_NONE;

			for (size_t i = 0, n = cell.size(); i < n; i++) {
				CSolidObject* collidee = cell[i];

				auto blockMapResult = blockMap.find(collidee);
				if (blockMapResult == blockMap.end()) {
					blockMapResult = blockMap.emplace(collidee, ObjectBlockType(collidee, &colliderInfo)).first;
				}

				ret |= blockMapResult->second;

				if ((ret & BLOCK_STRUCTURE) != 0)
					break;
			}
			results.emplace_back(ret);
		}
	}
}

// 这个函数的作用是检查一个指定的矩形区域内，是否存在任何一个被标记为“仅允许离开” (Exit Only) 的方格。它被 MoveDef::IsInExitOnly 调用，
// 用于判断一个单位的完整占地面积是否与工厂出口等特殊区域重叠。
// for 循环的步长是 += 2 而不是 += 1。这意味着它在检查时特意跳过了某些格子。
// 这不是一个bug，而是一项基于底层数据结构的关键性能优化。
// 原因：庭院图 (Yardmap) 的数据存储方式
// 低分辨率定义: 在单位的定义文件中，庭院图（Yardmap）的数据默认是以半精度（half-resolution）定义的。
// 这意味着，源数据中的一个点，实际上对应了最终高精度地图上的一个 2x2 的方格区域。
// 2x2 上采样 (Upsampling): 在游戏加载并创建单位的庭院图时，系统会将这个低精度的定义进行“上采样”。
// 一个低精度点的值（例如 YARDMAP_EXITONLY）会被复制到高精度碰撞图上的整个 2x2 区域。
bool CMoveMath::RangeHasExitOnly(int xmin, int xmax, int zmin, int zmax, const ObjectCollisionMapHelper& object) {
	for (int z = zmin; z <= zmax; z += FOOTPRINT_ZSTEP) {
		for (int x = xmin; x <= xmax; x += FOOTPRINT_XSTEP)
			// 检查这个格子有没有被标记为 Exit Only
			if (object.IsExitOnlyAt(x, z)) return true;
	}
	return false;
}

