/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

// #undef NDEBUG

#include "GroundMoveType.h"
#include "MoveDefHandler.h"
#include "Components/MoveTypesComponents.h"
#include "ExternalAI/EngineOutHandler.h"
#include "Game/Camera.h"
#include "Game/GameHelper.h"
#include "Game/GlobalUnsynced.h"
#include "Game/SelectedUnitsHandler.h"
#include "Game/Players/Player.h"
#include "Map/Ground.h"
#include "Map/MapInfo.h"
#include "Map/ReadMap.h"
#include "MoveMath/MoveMath.h"
#include "Sim/Ecs/Registry.h"
#include "Sim/Features/Feature.h"
#include "Sim/Features/FeatureHandler.h"
#include "Sim/Misc/GeometricObjects.h"
#include "Sim/Misc/ModInfo.h"
#include "Sim/Misc/QuadField.h"
#include "Sim/Misc/TeamHandler.h"
#include "Sim/Path/IPathManager.h"
#include "Sim/Units/Scripts/CobInstance.h"
#include "Sim/Units/CommandAI/CommandAI.h"
#include "Sim/Units/CommandAI/MobileCAI.h"
#include "Sim/Units/UnitDef.h"
#include "Sim/Units/UnitDefHandler.h"
#include "Sim/Units/UnitHandler.h"
#include "Sim/Weapons/WeaponDefHandler.h"
#include "Sim/Weapons/Weapon.h"
#include "System/creg/STL_Tuple.h"
#include "System/EventHandler.h"
#include "System/Log/ILog.h"
#include "System/FastMath.h"
#include "System/SpringMath.h"
#include "System/TimeProfiler.h"
#include "System/type2.h"
#include "System/Sound/ISoundChannels.h"
#include "System/SpringHash.h"
#include "Utils/UnitTrapCheckUtils.h"

#include "System/Misc/TracyDefs.h"

// #define PATHING_DEBUG

#ifdef PATHING_DEBUG
#include <sim/Path/HAPFS/PathGlobal.h>
#include "System/Threading/ThreadPool.h"
#endif

#if 1
#include "Rendering/IPathDrawer.h"
#define DEBUG_DRAWING_ENABLED ((gs->cheatEnabled || gu->spectatingFullView) && pathDrawer->IsEnabled())
spring::spinlock geometryLock;
#else
#define DEBUG_DRAWING_ENABLED false
#endif

using namespace MoveTypes;

#define LOG_SECTION_GMT "GroundMoveType"
LOG_REGISTER_SECTION_GLOBAL(LOG_SECTION_GMT)

// use the specific section for all LOG*() calls in this source file
#ifdef LOG_SECTION_CURRENT
	#undef LOG_SECTION_CURRENT
#endif
#define LOG_SECTION_CURRENT LOG_SECTION_GMT


// speeds near (MAX_UNIT_SPEED * 1e1) elmos / frame can be caused by explosion impulses
// CUnitHandler removes units with speeds > MAX_UNIT_SPEED as soon as they exit the map,
// so the assertion can be less strict
#define ASSERT_SANE_OWNER_SPEED(v) assert(v.SqLength() < (MAX_UNIT_SPEED * MAX_UNIT_SPEED * 1e2));

// magic number to reduce damage taken from collisions
// between a very heavy and a very light CSolidObject
#define COLLISION_DAMAGE_MULT    0.02f

#define MAX_IDLING_SLOWUPDATES     16
#define IGNORE_OBSTACLES            0
#define WAIT_FOR_PATH               0
#define MODEL_TURN_INERTIA          1

#define UNIT_EVENTS_RESERVE			8

#define UNIT_CMD_QUE_SIZE(u) (u->commandAI->commandQue.size())
// Not using IsMoveCommand on purpose, as the following is changing the effective goalRadius
#define UNIT_HAS_MOVE_CMD(u) (u->commandAI->commandQue.empty() || u->commandAI->commandQue[0].GetID() == CMD_MOVE || u->commandAI->commandQue[0].GetID() == CMD_FIGHT)

#define WAYPOINT_RADIUS (1.25f * SQUARE_SIZE)

#define MAXREVERSESPEED_MEMBER_IDX 7

#define MEMBER_CHARPTR_HASH(memberName) spring::LiteHash(memberName, strlen(memberName),     0)
#define MEMBER_LITERAL_HASH(memberName) spring::LiteHash(memberName, sizeof(memberName) - 1, 0)

CR_BIND_DERIVED(CGroundMoveType, AMoveType, (nullptr))
CR_REG_METADATA(CGroundMoveType, (
	CR_IGNORED(pathController),

	CR_IGNORED(jobId),

	CR_MEMBER(currWayPoint),
	CR_MEMBER(nextWayPoint),

	CR_MEMBER(earlyCurrWayPoint),
	CR_MEMBER(earlyNextWayPoint),

	CR_MEMBER(waypointDir),
	CR_MEMBER(flatFrontDir),
	CR_MEMBER(lastAvoidanceDir),
	CR_MEMBER(mainHeadingPos),
	CR_MEMBER(skidRotVector),

	CR_MEMBER(turnRate),
	CR_MEMBER(turnSpeed),
	CR_MEMBER(turnAccel),
	CR_MEMBER(accRate),
	CR_MEMBER(decRate),
	CR_MEMBER(myGravity),

	CR_MEMBER(maxReverseDist),
	CR_MEMBER(minReverseAngle),
	CR_MEMBER(maxReverseSpeed),
	CR_MEMBER(sqSkidSpeedMult),

	CR_MEMBER(wantedSpeed),
	CR_MEMBER(currentSpeed),
	CR_MEMBER(deltaSpeed),

	CR_MEMBER(currWayPointDist),
	CR_MEMBER(prevWayPointDist),
	CR_MEMBER(goalRadius),
	CR_MEMBER(ownerRadius),
	CR_MEMBER(extraRadius),

	CR_MEMBER(skidRotSpeed),
	CR_MEMBER(skidRotAccel),

	CR_MEMBER(resultantForces),
	CR_MEMBER(forceFromMovingCollidees),
	CR_MEMBER(forceFromStaticCollidees),

	CR_MEMBER(pathID),
	CR_MEMBER(nextPathId),
	CR_MEMBER(deletePathId),

	CR_MEMBER(numIdlingUpdates),
	CR_MEMBER(numIdlingSlowUpdates),

	CR_MEMBER(wantedHeading),
	CR_MEMBER(minScriptChangeHeading),

	CR_MEMBER(wantRepathFrame),
	CR_MEMBER(lastRepathFrame),
	CR_MEMBER(bestLastWaypointDist),
	CR_MEMBER(bestReattemptedLastWaypointDist),
	CR_MEMBER(setHeading),
	CR_MEMBER(setHeadingDir),
	CR_MEMBER(limitSpeedForTurning),

	CR_MEMBER(oldSpeed),
	CR_MEMBER(newSpeed),

	CR_MEMBER(atGoal),
	CR_MEMBER(atEndOfPath),
	CR_MEMBER(wantRepath),
	CR_MEMBER(lastWaypoint),

	CR_MEMBER(reversing),
	CR_MEMBER(idling),
	CR_MEMBER(pushResistant),
	CR_MEMBER(pushResistanceBlockActive),
	CR_MEMBER(canReverse),
	CR_MEMBER(useMainHeading),
	CR_MEMBER(useRawMovement),
	CR_MEMBER(pathingFailed),
	CR_MEMBER(pathingArrived),
	CR_MEMBER(positionStuck),
	CR_MEMBER(forceStaticObjectCheck),
	CR_MEMBER(avoidingUnits),

	CR_POSTLOAD(PostLoad),
	CR_PREALLOC(GetPreallocContainer)
))



static CGroundMoveType::MemberData gmtMemberData = {
	{{
		std::pair<unsigned int,  bool*>{MEMBER_LITERAL_HASH(       "atGoal"), nullptr},
		std::pair<unsigned int,  bool*>{MEMBER_LITERAL_HASH(  "atEndOfPath"), nullptr},
		std::pair<unsigned int,  bool*>{MEMBER_LITERAL_HASH("pushResistant"), nullptr},
	}},
	{{
		std::pair<unsigned int, short*>{MEMBER_LITERAL_HASH("minScriptChangeHeading"), nullptr},
	}},
	{{
		std::pair<unsigned int, float*>{MEMBER_LITERAL_HASH(       "turnRate"), nullptr},
		std::pair<unsigned int, float*>{MEMBER_LITERAL_HASH(      "turnAccel"), nullptr},
		std::pair<unsigned int, float*>{MEMBER_LITERAL_HASH(        "accRate"), nullptr},
		std::pair<unsigned int, float*>{MEMBER_LITERAL_HASH(        "decRate"), nullptr},
		std::pair<unsigned int, float*>{MEMBER_LITERAL_HASH(      "myGravity"), nullptr},
		std::pair<unsigned int, float*>{MEMBER_LITERAL_HASH( "maxReverseDist"), nullptr},
		std::pair<unsigned int, float*>{MEMBER_LITERAL_HASH("minReverseAngle"), nullptr},
		std::pair<unsigned int, float*>{MEMBER_LITERAL_HASH("maxReverseSpeed"), nullptr},
		std::pair<unsigned int, float*>{MEMBER_LITERAL_HASH("sqSkidSpeedMult"), nullptr},
	}},
};




namespace SAT {
	static float CalcSeparatingDist(
		const float3& axis,
		const float3& zdir,
		const float3& xdir,
		const float3& sepv,
		const float2& size
	) {
		const float axisDist = math::fabs(axis.dot(sepv)         );
		const float xdirDist = math::fabs(axis.dot(xdir) * size.x);
		const float zdirDist = math::fabs(axis.dot(zdir) * size.y);

		return (axisDist - zdirDist - xdirDist);
	}

	static bool HaveSeparatingAxis(
		const CSolidObject* collider,
		const CSolidObject* collidee,
		const MoveDef* colliderMD,
		const MoveDef* collideeMD,
		const float3& separationVec
	) {
		// const float2 colliderSize = (colliderMD != nullptr)? colliderMD->GetFootPrint(0.5f * SQUARE_SIZE): collider->GetFootPrint(0.5f * SQUARE_SIZE);
		const float2 colliderSize =                          colliderMD->GetFootPrint(0.5f * SQUARE_SIZE)                                            ;
		const float2 collideeSize = (collideeMD != nullptr)? collideeMD->GetFootPrint(0.5f * SQUARE_SIZE): collidee->GetFootPrint(0.5f * SQUARE_SIZE);

		// true if no overlap on at least one axis
		bool haveAxis = false;

		haveAxis = haveAxis || (CalcSeparatingDist(collider->frontdir,  collidee->frontdir, collidee->rightdir,  separationVec, collideeSize) > colliderSize.y);
		haveAxis = haveAxis || (CalcSeparatingDist(collider->rightdir,  collidee->frontdir, collidee->rightdir,  separationVec, collideeSize) > colliderSize.x);
		haveAxis = haveAxis || (CalcSeparatingDist(collidee->frontdir,  collider->frontdir, collider->rightdir,  separationVec, colliderSize) > collideeSize.y);
		haveAxis = haveAxis || (CalcSeparatingDist(collidee->rightdir,  collider->frontdir, collider->rightdir,  separationVec, colliderSize) > collideeSize.x);
		return haveAxis;
	}
};


static bool CheckCollisionExclSAT(
	const float4& separationVec,
	const CSolidObject* collider = nullptr,
	const CSolidObject* collidee = nullptr,
	const MoveDef* colliderMD = nullptr,
	const MoveDef* collideeMD = nullptr
) {
	return ((separationVec.SqLength() - separationVec.w) <= 0.01f);
}

static bool CheckCollisionInclSAT(
	const float4& separationVec,
	const CSolidObject* collider = nullptr,
	const CSolidObject* collidee = nullptr,
	const MoveDef* colliderMD = nullptr,
	const MoveDef* collideeMD = nullptr
) {
	assert(collider   != nullptr);
	assert(collidee   != nullptr);
	assert(colliderMD != nullptr);

	return (CheckCollisionExclSAT(separationVec) && !SAT::HaveSeparatingAxis(collider, collidee, colliderMD, collideeMD, separationVec));
}



static void HandleUnitCollisionsAux(
	const CUnit* collider,
	const CUnit* collidee,
	CGroundMoveType* gmtCollider,
	CGroundMoveType* gmtCollidee
) {
	RECOIL_DETAILED_TRACY_ZONE;
	if (!collider->IsMoving() || gmtCollider->progressState != AMoveType::Active)
		return;

	// if collidee shares our goal position and is no longer
	// moving along its path, trigger Arrived() to kill long
	// pushing contests
	//
	// check the progress-states so collisions with units which
	// failed to reach goalPos for whatever reason do not count
	// (or those that still have orders)
	//
	// CFactory applies random jitter to otherwise equal goal
	// positions of at most TWOPI elmos, use half as threshold
	// (simply bail if distance between collider and collidee
	// goal-positions exceeds PI)
	// if (!gmtCollider->IsAtGoalPos(gmtCollidee->goalPos, math::PI))
	// 	return;

	switch (gmtCollidee->progressState) {
		case AMoveType::Done: {
			if (collidee->IsMoving() || UNIT_CMD_QUE_SIZE(collidee) != 0)
				return;

			float separationDist = std::max(collider->unitDef->separationDistance, collidee->unitDef->separationDistance);
			const bool triggerArrived = gmtCollider->IsAtGoalPos(collidee->pos, gmtCollidee->GetOwnerRadius() + separationDist);
			if (triggerArrived) {
				gmtCollider->TriggerCallArrived();
			} else {
				// if the collidee is touching the waypoint, then also switch to the next waypoint.
				const float3& currWaypoint = gmtCollider->GetCurrWayPoint();
				const float collideeToCurrDistSq = currWaypoint.SqDistance2D(collidee->pos);
				const float collideeGoalRadius = gmtCollidee->GetOwnerRadius();

				if (collideeToCurrDistSq <= Square(collideeGoalRadius+separationDist)) {
					gmtCollider->TriggerSkipWayPoint();
					return;
				}
			}
		} break;

		case AMoveType::Active: {
			// collider and collidee are both actively moving and share the same goal position
			// (i.e. a traffic jam) so ignore current waypoint and go directly to the next one
			// or just make collider give up if already within footprint radius.
			if (gmtCollidee->GetCurrWayPoint() == gmtCollider->GetNextWayPoint()) {
				// const float3& currWaypoint = gmtCollider->GetCurrWayPoint();
				// const float3& nextWaypoint = gmtCollider->GetNextWayPoint();

				// const float unitToNextDistSq = nextWaypoint.SqDistance2D(collider->pos);
				// const float currToNextDistSq = nextWaypoint.SqDistance2D(currWaypoint);

				// // Switch waypoints if the current waypoint is effectively sending us in the
				// // wrong direction. This can happen as units push each other around. This check
				// // is important to prevent units in a long line back-propagating the next
				// // waypoint, which could cause units to try and cut corners, which could cause
				// // them to be unable to path around obstacles.
				// if (unitToNextDistSq <= currToNextDistSq) {
					gmtCollider->TriggerSkipWayPoint();
					return;
				// }
			}

			// Several large units can end up surrounding a waypoint and block each other from touching it, because it
			// sits diagonally from all of them and outside their radius.
			// Adding an extra diagonal distance compensation will allow such large units to "touch" the waypoint.
			constexpr float adjustForDiagonal = 1.45f;
			float separationDist = std::max(collider->unitDef->separationDistance, collidee->unitDef->separationDistance);
			const bool triggerArrived = (gmtCollider->IsAtGoalPos(collider->pos, (gmtCollider->GetOwnerRadius() + separationDist)*adjustForDiagonal)
										|| gmtCollider->IsAtGoalPos(collidee->pos, (gmtCollidee->GetOwnerRadius() + separationDist)*adjustForDiagonal));
			if (triggerArrived) {
				gmtCollider->TriggerCallArrived();
			} else {
				// if the collidee is touching the waypoint, then also switch to the next
				// waypoint.
				const float3& currWaypoint = gmtCollider->GetCurrWayPoint();
				const float collideeToCurrDistSq = currWaypoint.SqDistance2D(collidee->pos);
				const float collideeGoalRadius = gmtCollidee->GetOwnerRadius();
				if (collideeToCurrDistSq <= Square((collideeGoalRadius+separationDist)*adjustForDiagonal)) {
					gmtCollider->TriggerSkipWayPoint();
					return;
				}
			}

			// if (!gmtCollider->IsAtGoalPos(collider->pos, gmtCollider->GetOwnerRadius()))
			// 	return;

			// gmtCollider->TriggerCallArrived();
		} break;

		default: {
		} break;
	}
}


static float3 CalcSpeedVectorInclGravity(const CUnit* owner, const CGroundMoveType* mt, float hAcc, float vAcc) {
	RECOIL_DETAILED_TRACY_ZONE;
	float3 newSpeedVector;

	// NOTE:
	//   the drag terms ensure speed-vector always decays if
	//   wantedSpeed and deltaSpeed are 0 (needed because we
	//   do not call GetDragAccelerationVect while a unit is
	//   moving under its own power)
	const float dragCoeff = mix(0.99f, 0.9999f, owner->IsInAir());
	const float slipCoeff = mix(0.95f, 0.9999f, owner->IsInAir());

	const float3& ownerPos = owner->pos;
	const float3& ownerSpd = owner->speed;

	// use terrain-tangent vector because it does not
	// depend on UnitDef::upright (unlike o->frontdir)
	const float3& gndNormVec = mt->GetGroundNormal(ownerPos);
	const float3  gndTangVec = gndNormVec.cross(owner->rightdir);
	const float3& flatFrontDir = mt->GetFlatFrontDir();

	const int dirSign = Sign(flatFrontDir.dot(ownerSpd));
	const int revSign = Sign(int(!mt->IsReversing()));

	const float3 horSpeed = ownerSpd * XZVector * dirSign * revSign;
	const float3 verSpeed = UpVector * ownerSpd.y;

	if (!modInfo.allowHoverUnitStrafing || owner->moveDef->speedModClass != MoveDef::Hover) {
		const float3 accelVec = (gndTangVec * hAcc) + (UpVector * vAcc);
		const float3 speedVec = (horSpeed + verSpeed) + accelVec;

		newSpeedVector += (flatFrontDir * speedVec.dot(flatFrontDir)) * dragCoeff;
		newSpeedVector += (    UpVector * speedVec.dot(    UpVector));
	} else {
		// TODO: also apply to non-hovercraft on low-gravity maps?
		newSpeedVector += (              gndTangVec * (  std::max(0.0f,   ownerSpd.dot(gndTangVec) + hAcc * 1.0f))) * dragCoeff;
		newSpeedVector += (   horSpeed - gndTangVec * (/*std::max(0.0f,*/ ownerSpd.dot(gndTangVec) - hAcc * 0.0f )) * slipCoeff;
		newSpeedVector += (UpVector * UpVector.dot(ownerSpd + UpVector * vAcc));
	}

	// never drop below terrain while following tangent
	// (SPEED must be adjusted so that it does not keep
	// building up when the unit is on the ground or is
	// within one frame of hitting it)
	const float oldGroundHeight = mt->GetGroundHeight(ownerPos                 );
	const float newGroundHeight = mt->GetGroundHeight(ownerPos + newSpeedVector);

	if ((ownerPos.y + newSpeedVector.y) <= newGroundHeight)
		newSpeedVector.y = std::min(newGroundHeight - ownerPos.y, math::fabs(newGroundHeight - oldGroundHeight));

	return newSpeedVector;
}
// 根据单位当前的实际速度、最大速度限制以及本帧的加速度，计算出一个新的、符合物理规则的速度向量。
// 计算不考虑重力
// const CUnit* owner: 指向所属单位的指针。
// const CGroundMoveType* mt: 指向单位移动类型实例的指针。
// float hAcc: 本帧的水平加速度（deltaSpeed）。这是由 ChangeSpeed 函数计算得出的。
// float vAcc: 本帧的垂直加速度。对于地面单位，这个值通常是0，除非有特殊效果。
static float3 CalcSpeedVectorExclGravity(const CUnit* owner, const CGroundMoveType* mt, float hAcc, float vAcc) {
	RECOIL_DETAILED_TRACY_ZONE;
	// LuaSyncedCtrl::SetUnitVelocity directly assigns
	// to owner->speed which gets overridden below, so
	// need to calculate hSpeedScale from it (not from
	// currentSpeed) directly
	// 这段注释非常关键。它解释了为什么函数直接使用 owner->speed 而不是 moveType->currentSpeed。
	// 因为 Lua 脚本可以通过 SetUnitVelocity 直接修改单位的 owner->speed 向量。
	// 如果我们使用 currentSpeed（它只在 ChangeSpeed 中更新），就会忽略掉 Lua 脚本施加的速度变化。
	// 所以，为了正确处理来自脚本的外部速度干预，必须直接读取 owner->speed。


	// 这是一个提前停止的快捷路径。
	// mt->GetWantedSpeed() == 0.f: 检查上层逻辑是否希望单位停止
	// math::fabs(owner->speed.w) - hAcc <= 0.01f: 
	// 检查单位当前的速率（owner->speed.w 是速度向量的长度）是否已经非常小，并且本帧的加速度 hAcc 也无法使其显著增加。
	if (mt->GetWantedSpeed() == 0.f && math::fabs(owner->speed.w) - hAcc <= 0.01f)
		return ZeroVector;
	else {
		float vel = owner->speed.w; // 获取单位当前的速率（标量）。
		float maxSpeed = owner->moveType->GetMaxSpeed(); //  获取单位定义中的最大允许速度。
		// 这个 if 块处理一种特殊情况：单位当前的速率超过了它自身引擎能达到的最大速度。
		// 这种情况通常是由于外力（如爆炸冲击波、Lua脚本设置）造成的。
		if (vel > maxSpeed) { 
			// Once a unit is travelling faster than their maximum speed, their engine power is no longer sufficient to counteract
			// the drag from air and rolling resistance. So reduce their velocity by these forces until a return to maximum speed.
			// 注释: 解释了接下来的逻辑：当单位超速时，它的引擎动力不足以维持这个速度，
			// 所以我们需要模拟空气阻力和滚动摩擦力，使其速度自然衰减，直到回到其最大速度。
			float rollingResistanceCoeff = owner->unitDef->rollingResistanceCoefficient;
			// 这是核心的速度衰减计算。
			// owner->GetDragAccelerationVec(...): 这个函数计算出由于空气阻力和滚动摩擦力产生的负加速度向量（一个指向速度反方向的向量）。
			// owner->speed + ...: 将这个负加速度应用到当前的速度向量上，得到一个衰减后的新速度向量。
			// .Length(): 计算这个新速度向量的长度（速率）。
			// std::max(maxSpeed, ...): 确保衰减后的速度不会低于单位自身的最大速度 maxSpeed。
			// 这样可以防止单位因为摩擦力而减速到比自身引擎能维持的速度还慢。
			vel = std::max(maxSpeed,
				(owner->speed +
				owner->GetDragAccelerationVec(
					mapInfo->atmosphere.fluidDensity,
					mapInfo->water.fluidDensity,
					owner->unitDef->atmosphericDragCoefficient,
					rollingResistanceCoeff
				)).Length()
			);
		}
		// return (...): 这是函数的最终返回。
		// vel * Sign(int(!mt->IsReversing())):
		//    vel 是我们刚刚计算出的、可能经过超速衰减修正后的速率。
		//    Sign(...) 返回 +1（前进时）或 -1（倒车时）。
		//    这一部分计算出了单位在施加本帧加速度之前的“基础速度”标量。
		// ... + hAcc: 将本帧的加速度 hAcc（由 ChangeSpeed 计算得出）加到基础速度上。
		// 将最终计算出的速度标量，乘以单位当前的前方向量 owner->frontdir，得到最终的新速度向量。这个向量将被 UpdateOwnerPos 用来更新单位的位置。
		return (owner->frontdir * (vel * Sign(int(!mt->IsReversing())) + hAcc));
	}
}



static constexpr decltype(&CheckCollisionInclSAT) checkCollisionFuncs[] = {
	CheckCollisionExclSAT,
	CheckCollisionInclSAT,
};

static constexpr decltype(&CalcSpeedVectorInclGravity) calcSpeedVectorFuncs[] = {
	CalcSpeedVectorExclGravity,
	CalcSpeedVectorInclGravity,
};




CGroundMoveType::CGroundMoveType(CUnit* owner):
	AMoveType(owner),
	pathController(owner),

	currWayPoint(ZeroVector),
	nextWayPoint(ZeroVector),

	flatFrontDir(FwdVector),
	lastAvoidanceDir(ZeroVector),
	mainHeadingPos(ZeroVector),
	skidRotVector(UpVector),

	wantedHeading(0),
	minScriptChangeHeading((SPRING_CIRCLE_DIVS - 1) >> 1),

	pushResistant((owner != nullptr) && owner->unitDef->pushResistant),
	canReverse((owner != nullptr) && (owner->unitDef->rSpeed > 0.0f))
{
	// creg
	if (owner == nullptr)
		return;

	const UnitDef* ud = owner->unitDef;
	const MoveDef* md = owner->moveDef;

	assert(ud != nullptr);
	assert(md != nullptr);

	// maxSpeed is set in AMoveType's ctor
	maxReverseSpeed = ud->rSpeed / GAME_SPEED;

	// SPRING_CIRCLE_DIVS is 65536, but turnRate can be at most
	// 32767 since it is converted to (signed) shorts in places
	turnRate = std::clamp(ud->turnRate, 1.0f, SPRING_CIRCLE_DIVS * 0.5f - 1.0f);
	turnAccel = turnRate * mix(0.333f, 0.033f, md->speedModClass == MoveDef::Ship);

	accRate = std::max(0.01f, ud->maxAcc);
	decRate = std::max(0.01f, ud->maxDec);

	// unit-gravity must always be negative
	myGravity = mix(-math::fabs(ud->myGravity), mapInfo->map.gravity, ud->myGravity == 0.0f);

	ownerRadius = md->CalcFootPrintMinExteriorRadius();

	forceStaticObjectCheck = true;

	flatFrontDir = (owner->frontdir * XZVector).Normalize();

	// Override the unit size, it should match the MoveDef's to avoid conflicts elsewhere in the code.
	owner->xsize = md->xsize;
	owner->zsize = md->zsize;

	Connect();
}

CGroundMoveType::~CGroundMoveType()
{
	Disconnect();

	if (nextPathId != 0) {
		pathManager->DeletePath(nextPathId, true);
	}

	if (pathID != 0) {
		pathManager->DeletePath(pathID, true);
	}

	if (deletePathId != 0) {
		pathManager->DeletePath(deletePathId);
	}
}

void CGroundMoveType::PostLoad()
{
	RECOIL_DETAILED_TRACY_ZONE;
	pathController = GMTDefaultPathController(owner);

	// If the active moveType is not set to default ground move (i.e. is on scripted move type) then skip.
	if ((uint8_t *)owner->moveType != owner->amtMemBuffer) {
		// Safety measure to clear the path id.
		pathID = 0;
		return;
	}

	Connect();

	// HACK: re-initialize path after load
	if (pathID == 0)
		return;

	// There isn't a path to clear (we've just loaded a saved game), so we must now clear pathID
	// before requesting our new path; otherwise, a valid path for another unit could be deleted.
	pathID = 0;
	pathID = pathManager->RequestPath(owner, owner->moveDef, owner->pos, goalPos, goalRadius + extraRadius, true);
}

bool CGroundMoveType::OwnerMoved(const short oldHeading, const float3& posDif, const float3& cmpEps) {
	RECOIL_DETAILED_TRACY_ZONE;
	if (posDif.equals(ZeroVector, cmpEps)) {
		// note: the float3::== test is not exact, so even if this
		// evaluates to true the unit might still have an epsilon
		// speed vector --> nullify it to prevent apparent visual
		// micro-stuttering (speed is used to extrapolate drawPos)
		owner->SetVelocityAndSpeed(ZeroVector);

		// negative y-coordinates indicate temporary waypoints that
		// only exist while we are still waiting for the pathfinder
		// (so we want to avoid being considered "idle", since that
		// will cause our path to be re-requested and again give us
		// a temporary waypoint, etc.)
		// NOTE: this is only relevant for QTPFS (at present)
		// if the unit is just turning in-place over several frames
		// (eg. to maneuver around an obstacle), do not consider it
		// as "idling"
		idling = true;
		idling &= !atGoal;
		idling &= (std::abs(owner->heading - oldHeading) < turnRate);

		return false;
	}

	// note: HandleObjectCollisions() may have negated the position set
	// by UpdateOwnerPos() (so that owner->pos is again equal to oldPos)
	// note: the idling-check can only succeed if we are oriented in the
	// direction of our waypoint, which compensates for the fact distance
	// decreases much less quickly when moving orthogonal to <waypointDir>
	oldPos = owner->pos;

	const float3 ffd = flatFrontDir * posDif.SqLength() * 0.5f;
	const float3 wpd = waypointDir * ((int(!reversing) * 2) - 1);

	// too many false negatives: speed is unreliable if stuck behind an obstacle
	//   idling = (Square(owner->speed.w) < (accRate * accRate));
	//   idling &= (Square(currWayPointDist - prevWayPointDist) <= (accRate * accRate));
	// too many false positives: waypoint-distance delta and speed vary too much
	//   idling = (Square(currWayPointDist - prevWayPointDist) < Square(owner->speed.w));
	// too many false positives: many slow units cannot even manage 1 elmo/frame
	//   idling = (Square(currWayPointDist - prevWayPointDist) < 1.0f);
	idling = true;
	idling &= (math::fabs(posDif.y) < math::fabs(cmpEps.y * owner->pos.y));
	idling &= (Square(currWayPointDist - prevWayPointDist) < ffd.dot(wpd));
	idling &= (posDif.SqLength() < Square(owner->speed.w * 0.5f));

	return true;
}

void CGroundMoveType::UpdatePreCollisions()
{
	RECOIL_DETAILED_TRACY_ZONE;
 	ASSERT_SYNCED(owner->pos);
	ASSERT_SYNCED(owner->heading);
 	ASSERT_SYNCED(currWayPoint);
 	ASSERT_SYNCED(nextWayPoint);

	if (resultantForces.SqLength() > 0.f)
		owner->Move(resultantForces, true);

	SyncWaypoints();

	// The mt section may have noticed the new path was ready and switched over to it. If so then
	// delete the old path, which has to be done in an ST section.
	if (deletePathId != 0) {
		pathManager->DeletePath(deletePathId);
		deletePathId = 0;
	}

	if (pathingArrived) {
		Arrived(false);
		pathingArrived = false;
	}

 	if (pathingFailed) {
 		Fail(false);
 		pathingFailed = false;
 	}

	pathManager->UpdatePath(owner, pathID);

	if (owner->GetTransporter() != nullptr) return;

	owner->UpdatePhysicalStateBit(CSolidObject::PSTATE_BIT_SKIDDING, owner->IsSkidding() || OnSlope(1.0f));

	if (owner->IsSkidding()) {
		UpdateSkid();
		return;
	}

	// set drop height when we start to drop
	if (owner->IsFalling()) {
		UpdateControlledDrop();
		return;
	}

	reversing = UpdateOwnerSpeed(math::fabs(oldSpeed), math::fabs(newSpeed), newSpeed);
	oldSpeed = newSpeed = 0.f;
}

void CGroundMoveType::UpdateUnitPosition() {
	// 初始化成员变量 resultantForces（合力）为零向量。这个变量用于累加本帧所有使单位移动的力/位移，
	// 包括引擎自身的推力、来自碰撞的推力等。在函数开始时清零，确保每次计算都是从一个干净的状态开始。
	resultantForces = ZeroVector;
	// 这是一个守护条件。如果单位当前正处于“打滑”状态（例如，被爆炸击中后不受控制地滑动），
	// 它的移动完全由物理模拟（在 UpdateSkid 函数中处理）决定。因此，这里直接 return，跳过所有常规的、受控的移动逻辑。
	if (owner->IsSkidding()) return;

 	switch (setHeading) {
		// 当 FollowPath 函数确定单位需要沿着路径移动时，它会将 setHeading 设置为此值。
 		case HEADING_CHANGED_MOVE:
 			// ChangeHeading(setHeadingDir);
			ChangeSpeed(maxWantedSpeed, WantReverse(waypointDir, flatFrontDir));
 			setHeading = HEADING_CHANGED_NONE; // 将状态标志重置为“无变化”，防止在下一帧重复执行此逻辑。
 			break;
		// 当 FollowPath 发现单位已经到达目标或没有路径可走时，会设置此状态
		case HEADING_CHANGED_STOP:
	// 		SetMainHeading();
			ChangeSpeed(0.0f, false);
			setHeading = HEADING_CHANGED_NONE;
			break;
		// 当 UpdateOwnerAccelAndHeading 检测到单位被眩晕或正在建造时，会设置此状态。
		case HEADING_CHANGED_STUN:
			ChangeSpeed(0.0f, false);
			setHeading = HEADING_CHANGED_NONE;
			break;
	}

	if (owner->GetTransporter() != nullptr) return;

 	if (owner->UnderFirstPersonControl())
 		UpdateDirectControl();

	UpdateOwnerPos(owner->speed, calcSpeedVectorFuncs[modInfo.allowGroundUnitGravity](owner, this, deltaSpeed, myGravity));
}

void CGroundMoveType::UpdateCollisionDetections() {
	RECOIL_DETAILED_TRACY_ZONE;

	if (owner->GetTransporter() != nullptr) return;
	if (owner->IsSkidding()) return;
	if (owner->IsFalling()) return;

	HandleObjectCollisions();
}

bool CGroundMoveType::Update()
{
	RECOIL_DETAILED_TRACY_ZONE;
	if (owner->requestRemoveUnloadTransportId) {
		owner->unloadingTransportId = -1;
		owner->requestRemoveUnloadTransportId = false;
	}

	// Collisions can change waypoint states (y); though they won't move them (xz).
	SyncWaypoints();

	// do nothing at all if we are inside a transport
	if (owner->GetTransporter() != nullptr) return false;
	if (owner->IsSkidding()) return false;
	if (owner->IsFalling()) return false;

	if (resultantForces.SqLength() > 0.f)
		owner->Move(resultantForces, true);

	AdjustPosToWaterLine();

	ASSERT_SANE_OWNER_SPEED(owner->speed);

	// <dif> is normally equal to owner->speed (if no collisions)
	// we need more precision (less tolerance) in the y-dimension
	// for all-terrain units that are slowed down a lot on cliffs
	return (OwnerMoved(owner->heading, owner->pos - oldPos, float3(float3::cmp_eps(), float3::cmp_eps() * 1e-2f, float3::cmp_eps())));
}

void CGroundMoveType::UpdateOwnerAccelAndHeading()
{
	RECOIL_DETAILED_TRACY_ZONE;
	// 检查单位是否处于“眩晕”状态。如果单位被EMP武器击中，它会暂时失能，无法移动或执行任何动作。
	// 检查单位是否仍处于“建造中”的状态。建造中的单位（纳米框架）是固定的，不能移动。
	if (owner->IsStunned() || owner->beingBuilt) {
		setHeading = HEADING_CHANGED_STUN;
		return;
	}

	if (owner->UnderFirstPersonControl()) return;

	// either follow user control input or pathfinder
	// waypoints; change speed and heading as required
	// if (owner->UnderFirstPersonControl()) {
	// 	UpdateDirectControl();
	// } else {
		FollowPath(ThreadPool::GetThreadNum());
	// }
}

void CGroundMoveType::SlowUpdate()
{
	RECOIL_DETAILED_TRACY_ZONE;

	// bool printMoveInfo = (selectedUnitsHandler.selectedUnits.size() == 1)
	// 	&& (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end());
	// if (printMoveInfo) {
	// 	LOG("%s: unit selected=%d pathType=%d atEndOfPath=%d atGoal=%d currWayPoint=(%f,%f,%f) nextWayPoint=(%f,%f,%f) goal=(%f,%f,%f) pos=(%f,%f,%f)"
	// 			, __func__
	// 			, owner->id, owner->moveDef->pathType, int(atEndOfPath), int(atGoal)
	// 			, float(currWayPoint.x), float(currWayPoint.y), float(currWayPoint.z)
	// 			, float(nextWayPoint.x), float(nextWayPoint.y), float(nextWayPoint.z)
	// 			, goalPos.x, goalPos.y, goalPos.z
	// 			, owner->pos.x, owner->pos.y, owner->pos.z);
	// }

	if (owner->GetTransporter() != nullptr) {
		if (progressState == Active)
			StopEngine(false);

	} else {
		if (progressState == Active) {
			if (pathID != 0) {
				if (idling) {
					numIdlingSlowUpdates = std::min(MAX_IDLING_SLOWUPDATES, int(numIdlingSlowUpdates + 1));
				} else {
					numIdlingSlowUpdates = std::max(0, int(numIdlingSlowUpdates - 1));
				}

				if (numIdlingUpdates > (SPRING_MAX_HEADING / turnRate)) {
					// case A: we have a path but are not moving
					LOG_L(L_DEBUG, "[%s] unit %i has pathID %i but %i ETA failures", __func__, owner->id, pathID, numIdlingUpdates);

					if (numIdlingSlowUpdates < MAX_IDLING_SLOWUPDATES) {
						// avoid spamming rerequest paths if the unit is making progress.
						if (idling) {
							// Unit may have got stuck in
							// 1) a wreck that has spawned
							// 2) a push-resistant unit that stopped moving
							// 3) an amphibious unit has just emerged from water right underneath a structure.
							// 4) a push-resistant unit/building was spawned or teleported on top of us via lua
							// 5) a stopped unit we were inside of was newly made push-resistant via lua
							// 6) our movedef changed into one of a different size / crushStrength via lua
							forceStaticObjectCheck = true;
							ReRequestPath(true);
						}
					} else {
						// unit probably ended up on a non-traversable
						// square, or got stuck in a non-moving crowd
						Fail(false);
						// bool printMoveInfo = (selectedUnitsHandler.selectedUnits.size() == 1)
						// 	&& (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end());
						// if (printMoveInfo) {
						// 	LOG("%s: failed by idling too long.", __func__);
						// }
					}
				}
			} else {
				// case B: we want to be moving but don't have a path
				LOG_L(L_DEBUG, "[%s] unit %i has no path", __func__, owner->id);
				ReRequestPath(true);
			}

			if (wantRepath) {
				// When repaths are requested, they are pre-emptive and are made without
				// confirmation that it is really necessary. Give the unit a chance to
				// make progress: for example, when it got pushed against a building, but
				// is otherwise moving on. Pathing is expensive so we really want to keep
				// repathing to a minimum.
				// Resolution distance checks kept to 1/10th of an Elmo to reduce the
				// amount of time a unit can spend making insignificant progress, every
				// SlowUpdate.
				float curDist = math::floorf(currWayPoint.distance2D(owner->pos) * 10.f) / 10.f;
				if (curDist < bestLastWaypointDist) {
					bestLastWaypointDist = curDist;
					wantRepathFrame = gs->frameNum;
				}

				// lastWaypoint typically retries a repath and most likely won't get closer, so
				// in this case, don't wait around making the unit try to run into an obstacle for
				// longer than absolutely necessary.
				bool timeForRepath = gs->frameNum >= wantRepathFrame + modInfo.pfRepathDelayInFrames
									&& (gs->frameNum >= lastRepathFrame + modInfo.pfRepathMaxRateInFrames || lastWaypoint);

				// can't request a new path while the unit is stuck in terrain/static objects
				if (timeForRepath){
					if (lastWaypoint) {
						bestLastWaypointDist *= (1.f / SQUARE_SIZE);
						if (bestLastWaypointDist < bestReattemptedLastWaypointDist) {
							// Give a bad path another try, in case we can get closer.
							lastWaypoint = false;
							bestReattemptedLastWaypointDist = bestLastWaypointDist;
						}
						else {
							bestReattemptedLastWaypointDist = std::numeric_limits<decltype(bestReattemptedLastWaypointDist)>::infinity();
						}
					}
					if (!lastWaypoint) {
						ReRequestPath(true);
					} else {
						// This is here to stop units from trying forever to reach an goal they can't reach.
						Fail(false);
						// bool printMoveInfo = (selectedUnitsHandler.selectedUnits.size() == 1)
						// 	&& (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end());
						// if (printMoveInfo) {
							// LOG("%s: failed to reach final waypoint", __func__);
						// }
					}
				}
			}
		}

		// move us into the map, and update <oldPos>
		// to prevent any extreme changes in <speed>
		if (!owner->IsFlying() && !owner->pos.IsInBounds())
			owner->Move(oldPos = owner->pos.cClampInBounds(), false);
	}

	AMoveType::SlowUpdate();
}


void CGroundMoveType::StartMovingRaw(const float3 moveGoalPos, float moveGoalRadius) {
	RECOIL_DETAILED_TRACY_ZONE;
	const float deltaRadius = std::max(0.0f, ownerRadius - moveGoalRadius);

	#ifdef PATHING_DEBUG
	if (DEBUG_DRAWING_ENABLED) {
		bool printMoveInfo = (selectedUnitsHandler.selectedUnits.size() == 1)
			&& (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end());
		if (printMoveInfo) {
			LOG("%s unit goal (%f,%f,%f) [%f]", __func__
				, static_cast<float>(moveGoalPos.x)
				, static_cast<float>(moveGoalPos.y)
				, static_cast<float>(moveGoalPos.z)
				, moveGoalRadius);
		}
	}
	#endif

	goalPos = moveGoalPos * XZVector;
	goalRadius = moveGoalRadius;

	MoveTypes::CheckCollisionQuery collisionQuery(owner->moveDef, moveGoalPos);
	extraRadius = deltaRadius * (1 - owner->moveDef->TestMoveSquare(collisionQuery, moveGoalPos, ZeroVector, true, true));

	earlyCurrWayPoint = currWayPoint = goalPos;
	earlyNextWayPoint = nextWayPoint = goalPos;

	atGoal = (moveGoalPos.SqDistance2D(owner->pos) < Square(goalRadius + extraRadius));
	atEndOfPath = false;
	lastWaypoint = false;

	useMainHeading = false;
	useRawMovement = true;

	progressState = Active;

	numIdlingUpdates = 0;
	numIdlingSlowUpdates = 0;

	currWayPointDist = 0.0f;
	prevWayPointDist = 0.0f;

	pathingArrived = false;
	pathingFailed = false;
}

void CGroundMoveType::StartMoving(float3 moveGoalPos, float moveGoalRadius) {
	RECOIL_DETAILED_TRACY_ZONE;
	// add the footprint radius if moving onto goalPos would cause it to overlap impassable squares
	// (otherwise repeated coldet push-jittering can ensue if allowTerrainCollision is not disabled)
	// not needed if goalRadius actually exceeds ownerRadius, e.g. for builders
	const float deltaRadius = std::max(0.0f, ownerRadius - moveGoalRadius);

	#ifdef PATHING_DEBUG
	if (DEBUG_DRAWING_ENABLED) {
		bool printMoveInfo = (selectedUnitsHandler.selectedUnits.size() == 1)
			&& (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end());
		if (printMoveInfo) {
			LOG("%s unit goal (%f,%f,%f) [%f]", __func__
				, static_cast<float>(moveGoalPos.x)
				, static_cast<float>(moveGoalPos.y)
				, static_cast<float>(moveGoalPos.z)
				, moveGoalRadius);
		}
	}
	#endif

	// set the new goal
	goalPos = moveGoalPos * XZVector;

	float mapx = mapDims.mapxm1 * SQUARE_SIZE;
	float mapz = mapDims.mapym1 * SQUARE_SIZE;

	// Sanitize the move command.
	if (goalPos.x < 0.f)  { goalPos.x = 0.f; }
	if (goalPos.z < 0.f)  { goalPos.z = 0.f; }
	if (goalPos.x > mapx) { goalPos.x = mapx; }
	if (goalPos.z > mapz) { goalPos.z = mapz; }

	goalRadius = moveGoalRadius;
	MoveTypes::CheckCollisionQuery collisionQuery(owner->moveDef, goalPos);
	extraRadius = deltaRadius * (1 - owner->moveDef->TestMoveSquare(collisionQuery, goalPos, ZeroVector, true, true));

	atGoal = (goalPos.SqDistance2D(owner->pos) < Square(goalRadius + extraRadius));
	atEndOfPath = false;
	lastWaypoint = false;

	useMainHeading = false;
	useRawMovement = false;

	progressState = Active;

	numIdlingUpdates = 0;
	numIdlingSlowUpdates = 0;

	currWayPointDist = 0.0f;
	prevWayPointDist = 0.0f;

	pathingArrived = false;
	pathingFailed = false;

	LOG_L(L_DEBUG, "[%s] starting engine for unit %i", __func__, owner->id);

	if (atGoal)
		return;

	// silently free previous path if unit already had one
	//
	// units passing intermediate waypoints will TYPICALLY not cause any
	// script->{Start,Stop}Moving calls now (even when turnInPlace=true)
	// unless they come to a full stop first
	ReRequestPath(true);

	bestReattemptedLastWaypointDist = std::numeric_limits<decltype(bestReattemptedLastWaypointDist)>::infinity();

	if (owner->team == gu->myTeam)
		Channels::General->PlayRandomSample(owner->unitDef->sounds.activate, owner);
}

void CGroundMoveType::StopMoving(bool callScript, bool hardStop, bool cancelRaw) {
	RECOIL_DETAILED_TRACY_ZONE;
	LOG_L(L_DEBUG, "[%s] stopping engine for unit %i", __func__, owner->id);

	if (!atGoal)
		earlyCurrWayPoint = (goalPos = (currWayPoint = Here()));

	// this gets called under a variety of conditions (see MobileCAI)
	// the most common case is a CMD_STOP being issued which means no
	// StartMoving-->StartEngine will follow
	StopEngine(callScript, hardStop);

	// force goal to be set to prevent obscure conditions triggering a
	// stationary unit to try moving due to obstacle collisions.
	atGoal = true;

	// force WantToStop to return true when useRawMovement is enabled
	atEndOfPath |= useRawMovement;
	// only a new StartMoving call can normally reset this
	useRawMovement &= (!cancelRaw);
	useMainHeading = false;

	progressState = Done;
}

void CGroundMoveType::UpdateTraversalPlan() {
	RECOIL_DETAILED_TRACY_ZONE;

	earlyCurrWayPoint = currWayPoint;
	earlyNextWayPoint = nextWayPoint;

	// Check whether the new path is ready.
	// 如何下一个路径准备好了
	if (nextPathId != 0) {
		// 找到下一个路径点 如果没有路径点 返回(-1, -1, -1)
		float3 tempWaypoint = pathManager->NextWayPoint(owner, nextPathId, 0,   owner->pos, std::max(WAYPOINT_RADIUS, currentSpeed * 1.05f), true);

		// a non-temp answer tells us that the new path is ready to be used.
		// 如果返回的路径点合法
		if (tempWaypoint.y != (-1.f)) {
			// if the unit has switched to a raw move since the new path was requested then don't
			// try to redirect onto the new path.
			// 如果单位在请求新路径之后，已经切换到了“原始移动”(raw move)模式，那么就不要尝试将其重定向到这条新路径上。
			if (!useRawMovement) {
				// switch straight over to the new path
				earlyCurrWayPoint = tempWaypoint;
				earlyNextWayPoint = pathManager->NextWayPoint(owner, nextPathId, 0, earlyCurrWayPoint, std::max(WAYPOINT_RADIUS, currentSpeed * 1.05f), true);
				lastWaypoint = false;
				wantRepath = false;
				atEndOfPath = false;
			}

			// can't delete the path in an MT section
			deletePathId = pathID;
			pathID = nextPathId;
			nextPathId = 0;
		}
	}

	if (owner->GetTransporter() != nullptr) return;
	if (owner->IsSkidding()) return;
	if (owner->IsFalling()) return;

	UpdateObstacleAvoidance();
	UpdateOwnerAccelAndHeading();
}

void CGroundMoveType::UpdateObstacleAvoidance() {
	// 如果单位被眩晕或正在建造中，则不进行任何操作
	if (owner->IsStunned() || owner->beingBuilt)
		return;

	// 如果单位正处于玩家的第一人称控制下，则不进行任何操作
	if (owner->UnderFirstPersonControl())
		return;
	// 获取单位在水平面上的前向向量
	const float3&  ffd = flatFrontDir;
	// 判断单位是否需要倒车
	auto wantReverse = WantReverse(waypointDir, ffd);
	// 计算原始的期望方向（朝向路径点，考虑倒车）
	const float3  rawWantedDir = waypointDir * Sign(int(!wantReverse));
	// 调用核心避障函数，并混合方向
	GetObstacleAvoidanceDir(mix(ffd, rawWantedDir, !atGoal));
}

// int thread - 当前执行此代码的线程ID。这在多线程寻路请求中很有用。
// 返回值 bool - 函数返回一个布尔值，表示单位在当前帧是否想要倒车 (true 表示要倒车)。
bool CGroundMoveType::FollowPath(int thread) 
{
	RECOIL_DETAILED_TRACY_ZONE;
	// 初始化一个局部变量 wantReverse 为 false。这个变量将在函数末尾作为返回值，并在此过程中被更新。
	bool wantReverse = false;
	// 检查单位是否应该停止移动。WantToStop() 在单位没有路径 (pathID == 0) 且处于“原始移动”模式且已到达路径终点时返回 true
	if (WantToStop()) {// 处理停止移动的情况
		// 如果要停止，就将当前和下一个路径点的 y 坐标设置为 -1.0f。
		// 在引擎的其他部分，y == -1.0f 是一个特殊的标记，表示这是一个无效或临时的路径点
		earlyCurrWayPoint.y = -1.0f;
		earlyNextWayPoint.y = -1.0f;
		// 设置状态标志 setHeading。这个标志会在稍后的 UpdateUnitPosition 函数中被识别，并触发单位的减速逻辑，使其速度降为零。
		setHeading = HEADING_CHANGED_STOP;
		// 事件标记为已发生。
		// 另一个系统会监听这个事件，并执行相应的逻辑，比如让单位朝向其“主航向”（mainHeading）目标。
		auto& event = Sim::registry.get<ChangeMainHeadingEvent>(owner->entityReference);
		event.changed = true;
	} else { // 核心路径跟随逻辑
		#ifdef PATHING_DEBUG
		if (DEBUG_DRAWING_ENABLED) {
			bool printMoveInfo = (selectedUnitsHandler.selectedUnits.size() == 1)
				&& (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end());
			if (printMoveInfo) {
				LOG("%s unit origin (%f,%f,%f)", __func__
					, static_cast<float>(owner->pos.x)
					, static_cast<float>(owner->pos.y)
					, static_cast<float>(owner->pos.z));

				LOG("%s currWayPoint (%f,%f,%f)", __func__
					, static_cast<float>(currWayPoint.x)
					, static_cast<float>(currWayPoint.y)
					, static_cast<float>(currWayPoint.z));

				LOG("%s nextWayPoint (%f,%f,%f)", __func__
					, static_cast<float>(nextWayPoint.x)
					, static_cast<float>(nextWayPoint.y)
					, static_cast<float>(nextWayPoint.z));
			}
		}
		#endif
		// (单位当前位置)
		const float3& opos = owner->pos;
		// (单位当前速度)
		const float3& ovel = owner->speed;
		// (单位水平前方向量)
		const float3&  ffd = flatFrontDir;
		// (当前目标路径点)
		const float3&  cwp = earlyCurrWayPoint;
		// 将上一帧计算的到路径点的距离保存到 prevWayPointDist
		prevWayPointDist = currWayPointDist;
		// 计算单位当前位置与当前路径点在XZ平面上的新距离，并更新 currWayPointDist。
		currWayPointDist = earlyCurrWayPoint.distance2D(opos);

		{
			// NOTE:
			//   uses owner->pos instead of currWayPoint (ie. not the same as atEndOfPath)
			//
			//   if our first command is a build-order, then goal-radius is set to our build-range
			//   and we cannot increase tolerance safely (otherwise the unit might stop when still
			//   outside its range and fail to start construction)
			//
			//   units moving faster than <minGoalDist> elmos per frame might overshoot their goal
			//   the last two atGoal conditions will just cause flatFrontDir to be selected as the
			//   "wanted" direction when this happens
			// 注意：
			// 使用 owner->pos 而非 currWayPoint（即与 atEndOfPath 不同）
			// 如果我们的第一个指令是建造指令，那么目标半径会被设置为我们的建造范围
			// 且我们不能安全地增加公差（否则单位可能在仍处于范围外时停止移动，导致无法开始建造）
			// 移动速度超过每帧 <minGoalDist> 埃尔莫斯的单位可能会越过其目标
			// 当这种情况发生时，最后两个 atGoal 条件将仅导致 flatFrontDir 被选为 "期望" 方向

			// 计算单位当前位置与最终目标点 goalPos 距离的平方。
			const float curGoalDistSq = (opos - goalPos).SqLength2D();
			// 计算“最小目标距离”的平方。这是一个动态的容差范围。
			// 这是一个宏，检查单位当前的命令是否是移动或战斗命令。
			const float minGoalDistSq = (UNIT_HAS_MOVE_CMD(owner))?
				// 如果单位是移动命令，并且它似乎卡住了（numIdlingSlowUpdates > 0），那么就动态地放大这个容差范围。
				// 这是一种“解卡”机制：如果单位在目标附近徘徊但就是无法精确到达，就放宽标准让它认为已经到达，从而可以继续执行下一个命令。
				Square((goalRadius + extraRadius) * (numIdlingSlowUpdates + 1)):
				Square((goalRadius + extraRadius));
			// 计算与速度相关的目标距离平方。大致等于单位下一帧能移动的距离
			const float spdGoalDistSq = Square(currentSpeed * 1.05f);
			// 条件1 (主要条件): 如果单位与最终目标的距离小于（或等于）我们计算出的动态容差范围，就认为它到达了。
			// |= 是“或等于”，意味着只要 atGoal 变为 true，它就不会再变回 false。
			atGoal |= (curGoalDistSq <= minGoalDistSq);
			//  条件2 (过冲判断 - 前进时): 这个条件用于处理高速单位可能“冲过”目标点的情况。
			//  单位必须离目标很近。
			//  单位必须是正在前进, 
			//  目标点必须仍在单位的前方。, 
			//  在下一帧的预测位置 opos + ovel，目标点将会在单位的后方或正好在旁边
			atGoal |= ((curGoalDistSq <= spdGoalDistSq) && !reversing && (ffd.dot(goalPos - opos) > 0.0f && ffd.dot(goalPos - (opos + ovel)) <= 0.0f));
			// 条件3 (过冲判断 - 倒车时): 逻辑与条件2完全相同，只是方向相反，用于处理倒车时冲过目标点的情况。
			atGoal |= ((curGoalDistSq <= spdGoalDistSq) &&  reversing && (ffd.dot(goalPos - opos) < 0.0f && ffd.dot(goalPos - (opos + ovel)) >= 0.0f));
			// 如果单位到达了最终目标点 (atGoal)，那么它必然也到达了路径的终点 (atEndOfPath)。
			atEndOfPath |= atGoal;

			#ifdef PATHING_DEBUG
			if (DEBUG_DRAWING_ENABLED) {
				bool printMoveInfo = (selectedUnitsHandler.selectedUnits.size() == 1)
					&& (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end());
				if (printMoveInfo) {
					LOG("%s opos (%f,%f,%f)", __func__
						, static_cast<float>(opos.x)
						, static_cast<float>(opos.y)
						, static_cast<float>(opos.z));

					LOG("%s cwp (%f,%f,%f)", __func__
						, static_cast<float>(cwp.x)
						, static_cast<float>(cwp.y)
						, static_cast<float>(cwp.z));

					LOG("%s goalpos (%f,%f,%f)", __func__
						, static_cast<float>(goalPos.x)
						, static_cast<float>(goalPos.y)
						, static_cast<float>(goalPos.z));

					LOG("%s curGoalDistSq(%f) <= minGoalDistSq(%f)", __func__, curGoalDistSq, minGoalDistSq);
					LOG("%s (ffd.dot(goalPos - opos)(%f) ffd.dot(goalPos - (opos + ovel))(%f)", __func__, ffd.dot(goalPos - opos), ffd.dot(goalPos - (opos + ovel)));
					LOG("%s atGoal(%d?) reversing(%d?)", __func__, (int)atGoal, (int)reversing);
				}
			}
			#endif
		}

		if (!atGoal) { // 如果还没到终点
			// 如果单位正在移动 (idling 为 false)，就减少“空闲计数器
			numIdlingUpdates -= ((numIdlingUpdates >                  0) * (1 - idling));
			//  如果单位卡住了 (idling 为 true)，就增加“空闲计数器”。这个计数器在 SlowUpdate 中被用来判断是否需要重新寻路或宣告寻路失败
			numIdlingUpdates += ((numIdlingUpdates < SPRING_MAX_HEADING) *      idling );
		}

		// An updated path must be re-evaluated.
		//  这是一个特殊情况。atEndOfPath 可能因为到达了路径的最后一个路径点而为 true，但单位离真正的 goalPos 可能还有一段距离。
		if (atEndOfPath && !atGoal)
			// 在这种情况下，我们询问寻路管理器，当前的路径是否被更新过。如果路径没有被更新，atEndOfPath 保持 true；
			// 如果寻路器在我们行进过程中给了一条新路径，atEndOfPath 就变回 false，让单位继续沿新路径前进。
			atEndOfPath = !pathManager->PathUpdated(pathID);

		// atEndOfPath never becomes true when useRawMovement, except via StopMoving
		if (!atEndOfPath && !useRawMovement) {
			// 这个函数会检查单位是否已经足够接近当前的路径点，如果是，就从寻路管理器获取下一个路径点作为新的目标。这是路径跟随的核心驱动力。
			SetNextWayPoint(thread);
		} else {
			if (atGoal){ //  如果已经到达路径终点 (atEndOfPath 和 atGoal为 true)
				#ifdef PATHING_DEBUG
				if (DEBUG_DRAWING_ENABLED) {
					bool printMoveInfo = (selectedUnitsHandler.selectedUnits.size() == 1)
						&& (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end());
					if (printMoveInfo) {
						LOG("%s arrival signaled", __func__);
					}
				}
				#endif
				// 设置 pathingArrived 标志。这个标志会在下一轮更新的 UpdatePreCollisions 中被检测到，
				// 并触发 Arrived() 函数，正式宣告移动任务完成。
				pathingArrived = true;
			}
		}
		// 在请求路径点之后再设置方向；方向不应为空向量
		// 不要比较 y 分量，因为它们通常不同，只有 x 和 z 分量重要
		// set direction to waypoint AFTER requesting it; should not be a null-vector
		// do not compare y-components since these usually differ and only x&z matter

		// 计算出从当前位置 opos 指向当前路径点 cwp 的归一化方向向量，并存入 waypointDir 成员变量。
		SetWaypointDir(cwp, opos);

		//ASSERT_SYNCED(waypointVec);
		//ASSERT_SYNCED(waypointDir);
		// 调用 WantReverse 函数，这是一个复杂的启发式函数，它会比较“原地转身180度再前进”和“直接倒车”到达路径点的预估时间，
		// 来决定是否应该倒车。结果存入 wantRevers
		wantReverse = WantReverse(waypointDir, ffd);

		// apply obstacle avoidance (steering), prevent unit from chasing its own tail if already at goal
		// const float3  rawWantedDir = waypointDir * Sign(int(!wantReverse));
		// const float3& modWantedDir = GetObstacleAvoidanceDir(mix(ffd, rawWantedDir, !atGoal));

		// const float3& modWantedDir = GetObstacleAvoidanceDir(mix(ffd, rawWantedDir, (!atGoal) && (wpProjDists.x > wpProjDists.y || wpProjDists.z < 0.995f)));

		// 获取修正后的期望方向。注意，这里直接使用了 lastAvoidanceDir。
		// 这是因为真正的方向计算（包括调用 GetObstacleAvoidanceDir）已经在 UpdateTraversalPlan -> UpdateObstacleAvoidance 中提前完成了，
		// 并将结果存放在了 lastAvoidanceDir 中。这里只是直接读取那个已经计算好的结果
		const float3& modWantedDir = lastAvoidanceDir;

		// ChangeHeading(GetHeadingFromVector(modWantedDir.x, modWantedDir.z));
		// ChangeSpeed(maxWantedSpeed, wantReverse);
		//  设置状态标志，告诉后续流程，单位的朝向和速度是根据移动逻辑来改变的
		setHeading = HEADING_CHANGED_MOVE;
		auto& event = Sim::registry.get<ChangeHeadingEvent>(owner->entityReference);
		// 从最终的、经过避障修正的方向向量 
		event.deltaHeading = GetHeadingFromVector(modWantedDir.x, modWantedDir.z);
		event.changed = true;
		// setHeadingDir = GetHeadingFromVector(modWantedDir.x, modWantedDir.z);


		#ifdef PATHING_DEBUG
		if (DEBUG_DRAWING_ENABLED) {
			bool printMoveInfo = (selectedUnitsHandler.selectedUnits.size() == 1)
				&& (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end());
			if (printMoveInfo) {
				LOG("%s waypointDir (%f,%f,%f)", __func__
					, static_cast<float>(waypointDir.x)
					, static_cast<float>(waypointDir.y)
					, static_cast<float>(waypointDir.z));

				LOG("%s ffd (%f,%f,%f)", __func__
					, static_cast<float>(ffd.x)
					, static_cast<float>(ffd.y)
					, static_cast<float>(ffd.z));

				LOG("%s change heading (%f, %f) [%d]", __func__
						, modWantedDir.x, modWantedDir.z
						, GetHeadingFromVector(modWantedDir.x, modWantedDir.z));
				LOG("%s change speed (%f, %d?)", __func__, maxWantedSpeed, (int)wantReverse);
			}
		}
		#endif
	}

	//pathManager->UpdatePath(owner, pathID);
	return wantReverse;
}

void CGroundMoveType::SetWaypointDir(const float3& cwp, const float3 &opos) {
	RECOIL_DETAILED_TRACY_ZONE;
	if (!epscmp(cwp.x, opos.x, float3::cmp_eps()) || !epscmp(cwp.z, opos.z, float3::cmp_eps())) {
		float3 waypointVec = (cwp - opos) * XZVector;
		waypointDir = waypointVec / waypointVec.Length();
		// wpProjDists = {math::fabs(waypointVec.dot(ffd)), 1.0f, math::fabs(waypointDir.dot(ffd))};
	}
}

/**
 * float newWantedSpeed: 上层逻辑期望单位达到的速度。如果是 0.0f，表示希望单位停下。
 * bool wantReverse: 单位是否应该倒车。
 * bool fpsMode: 单位当前是否处于玩家第一人称直接控制模式。
 */
void CGroundMoveType::ChangeSpeed(float newWantedSpeed, bool wantReverse, bool fpsMode)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// shortcut to specify acceleration to bring to a stop.
	// 这是一个快捷路径 (shortcut)，用于处理“让一个已经几乎停止的单位保持停止”的情况。
	if ((wantedSpeed = newWantedSpeed) <= 0.0f && currentSpeed < 0.01f) {
		// 如果两个条件都满足，就直接设置加速度 deltaSpeed 为 -currentSpeed。
		// 这会精确地抵消掉单位当前任何微小的残留速度，使其完全停下。
		deltaSpeed = -currentSpeed;
		return;
	}

	// first calculate the "unrestricted" speed and acceleration
	// 初始化一个局部变量 targetSpeed。这是单位在理想情况下（没有地形、转弯等任何限制时）所能达到的最大速度。
	// 如果 wantReverse 为 false，targetSpeed 被设置为 maxSpeed (最大前进速度)。
	// 如果 wantReverse 为 true，targetSpeed 被设置为 maxReverseSpeed (最大倒车速度)。
	float targetSpeed = mix(maxSpeed, maxReverseSpeed, wantReverse);

	#if (WAIT_FOR_PATH == 1)
	// 在获得实际路径之前不要移动，试图隐藏排队延迟是非常危险的，
	// 因为单位可能会盲目地撞到物体、悬崖等（需要在 Update 中进行 QTPFS 空闲检查）
	// don't move until we have an actual path, trying to hide queuing
	// lag is too dangerous since units can blindly drive into objects,
	// cliffs, etc. (requires the QTPFS idle-check in Update)
	if (currWayPoint.y == -1.0f && nextWayPoint.y == -1.0f) {
		targetSpeed = 0.0f;
	} else
	#endif
	{
		if (wantedSpeed > 0.0f) {
			const UnitDef* ud = owner->unitDef;
			const MoveDef* md = owner->moveDef;

			// the pathfinders do NOT check the entire footprint to determine
			// passability wrt. terrain (only wrt. structures), so we look at
			// the center square ONLY for our current speedmod
			// 寻路器不会检查整个占地面积来判断
			// 相对于地形的可通行性（仅检查相对于建筑物的），因此我们仅查看
			// 中心方格来获取当前的速度系数

			// 计算单位当前所在位置的地形速度修正系数。这个系数由地形类型（如公路、泥地）、坡度等因素决定。
			// 例如，在公路上可能是 1.1 (加速)，在泥地里可能是 0.5 (减速)。
			float groundSpeedMod = CMoveMath::GetPosSpeedMod(*md, owner->pos, flatFrontDir);

			// the pathfinders don't check the speedmod of the square our unit is currently on
			// so if we got stuck on a nonpassable square and can't move try to see if we're
			// trying to release ourselves towards a passable square
			// 寻路器不会检查我们单位当前所在方格的速度系数
			// 因此，如果我们被困在一个不可通行的方格上且无法移动，尝试看看我们是否
			// 正试图朝着一个可通行的方格移动以摆脱困境 当前程序时Unit的正前方
			if (groundSpeedMod == 0.0f)
				groundSpeedMod = CMoveMath::GetPosSpeedMod(*md, owner->pos + flatFrontDir * SQUARE_SIZE, flatFrontDir);
			
			const float curGoalDistSq = (owner->pos - goalPos).SqLength2D();
			// 计算刹车距离的平方。BrakingDistance 是一个辅助函数，它根据当前速度 currentSpeed 和减速率 decRate 估算出单位完全停下来需要滑行的距离。
			const float minGoalDistSq = Square(BrakingDistance(currentSpeed, decRate));

			const float3& waypointDifFwd = waypointDir; // 路径点的前进方向
			const float3  waypointDifRev = -waypointDifFwd; // 路径点的后退方向
			// 根据是否倒车 来决定方向
			const float3& waypointDif = mix(waypointDifFwd, waypointDifRev, reversing);
			// 当前方向与路径点方向的差值
			const short turnDeltaHeading = owner->heading - GetHeadingFromVector(waypointDif.x, waypointDif.z);
			// 判断是否应该开始刹车
			// 只有当这是最后一个或唯一的命令时才刹车。如果后面还有其他移动命令，单位应该直接开往下一个目标，而不是在当前目标点停下。
			const bool startBraking = (UNIT_CMD_QUE_SIZE(owner) <= 1 && curGoalDistSq <= minGoalDistSq && !fpsMode);

			float maxSpeedToMakeTurn = std::numeric_limits<float>::infinity();

			if (!fpsMode && turnDeltaHeading != 0) { // 不是第一人称模式 且旋转差值不等于0
				// only auto-adjust speed for turns when not in FPS mode
				// 仅在非 FPS 模式下为转弯自动调整速度
				// 计算单位需要转动的总角度，单位是“度”
				const float reqTurnAngle = math::fabs(180.0f * short(owner->heading - wantedHeading) / SPRING_MAX_HEADING);
				// 计算单位在一帧内最多能转动多少度
				// turnRate: 单位的最大转向速率（以引擎内部单位/帧 表示）
				const float maxTurnAngle = (turnRate / SPRING_CIRCLE_DIVS) * 360.0f;
				// 获取当前状态下（前进或倒车）的理论最大速度
				const float turnMaxSpeed = mix(maxSpeed, maxReverseSpeed, reversing);
				// 初始化一个“转弯修正速度”变量，其初始值为理论最大速度。我们接下来要做的就是根据转弯的需要来降低这个值
				float turnModSpeed = turnMaxSpeed;

				if (reqTurnAngle != 0.0f)
					// axTurnAngle / reqTurnAngle: 计算“单帧最大转角”与“总共需要转的角度”的比率
					// 如果是急转弯 (reqTurnAngle 很大)，这个比率会很小
					// 如果是缓和的转弯 (reqTurnAngle 很小)，这个比率会很大
					// 将这个比率限制在 [0.1, 1.0] 的范围内。这意味着即使是再急的弯，速度最多也只会降到原来的10%。
					// 将这个计算出的比例乘到 turnModSpeed 上。转弯越急，这个比例越小，turnModSpeed 也就越低。
					turnModSpeed *= std::clamp(maxTurnAngle / reqTurnAngle, 0.1f, 1.0f);

				// 这段代码根据单位是否能“原地转向”来应用不同的减速逻辑。
				if (waypointDir.SqLength() > 0.1f) { // 这是一个安全检查，确保我们有一个合法的、非零的路径点方向向量。
					if (!ud->turnInPlace) { // 如果单位不能原地转向（像汽车一样，需要有前进速度才能转弯）
						// never let speed drop below TIPSL, but limit TIPSL itself to turnMaxSpeed
						// 将 targetSpeed 限制在一个范围内。它不能低于一个最小转弯速度 turnInPlaceSpeedLimit（TIPSL），
						// 也不能高于我们上面计算出的 turnModSpeed。这确保了单位既会为了转弯而减速，又不会因为减速太多而完全停下来导致无法转弯。
						targetSpeed = std::clamp(turnModSpeed, std::min(ud->turnInPlaceSpeedLimit, turnMaxSpeed), turnMaxSpeed);
					} else {
						// reqTurnAngle > ud->turnInPlaceAngleLimit: 检查需要转的角度是否大于一个预设的阈值。
						// 只有当转弯角度足够大时，才会将 targetSpeed 向 turnModSpeed 进行混合（即降低速度）。对于很小的角度调整，坦克不需要减速
						targetSpeed = mix(targetSpeed, turnModSpeed, reqTurnAngle > ud->turnInPlaceAngleLimit);
					}
				}
				// 这是一个特殊情况的处理。如果单位已经到达了路径的最后一个路径点。
				// 此时，单位必须强制减速，以确保能够精确地停在最终目标点，而不是因为速度过快而开始绕着目标点“转圈”。
				if (atEndOfPath) {
					// at this point, Update() will no longer call SetNextWayPoint()
					// and we must slow down to prevent entering an infinite circle
					// base ftt on maximum turning speed
					// 此时，Update () 将不再调用 SetNextWayPoint ()
					// 因此我们必须减速，以避免进入无限循环
					// 于最大转弯速度确定基础
					const float absTurnSpeed = turnRate;
					// 估算单位转动半圈（360度）大约需要多少帧
					const float framesToTurn = SPRING_CIRCLE_DIVS / absTurnSpeed;
					// (currWayPointDist * math::PI) / framesToTurn: 这是一个基于“匀速圆周运动”模型的估算。 这个公式需要仔细研究一下 
					// 它计算出一个速度值，如果单位以此速度前进，它刚好能在一个“转半圈”的时间内走完到目标点的距离。
					targetSpeed = std::min(targetSpeed, (currWayPointDist * math::PI) / framesToTurn);
				}
			}

			// When a waypoint is moved, it is because of a collision with a building. This can
			// result in the unit having to loop back around, but we need to make sure it doesn't
			// go too fast looping back around that it crashes right back into the same building
			// and trigger a looping behaviour.
			// 路径点发生移动，是因为它与建筑物发生了碰撞。这种情况可能会导致单位不得不掉头折返，
			// 但我们需要确保单位在折返时不会速度过快，以免再次撞上同一建筑物，从而触发循环往复的行为。

			// 这是一个入口条件。limitSpeedForTurning 是一个成员变量，在单位与静态物体发生碰撞时，它会被设置为一个大于0的数值（例如2）。
			// 它就像一个临时计时器或状态标志，告诉单位在接下来的几帧需要“特别小心地”转弯。每经过一个路径点，这个值会减1。
			if (limitSpeedForTurning > 0) {
				//  计算单位的移动方向符号（前进为+1，倒车为-1）。虽然在这个特定的代码块里 dirSign 没有被直接使用，但它通常用于需要区分前进和后退的计算中。
				const int dirSign = Sign(int(!reversing));
				short idealHeading = GetHeadingFromVector(waypointDir.x, waypointDir.z);
				// 计算出单位的当前航向与理想航向之间的角度差。这就是单位总共需要转动的角度。
				short offset = idealHeading - owner->heading;
				// 取单位的最大转向速率 turnRate 的绝对值。使用 std::max 确保这个值至少是一个很小的正数，以防止在下一步的除法中出现除以零的错误。
				const float absTurnSpeed = std::max(0.0001f, math::fabs(turnRate));
				// 算完成整个转向（转动 offset 那么多角度）所需要的时间，单位是“游戏帧”。这个估算是通过 总角度 / 每帧能转动的角度 得出的。
				const float framesToTurn = std::max(0.0001f, math::fabs(offset / absTurnSpeed));
				// 这是这个代码块的核心。它计算出了一个极其严格的“最大允许转弯速度”。
				// 逻辑: 它基于简单的物理公式 速度 = 距离 / 时间。
				// currWayPointDist: 这是单位到下一个路径点的距离。
				// framesToTurn: 这是单位完成转向所需要的时间。
				// currWayPointDist / framesToTurn: 这个除法计算出的速度，能确保单位只有在刚好完成转向之后，才会到达路径点。
				// * (.95f): 乘以0.95是为了增加一个5%的安全边际，让减速更加保守。
				// std::max(0.01f, ...): 确保这个速度至少是一个很小的正值。
				// 最终结果: maxSpeedToMakeTurn 被赋予了一个非常低的速度值，强制单位在碰撞后以“爬行”的速度进行转向，从而保证安全。
				maxSpeedToMakeTurn = std::max(0.01f, (currWayPointDist / framesToTurn) * (.95f));

				// bool printMoveInfo = (selectedUnitsHandler.selectedUnits.size() == 1)
				// 	&& (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end());
				// if (printMoveInfo) {
				// 	LOG("%s: dirSign %d, waypointDir=(%f,%f,%f), idealHeading=%d, offset=%d, absTurnSpeed=%f, framesToTurn=%f, maxSpeedToMakeTurn=%f"
				// 			, __func__
				// 			, dirSign, waypointDir.x, waypointDir.y, waypointDir.z, idealHeading, offset
				// 			, absTurnSpeed, framesToTurn, maxSpeedToMakeTurn);
				// }
			}

			// now apply the terrain and command restrictions
			// NOTE:
			//   if wantedSpeed > targetSpeed, the unit will
			//   not accelerate to speed > targetSpeed unless
			//   its actual max{Reverse}Speed is also changed
			//
			//   raise wantedSpeed iff the terrain-modifier is
			//   larger than 1 (so units still get their speed
			//   bonus correctly), otherwise leave it untouched
			//
			//   disallow changing speed (except to zero) without
			//   a path if not in FPS mode (FIXME: legacy PFS can
			//   return path when none should exist, mantis3720)
			// 调整上层命令的期望速度。如果地形有速度加成（groundSpeedMod > 1.0），就相应地提高 wantedSpeed。
			// 这样做是为了让 std::min(targetSpeed, wantedSpeed) 这个比较有意义，确保单位能真正享受到地形带来的速度提升。如果地形是减速的，wantedSpeed 保持不变。
			wantedSpeed *= std::max(groundSpeedMod, 1.0f);
			// 将地形修正应用到我们正在计算的实际目标速度 targetSpeed 上。如果地形是泥地（例如 groundSpeedMod = 0.5），targetSpeed 就会减半。
			targetSpeed *= groundSpeedMod;
			// 应用刹车逻辑。startBraking 是一个布尔值。如
			// 果它为 true (在C++中等于1)，那么 1 - startBraking 就是0，targetSpeed 会直接变为0，强制单位开始减速。
			// 如果为 false (0)，则乘以1，targetSpeed 不变。
			targetSpeed *= (1 - startBraking);
			// 应用停止意图。如果单位想要停止 (WantToStop() 为 true) 
			// 并且不在FPS模式下，那么 (1 - 1) || false 的结果是 false (0)，targetSpeed 也会变为0。
			// 在FPS模式下，这个检查被忽略。
			targetSpeed *= ((1 - WantToStop()) || fpsMode);
			// targetSpeed = std::min(...): 确保 targetSpeed 不会超过上层命令给出的（经过地形加成修正后的）wantedSpeed。
			targetSpeed = std::min(targetSpeed, wantedSpeed);
			// 这是最后的、也是最重要的一个限制。它将经过了地形、刹车、指令等所有修正后的 targetSpeed，
			// 与我们第一部分计算出的、极其严格的“碰撞后紧急转弯速度” maxSpeedToMakeTurn 进行比较，并取两者中的最小值。
			// 最终效果: 安全永远是第一位的。即使在高速公路上（地形加成高），
			// 如果单位刚刚撞了墙需要掉头，它的速度也会被强制降低到 maxSpeedToMakeTurn 这个极低的值，直到 limitSpeedForTurning 标志消失。
			targetSpeed = std::min(targetSpeed, maxSpeedToMakeTurn);
		} else {
			targetSpeed = 0.0f;
		}
	}

	deltaSpeed = pathController.GetDeltaSpeed(
		pathID,
		targetSpeed,
		currentSpeed,
		accRate,
		decRate,
		wantReverse,
		reversing
	);
}

/*
 * Changes the heading of the owner.
 * Also updates world position of aim points, and orientation if walking over terrain slopes.
 * FIXME near-duplicate of HoverAirMoveType::UpdateHeading
 */
/*
* 改变所有者的朝向。
* 同时更新瞄准点的世界位置，以及在地形斜坡上行走时的方向。
* FIXME HoverAirMoveType::UpdateHeading 存在近乎重复的代码
*/
// 参数: short newHeading 这是单位的“期望航向”，是一个介于 0 和 SPRING_CIRCLE_DIVS (65535) 之间的整数，代表一个完整的圆周。上层逻辑（如寻路、避障）计算出这个值，并传递给此函数。
void CGroundMoveType::ChangeHeading(short newHeading) {
	RECOIL_DETAILED_TRACY_ZONE;
	if (owner->IsFlying())
		return;
	if (owner->GetTransporter() != nullptr)
		return;

	// 将传入的“期望航向” newHeading 存储到成员变量 wantedHeading 中。这使得类的其他部分可以随时知道单位当前的目标朝向是什么
	wantedHeading = newHeading;
	// 如果单位的当前航向 owner->heading 已经等于期望航向 wantedHeading，那就意味着转向已经完成，不需要再做任何计算
	if (owner->heading == wantedHeading) {
		// 即使不需要转向，单位的 updir（上方向量）可能仍然需要更新，因为它会根据地形的坡度变化。
		// 例如，当单位从平地开上斜坡时，它的车体会向上倾斜。这个函数就是用来更新 updir、frontdir 和 rightdir 以匹配地形的
		owner->UpdateDirVectors(!owner->upright && owner->IsOnGround(), owner->IsInAir(), owner->unitDef->upDirSmoothing);
		return;
	}

	// 这两行代码的核心是计算出在本帧单位应该转动的角度增量
	#if (MODEL_TURN_INERTIA == 0)
	// 这是一个简单的模型。它会计算出从当前朝向到目标朝向的最短路径（顺时针或逆时针），然后返回一个不超过最大转向速率 turnRate 的角度增量。
	// 单位会立即以最大速率开始转向，也立即以最大速率停止
	const short rawDeltaHeading = pathController.GetDeltaHeading(pathID, wantedHeading, owner->heading, turnRate);
	#else
	// model rotational inertia (more realistic for ships)
	// 模拟转动惯量（使船舶运动更符合实际情况）
	// 这个版本的 GetDeltaHeading 会考虑当前的转向速度 turnSpeed。如果单位需要开始转向，它会施加 turnAccel 来加速；
	// 如果单位快要对准目标了，它会提前减速，以确保能平滑地停在 wantedHeading，而不是冲过头。
	// 这对于大型单位（如舰船）的行为表现至关重要。
	const short rawDeltaHeading = pathController.GetDeltaHeading(pathID, wantedHeading, owner->heading, turnRate, turnAccel, BrakingDistance(turnSpeed, turnAccel), &turnSpeed);
	#endif
	const short absDeltaHeading = rawDeltaHeading * Sign(rawDeltaHeading);
	// 这是一个阈值检查，用于决定是否需要通知单位的脚本（如COB或LUA脚本）它正在转向。
	if (absDeltaHeading >= minScriptChangeHeading)
		owner->script->ChangeHeading(rawDeltaHeading);
	// 这是真正执行旋转的函数。它将计算出的角度增量 rawDeltaHeading 加到单位的当前航向 owner->heading 上
	owner->AddHeading(rawDeltaHeading, !owner->upright && owner->IsOnGround(), owner->IsInAir(), owner->unitDef->upDirSmoothing);
	// 更新单位的方向向量 纯二维
	flatFrontDir = (owner->frontdir * XZVector).Normalize();
}


bool CGroundMoveType::CanApplyImpulse(const float3& impulse)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// NOTE: ships must be able to receive impulse too (for collision handling)
	if (owner->beingBuilt)
		return false;
	// will be applied to transporter instead
	if (owner->GetTransporter() != nullptr)
		return false;
	if (impulse.SqLength() <= 0.01f)
		return false;

	UseHeading(false);

	skidRotSpeed = 0.0f;
	skidRotAccel = 0.0f;

	float3 newSpeed = owner->speed + impulse;
	float3 skidDir = mix(float3(owner->frontdir), owner->frontdir * -1, reversing);

	// NOTE:
	//   we no longer delay the skidding-state until owner has "accumulated" an
	//   arbitrary hardcoded amount of impulse (possibly across several frames),
	//   but enter it on any vector that causes speed to become misaligned with
	//   frontdir
	// TODO
	//   there should probably be a configurable minimum-impulse below which the
	//   unit does not react at all but also does NOT "store" the impulse like a
	//   small-charge capacitor, or a more physically-based approach (|N*m*g*cf|
	//   > |impulse/dt|) could be used
	//
	const bool startSkidding = StartSkidding(newSpeed, skidDir);
	const bool startFlying = StartFlying(newSpeed, CGround::GetNormal(owner->pos.x, owner->pos.z));

	if (startSkidding)
		owner->script->StartSkidding(newSpeed);

	if (newSpeed.SqLength2D() >= 0.01f)
		skidDir = newSpeed.Normalize2D();

	skidRotVector = skidDir.cross(UpVector) * startSkidding;
	skidRotAccel = ((gsRNG.NextFloat() - 0.5f) * 0.04f) * startFlying;

	owner->SetPhysicalStateBit(CSolidObject::PSTATE_BIT_SKIDDING * (startSkidding | startFlying));
	owner->SetPhysicalStateBit(CSolidObject::PSTATE_BIT_FLYING * startFlying);

	// indicate we want to react to the impulse
	return true;
}

void CGroundMoveType::UpdateSkid()
{
	RECOIL_DETAILED_TRACY_ZONE;
	ASSERT_SYNCED(owner->midPos);

	const float3& pos = owner->pos;
	const float4& spd = owner->speed;

	const float minCollSpeed = owner->unitDef->minCollisionSpeed;
	const float groundHeight = GetGroundHeight(pos);
	const float negAltitude = groundHeight - pos.y;

	owner->SetVelocity(
		spd +
		owner->GetDragAccelerationVec(
			mapInfo->atmosphere.fluidDensity,
			mapInfo->water.fluidDensity,
			owner->unitDef->atmosphericDragCoefficient,
			owner->unitDef->groundFrictionCoefficient
		)
	);

	if (owner->IsFlying()) {
		const float collImpactSpeed = pos.IsInBounds()?
			-spd.dot(CGround::GetNormal(pos.x, pos.z)):
			-spd.dot(UpVector);
		const float impactDamageMul = collImpactSpeed * owner->mass * COLLISION_DAMAGE_MULT;

		if (negAltitude > 0.0f) {
			// ground impact, stop flying
			owner->ClearPhysicalStateBit(CSolidObject::PSTATE_BIT_FLYING);
			owner->Move(UpVector * negAltitude, true);

			// deal ground impact damage
			// TODO:
			//   bouncing behaves too much like a rubber-ball,
			//   most impact energy needs to go into the ground
			if (modInfo.allowUnitCollisionDamage && collImpactSpeed > minCollSpeed && minCollSpeed >= 0.0f)
				owner->DoDamage(DamageArray(impactDamageMul), ZeroVector, nullptr, -CSolidObject::DAMAGE_COLLISION_GROUND, -1);

			skidRotSpeed = 0.0f;
			// skidRotAccel = 0.0f;
		} else {
			owner->SetVelocity(spd + (UpVector * mapInfo->map.gravity));
		}
	} else {
		// *assume* this means the unit is still on the ground
		// (Lua gadgetry can interfere with our "physics" logic)
		float skidRotSpd = 0.0f;

		const bool onSlope = OnSlope(0.0f);
		const bool stopSkid = StopSkidding(spd, owner->frontdir);

		if (!onSlope && stopSkid) {
			skidRotSpd = math::floor(skidRotSpeed + skidRotAccel + 0.5f);
			skidRotAccel = (skidRotSpd - skidRotSpeed) * 0.5f;
			skidRotAccel *= math::DEG_TO_RAD;

			owner->ClearPhysicalStateBit(CSolidObject::PSTATE_BIT_SKIDDING);
			owner->script->StopSkidding();

			UseHeading(true);
			// update wanted-heading after coming to a stop
			ChangeHeading(owner->heading);
		} else {
			constexpr float speedReduction = 0.35f;

			// number of frames until rotational speed would drop to 0
			const float speedScale = owner->SetSpeed(spd);
			const float rotRemTime = std::max(1.0f, speedScale / speedReduction);

			if (onSlope) {
				const float3& normalVector = CGround::GetNormal(pos.x, pos.z);
				const float3 normalForce = normalVector * normalVector.dot(UpVector * mapInfo->map.gravity);
				const float3 newForce = UpVector * mapInfo->map.gravity - normalForce;

				owner->SetVelocity(spd + newForce);
				owner->SetVelocity(spd * (1.0f - (0.1f * normalVector.y)));
			} else {
				// RHS is clamped to 0..1
				owner->SetVelocity(spd * (1.0f - std::min(1.0f, speedReduction / speedScale)));
			}

			skidRotSpd = math::floor(skidRotSpeed + skidRotAccel * (rotRemTime - 1.0f) + 0.5f);
			skidRotAccel = (skidRotSpd - skidRotSpeed) / rotRemTime;
			skidRotAccel *= math::DEG_TO_RAD;

			if (math::floor(skidRotSpeed) != math::floor(skidRotSpeed + skidRotAccel)) {
				skidRotSpeed = 0.0f;
				skidRotAccel = 0.0f;
			}
		}

		if (negAltitude < (spd.y + mapInfo->map.gravity)) {
			owner->SetVelocity(spd + (UpVector * mapInfo->map.gravity));

			// flying requires skidding and relies on CalcSkidRot
			owner->SetPhysicalStateBit(CSolidObject::PSTATE_BIT_FLYING);
			owner->SetPhysicalStateBit(CSolidObject::PSTATE_BIT_SKIDDING);

			UseHeading(false);
		} else if (negAltitude > spd.y) {
			// LHS is always negative, so this becomes true when the
			// unit is falling back down and will impact the ground
			// in one frame
			const float3& gndNormal = (pos.IsInBounds())? CGround::GetNormal(pos.x, pos.z): UpVector;
			const float projSpeed = spd.dot(gndNormal);

			if (projSpeed > 0.0f) {
				// not possible without lateral movement
				owner->SetVelocity(spd * 0.95f);
			} else {
				owner->SetVelocity(spd + (gndNormal * (math::fabs(projSpeed) + 0.1f)));
				owner->SetVelocity(spd * 0.8f);
			}
		}
	}

	// finally update speed.w
	owner->SetSpeed(spd);
	// translate before rotate, match terrain normal if not in air
	owner->Move(spd, true);
	owner->UpdateDirVectors(!owner->upright && owner->IsOnGround(), owner->IsInAir(), owner->unitDef->upDirSmoothing);

	if (owner->IsSkidding()) {
		CalcSkidRot();
		CheckCollisionSkid();
	} else {
		// do this here since ::Update returns early if it calls us
		// HandleObjectCollisions(); // No longer the case
	}

	AdjustPosToWaterLine();

	// always update <oldPos> here so that <speed> does not make
	// extreme jumps when the unit transitions from skidding back
	// to non-skidding
	oldPos = owner->pos;

	ASSERT_SANE_OWNER_SPEED(spd);
	ASSERT_SYNCED(owner->midPos);
}

void CGroundMoveType::UpdateControlledDrop()
{
	RECOIL_DETAILED_TRACY_ZONE;
	const float3& pos = owner->pos;
	const float4& spd = owner->speed;
	const float3  acc = UpVector * std::min(mapInfo->map.gravity * owner->fallSpeed, 0.0f);
	const float   alt = GetGroundHeight(pos) - pos.y;

	owner->SetVelocity(spd + acc);
	owner->SetVelocity(
		spd +
		owner->GetDragAccelerationVec(
			mapInfo->atmosphere.fluidDensity,
			mapInfo->water.fluidDensity,
			owner->unitDef->atmosphericDragCoefficient,
			owner->unitDef->groundFrictionCoefficient * 10
		)
	);
	owner->SetSpeed(spd);
	owner->Move(spd, true);

	if (alt <= 0.0f)
		return;

	// ground impact, stop parachute animation
	owner->Move(UpVector * alt, true);
	owner->ClearPhysicalStateBit(CSolidObject::PSTATE_BIT_FALLING);
	owner->script->Landed();
}

void CGroundMoveType::CheckCollisionSkid()
{
	RECOIL_DETAILED_TRACY_ZONE;
	CUnit* collider = owner;

	// NOTE:
	//   the QuadField::Get* functions check o->midPos,
	//   but the quad(s) that objects are stored in are
	//   derived from o->pos (!)
	const float3& pos = collider->pos;

	const float colliderMinCollSpeed = collider->unitDef->minCollisionSpeed;
	      float collideeMinCollSpeed = 0.0f;

	// copy on purpose, since the below can call Lua
	QuadFieldQuery qfQuery;
	quadField.GetUnitsExact(qfQuery, pos, collider->radius);
	quadField.GetFeaturesExact(qfQuery, pos, collider->radius);

	for (CUnit* collidee: *qfQuery.units) {
		if (!collidee->HasCollidableStateBit(CSolidObject::CSTATE_BIT_SOLIDOBJECTS))
			continue;

		const UnitDef* collideeUD = collidee->unitDef;

		const float sqDist = (pos - collidee->pos).SqLength();
		const float totRad = collider->radius + collidee->radius;

		if ((sqDist >= totRad * totRad) || (sqDist <= 0.01f))
			continue;

		const float3 collSeparationDir = (pos - collidee->pos).SafeNormalize();

		if (collideeUD->IsImmobileUnit()) {
			const float collImpactSpeed = -collider->speed.dot(collSeparationDir);
			const float impactDamageMul = std::min(collImpactSpeed * collider->mass * COLLISION_DAMAGE_MULT, MAX_UNIT_SPEED);

			if (collImpactSpeed <= 0.0f)
				continue;

			// damage the collider, no added impulse
			if (modInfo.allowUnitCollisionDamage && collImpactSpeed > colliderMinCollSpeed && colliderMinCollSpeed >= 0.0f)
				collider->DoDamage(DamageArray(impactDamageMul), ZeroVector, collidee, -CSolidObject::DAMAGE_COLLISION_OBJECT, -1);

			// damage the (static) collidee based on collider's mass, no added impulse
			if (modInfo.allowUnitCollisionDamage && collImpactSpeed > (collideeMinCollSpeed = collideeUD->minCollisionSpeed) && collideeMinCollSpeed >= 0.0f)
				collidee->DoDamage(DamageArray(impactDamageMul), ZeroVector, collider, -CSolidObject::DAMAGE_COLLISION_OBJECT, -1);

			collider->Move(collSeparationDir * collImpactSpeed, true);
			collider->SetVelocity(collider->speed + ((collSeparationDir * collImpactSpeed) * 1.8f));
		} else {
			assert(collider->mass > 0.0f && collidee->mass > 0.0f);

			// don't conserve momentum (impact speed is halved, so impulses are too)
			// --> collisions are neither truly elastic nor truly inelastic to prevent
			// the simulation from blowing up from impulses applied to tight groups of
			// units
			const float collImpactSpeed = (collidee->speed - collider->speed).dot(collSeparationDir) * 0.5f;
			const float colliderRelMass = (collider->mass / (collider->mass + collidee->mass));
			const float colliderRelImpactSpeed = collImpactSpeed * (1.0f - colliderRelMass);
			const float collideeRelImpactSpeed = collImpactSpeed * (       colliderRelMass);

			const float  colliderImpactDmgMult = std::min(colliderRelImpactSpeed * collider->mass * COLLISION_DAMAGE_MULT, MAX_UNIT_SPEED);
			const float  collideeImpactDmgMult = std::min(collideeRelImpactSpeed * collider->mass * COLLISION_DAMAGE_MULT, MAX_UNIT_SPEED);
			const float3 colliderImpactImpulse = collSeparationDir * colliderRelImpactSpeed;
			const float3 collideeImpactImpulse = collSeparationDir * collideeRelImpactSpeed;

			if (collImpactSpeed <= 0.0f)
				continue;

			// damage the collider
			if (modInfo.allowUnitCollisionDamage && collImpactSpeed > colliderMinCollSpeed && colliderMinCollSpeed >= 0.0f)
				collider->DoDamage(DamageArray(colliderImpactDmgMult), collSeparationDir * colliderImpactDmgMult, collidee, -CSolidObject::DAMAGE_COLLISION_OBJECT, -1);

			// damage the collidee
			if (modInfo.allowUnitCollisionDamage && collImpactSpeed > (collideeMinCollSpeed = collideeUD->minCollisionSpeed) && collideeMinCollSpeed >= 0.0f)
				collidee->DoDamage(DamageArray(collideeImpactDmgMult), collSeparationDir * -collideeImpactDmgMult, collider, -CSolidObject::DAMAGE_COLLISION_OBJECT, -1);

			collider->Move( colliderImpactImpulse, true);
			collidee->Move(-collideeImpactImpulse, true);
			collider->SetVelocity        (collider->speed + colliderImpactImpulse);
			collidee->SetVelocityAndSpeed(collidee->speed - collideeImpactImpulse);
		}
	}

	for (CFeature* collidee: *qfQuery.features) {
		if (!collidee->HasCollidableStateBit(CSolidObject::CSTATE_BIT_SOLIDOBJECTS))
			continue;

		const float sqDist = (pos - collidee->pos).SqLength();
		const float totRad = collider->radius + collidee->radius;

		if ((sqDist >= totRad * totRad) || (sqDist <= 0.01f))
			continue;

		const float3 collSeparationDir = (pos - collidee->pos).SafeNormalize();
		const float collImpactSpeed = -collider->speed.dot(collSeparationDir);
		const float impactDamageMul = std::min(collImpactSpeed * collider->mass * COLLISION_DAMAGE_MULT, MAX_UNIT_SPEED);
		const float3 impactImpulse = collSeparationDir * collImpactSpeed;

		if (collImpactSpeed <= 0.0f)
			continue;

		// damage the collider, no added impulse (!)
		// the collidee feature can not be passed along to the collider as attacker
		// yet, keep symmetry and do not pass collider along to the collidee either
		if (modInfo.allowUnitCollisionDamage && collImpactSpeed > colliderMinCollSpeed && colliderMinCollSpeed >= 0.0f)
			collider->DoDamage(DamageArray(impactDamageMul), ZeroVector, nullptr, -CSolidObject::DAMAGE_COLLISION_OBJECT, -1);

		// damage the collidee feature based on collider's mass
		collidee->DoDamage(DamageArray(impactDamageMul), -impactImpulse, nullptr, -CSolidObject::DAMAGE_COLLISION_OBJECT, -1);

		collider->Move(impactImpulse, true);
		collider->SetVelocity(collider->speed + (impactImpulse * 1.8f));
	}

	// finally update speed.w
	collider->SetSpeed(collider->speed);

	ASSERT_SANE_OWNER_SPEED(collider->speed);
}

void CGroundMoveType::CalcSkidRot()
{
	RECOIL_DETAILED_TRACY_ZONE;
	skidRotSpeed += skidRotAccel;
	skidRotSpeed *= 0.999f;
	skidRotAccel *= 0.95f;

	const float angle = (skidRotSpeed / GAME_SPEED) * math::TWOPI;
	const float cosp = math::cos(angle);
	const float sinp = math::sin(angle);

	float3 f1 = skidRotVector * skidRotVector.dot(owner->frontdir);
	float3 f2 = owner->frontdir - f1;

	float3 r1 = skidRotVector * skidRotVector.dot(owner->rightdir);
	float3 r2 = owner->rightdir - r1;

	float3 u1 = skidRotVector * skidRotVector.dot(owner->updir);
	float3 u2 = owner->updir - u1;

	f2 = f2 * cosp + f2.cross(skidRotVector) * sinp;
	r2 = r2 * cosp + r2.cross(skidRotVector) * sinp;
	u2 = u2 * cosp + u2.cross(skidRotVector) * sinp;

	owner->frontdir = f1 + f2;
	owner->rightdir = r1 + r2;
	owner->updir    = u1 + u2;

	owner->UpdateMidAndAimPos();
}




/*
 * Dynamic obstacle avoidance, helps the unit to
 * follow the path even when it's not perfect.
 */
/**
 * 动态避障功能，有助于单位
 * 即使在路径不够理想的情况下也能沿着路径行进。
*/
float3 CGroundMoveType::GetObstacleAvoidanceDir(const float3& desiredDir) {
	#if (IGNORE_OBSTACLES == 1)
	return desiredDir;
	#endif

	// obstacle-avoidance only needs to run if the unit wants to move
	//  只有当单位想要移动时，才需要运行避障程序
	//  如果要停止，就没必要进行避障计算了。它将 lastAvoidanceDir (上一帧的避障方向) 更新为单位当前的朝向 flatFrontDir，然后返回这个值，使单位保持静止时的朝向。
	if (WantToStop())
		return lastAvoidanceDir = flatFrontDir;

	// Speed-optimizer. Reduces the times this system is run.
	// 速度优化器。减少该系统的运行次数。
	if ((gs->frameNum + owner->id) % modInfo.groundUnitCollisionAvoidanceUpdateRate) {
		if (!avoidingUnits)
			lastAvoidanceDir = desiredDir;

		return lastAvoidanceDir;
	}

	float3 avoidanceVec = ZeroVector; // 这个向量将用来累加所有来自障碍物的“排斥力”或“躲避向量”。
	float3 avoidanceDir = desiredDir; // 如果最终没有检测到任何需要躲避的障碍物，这个值就会被用作最终结果。

	if (avoidingUnits)
		avoidingUnits = false;

	lastAvoidanceDir = desiredDir; // 在计算开始前，先将 lastAvoidanceDir (上一帧的最终方向)也更新为当前的期望方向。这是一个备用值，以防计算中途出现问题。

	CUnit* avoider = owner;

	// const UnitDef* avoiderUD = avoider->unitDef;
	const MoveDef* avoiderMD = avoider->moveDef;

	// degenerate case: if facing anti-parallel to desired direction,
	// do not actively avoid obstacles since that can interfere with
	// normal waypoint steering (if the final avoidanceDir demands a
	// turn in the opposite direction of desiredDir)
	// 计算单位当前3D朝向与期望方向的点积。点积为负数意味着两个向量之间的夹角大于90度。
	// 如果单位的朝向与期望方向几乎相反（例如，正在进行180度大转弯），就暂时不进行主动避障。
	// 因为此时避障逻辑可能会干扰正常的转向行为，导致奇怪的抖动。函数直接返回上一帧的方向。
	if (avoider->frontdir.dot(desiredDir) < 0.0f)
		return lastAvoidanceDir;

	// 用于混合向量的权重。
	static constexpr float AVOIDER_DIR_WEIGHT = 1.0f;
	static constexpr float DESIRED_DIR_WEIGHT = 0.5f;
	// 用于与上一帧结果进行平滑插值的alpha值。
	static constexpr float LAST_DIR_MIX_ALPHA = 0.7f;
	// 点积结果如果小于这个值，意味着障碍物与单位朝向的夹角大于120度，即障碍物在单位的侧后方，可以忽略。
	static const     float MAX_AVOIDEE_COSINE = math::cosf(120.0f * math::DEG_TO_RAD);

	// now we do the obstacle avoidance proper
	// avoider always uses its never-rotated MoveDef footprint
	// note: should increase radius for smaller turnAccel values
	// 计算本次避障的检测半径
	// 这个半径是动态的，与单位的当前速度 currentSpeed 成正比。速度越快，需要看得越远，提前反应。avoider->radius * 2.0f 提供了一个基础半径
	const float avoidanceRadius = std::max(currentSpeed, 1.0f) * (avoider->radius * 2.0f);
	// 获取避障者自身的碰撞半径。这个半径基于 MoveDef 的占地面积计算，比单位模型的视觉半径更准确。
	const float avoiderRadius = avoiderMD->CalcFootPrintMinExteriorRadius();

	MoveTypes::CheckCollisionQuery avoiderInfo(avoider);

	QuadFieldQuery qfQuery;
	qfQuery.threadOwner = ThreadPool::GetThreadNum();
	// 请求在 avoider->pos (单位位置) 周围 avoidanceRadius (检测半径) 内的所有“固体”对象。
	quadField.GetSolidsExact(qfQuery, avoider->pos, avoidanceRadius, 0xFFFFFFFF, CSolidObject::CSTATE_BIT_SOLIDOBJECTS);
	// 遍历quadField找到邻近物体
	for (const CSolidObject* avoidee: *qfQuery.solids) {
	
		const MoveDef* avoideeMD = avoidee->moveDef;
		const UnitDef* avoideeUD = dynamic_cast<const UnitDef*>(avoidee->GetDef());

		// cases in which there is no need to avoid this obstacle
		// 不能自己躲避自己。
		if (avoidee == owner)
			continue;
		// do not avoid statics (it interferes too much with PFS)
		// 为空说明是静态物体（如岩石、建筑残骸），这个系统不处理静态障碍物，因为它们应该由全局路径规划器PFS来处理。
		if (avoideeMD == nullptr)
			continue;
		// ignore aircraft (or flying ground units)
		// 不躲避空中的或正在飞行的单位。
		if (avoidee->IsInAir() || avoidee->IsFlying())
			continue;
		// 如果根据移动定义，这个障碍物是非阻塞的（例如可以穿过）
		if (CMoveMath::IsNonBlocking(avoidee, &avoiderInfo))
			continue;
		// 或者可以被我们的单位碾压，那就不需要躲避。
		if (!CMoveMath::CrushResistant(*avoiderMD, avoidee))
			continue;
		// 是否是可移动单位
		const bool avoideeMobile  = (avoideeMD != nullptr);
		// 是否是可以被推动的单位
		const bool avoideeMovable = (avoideeUD != nullptr && !static_cast<const CUnit*>(avoidee)->moveType->IsPushResistant());
		// 这里的speed是单位的当前速度向量。
		// 预测了避障者（avoider）和障碍物（avoidee）在下一帧的大致位置。然后计算这两个预测位置之间的向量。
		// 这样做是为了提前反应，躲避对方将要到达的位置，而不是它现在所在的位置。
		// avoidee -> avoider
		const float3 avoideeVector = (avoider->pos + avoider->speed) - (avoidee->pos + avoidee->speed);

		// use the avoidee's MoveDef footprint as radius if it is mobile
		// use the avoidee's Unit (not UnitDef) footprint as radius otherwise
		// 获取障碍物的碰撞半径。
		// 如果是可移动单位 (avoideeMobile 为 true)，就使用 MoveDef 中定义的、更精确的占地面积（footprint）来计算半径。
		// 否则（如果是静态物体），就使用其作为 CSolidObject 的通用半径计算方法。
		const float avoideeRadius = avoideeMobile?
			avoideeMD->CalcFootPrintMinExteriorRadius():
			avoidee->CalcFootPrintMinExteriorRadius();
		
		// 计算我们自己和障碍物的半径之和。如果我们的中心点与对方中心点的距离小于这个值，就意味着发生了碰撞。
		const float avoidanceRadiusSum = avoiderRadius + avoideeRadius;
		// 计算双方的总质量。
		const float avoidanceMassSum = avoider->mass + avoidee->mass;
		// 计算一个质量缩放因子，用于调整躲避反应的强度
		// 如果障碍物是可移动的，这个因子就是障碍物的质量占总质量的比例。这意味着，
		// 当我们躲避一个比我们重得多的单位时，我们会做出更强的躲避反应（因为对方不太可能为我们让路）。反之，躲避轻单位时反应会弱一些。
		// 如果障碍物是静态的，缩放因子为 1.0f，意味着我们会对其做出最大强度的躲避反应（因为它完全不会移动）
		const float avoideeMassScale = avoideeMobile? (avoidee->mass / avoidanceMassSum): 1.0f;
		// 计算预测的相对位置向量的长度的平方。在编程中，比较距离的平方比比较距离本身要快，因为它避免了开方（sqrt）运算。
		const float avoideeDistSq = avoideeVector.SqLength();
		// 计算实际的预测距离，并加上一个很小的数 0.01f 来防止后续计算中出现除以零的错误。
		const float avoideeDist   = math::sqrt(avoideeDistSq) + 0.01f;

		// do not bother steering around idling MOBILE objects
		// (since collision handling will just push them aside)
		// 适用于那些既能自己移动、又能被我们推开的障碍物
		if (avoideeMobile && avoideeMovable) {
			// !avoiderMD->avoidMobilesOnPath: 第一个条件是检查我们自己的 MoveDef 是否设置了“不在路径上躲避移动单位”。如果设置了，就忽略。
			// !avoidee->IsMoving() && avoidee->allyteam == avoider->allyteam: 如果障碍物是一个没有在移动的、并且是盟友的单位，我们就忽略它。
			// 为什么？ 想象一下在一条狭窄的通道里，一队坦克正在前进，前面的坦克停了一下。如果后面的坦克执行标准的避障逻辑，它会尝试向左或向右绕。
			// 但在狭窄通道里，这可能会导致它撞墙或者与旁边的单位发生更复杂的碰撞，最终导致整个队伍卡死。
			// 这个规则的意图是：对于静止的盟友，我们不主动进行转向躲避，而是直接开过去，
			// 依赖更底层的物理碰撞系统（HandleObjectCollisions）去推动它。这样可以更有效地解决单位“礼让”造成的交通堵塞。
			if (!avoiderMD->avoidMobilesOnPath || (!avoidee->IsMoving() && avoidee->allyteam == avoider->allyteam))
				continue;
		}

		// ignore objects that are more than this many degrees off-center from us
		// NOTE:
		//   if MAX_AVOIDEE_COSINE is too small, then this condition can be true
		//   one frame and false the next (after avoider has turned) causing the
		//   avoidance vector to oscillate --> units with turnInPlace = true will
		//   slow to a crawl as a result
		// 释提醒了一个潜在问题：如果这个角度阈值设置得太窄（例如90度），
		// 单位在转向时可能会导致障碍物“时而进入，时而离开”躲避范围，从而引起躲避向量的抖动，导致单位移动不流畅甚至减速。

		// 忽略那些位于我们（avoider）侧后方的障碍物
		// avoideeVector 是从障碍物预测的下一帧位置指向我们预测的下一帧位置的向量。
		// / avoideeDist 将其归一化，得到一个纯粹的方向向量。
		// 前面的负号 - 将这个向量反转，现在它代表了从我们指向障碍物的方向。
		if (avoider->frontdir.dot(-(avoideeVector / avoideeDist)) < MAX_AVOIDEE_COSINE)
			continue;
		// 距离过滤。忽略那些虽然在前方，但离得太远的障碍物。
		if (avoideeDistSq >= Square(std::max(currentSpeed, 1.0f) * GAME_SPEED + avoidanceRadiusSum))
			continue;
		// 如果障碍物离我们的距离，比我们离最终目标点 goalPos 的距离还要远，那么我们通常不需要现在就担心它。我们会在沿着路径前进的过程中自然地处理它。
		if (avoideeDistSq >= avoider->pos.SqDistance2D(goalPos))
			continue;

		// if object and unit in relative motion are closing in on one another
		// (or not yet fully apart), then the object is on the path of the unit
		// and they are not collided
		// 调试可视化
		if (DEBUG_DRAWING_ENABLED) {
			if (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end()){
				geometryLock.lock();
				geometricObjects->AddLine(avoider->pos + (UpVector * 20.0f), avoidee->pos + (UpVector * 20.0f), 3, 1, 4);
				geometryLock.unlock();
			}
		}
		// 这里需要使用几何画板验证一下
		// 通过GGB验证结果规律 如果单位在前方向量左边值为1 否则为 -1 
		float avoiderTurnSign = -Sign(avoidee->pos.dot(avoider->rightdir) - avoider->pos.dot(avoider->rightdir));
		// 通过GGB验证结果规律 如果单位在前方向量左边值为1 否则为 -1
		float avoideeTurnSign = -Sign(avoider->pos.dot(avoidee->rightdir) - avoidee->pos.dot(avoidee->rightdir));

		// for mobile units, avoidance-response is modulated by angle
		// between avoidee's and avoider's frontdir such that maximal
		// avoidance occurs when they are anti-parallel
		// avoidanceCosAngle: 计算我们和障碍物朝向的点积，即朝向夹角的余弦。-1 表示迎头相对，+1 表示完全同向。
		const float avoidanceCosAngle = std::clamp(avoider->frontdir.dot(avoidee->frontdir), -1.0f, 1.0f);
		// 这是角度响应强度。1.0f - avoidanceCosAngle 这个表达式使得当两个单位迎头相向时（cos=-1），响应值最大（为2）；同向时（cos=1），
		// 响应值最小（为0）。* int(avoideeMobile) 表示这个角度调制只对可移动的单位生效。最后的 + 0.1f 保证了总有一个最小的响应值。
		const float avoidanceResponse = (1.0f - avoidanceCosAngle * int(avoideeMobile)) + 0.1f;
		// 这是距离衰减强度。avoideeDist / (5.0f * avoidanceRadiusSum) 计算了一个相对距离。
		// 当单位很近时，这个比值接近0，avoidanceFallOff 接近1.0（强度最大）。
		// 当单位远离时，比值增大，avoidanceFallOff 接近0（强度减弱）。5.0f * 表示衰减的有效范围大约是双方半径和的5倍。
		const float avoidanceFallOff  = (1.0f - std::min(1.0f, avoideeDist / (5.0f * avoidanceRadiusSum)));

		// if parties are anti-parallel, it is always more efficient for
		// both to turn in the same local-space direction (either R/R or
		// L/L depending on relative object positions) but there exists
		// a range of orientations for which the signs are not equal
		//
		// (this is also true for the parallel situation, but there the
		// degeneracy only occurs when one of the parties is behind the
		// other and can be ignored)
		// 如果双方是反平行的，那么对两者而言，
		// 朝相同的局部空间方向转弯（根据相对物体位置，要么都右转 / R，要么都左转 / L）总是更高效的，
		// 但存在一定的朝向范围，在该范围内双方的转向方向并不相同
		//
		// （这一点在平行情况下同样成立，但在平行时，
		// 只有当其中一方位于另一方后方时才会出现这种退化情况，这种情况可以忽略不计）

		// 如果 avoidanceCosAngle < 0.0f，说明两个单位大体上是相向而行。
		// 这时，为了避免双方都向对方内侧转向而“顶牛”，代码尝试让双方选择一个统一的转向方向（都向自己的左边或都向自己的右边）。
		if (avoidanceCosAngle < 0.0f)
			avoiderTurnSign = std::max(avoiderTurnSign, avoideeTurnSign);
		// 根据 avoiderTurnSign 决定一个基础的躲避方向，即我们自己的左方向或右方向。
		avoidanceDir = avoider->rightdir * AVOIDER_DIR_WEIGHT * avoiderTurnSign;
		// avoidanceVec += ...: 然后，将这个基础方向向量乘以我们刚刚计算出的所有强度因子：
		// avoidanceResponse（角度响应）、
		// avoidanceFallOff（距离衰减）
		// 以及 avoideeMassScale（质量比例）。
		// 这个最终计算出的、带有权重的向量被加到 avoidanceVec 上。avoidanceVec 会累加所有障碍物产生的这种“推力”。
		avoidanceVec += (avoidanceDir * avoidanceResponse * avoidanceFallOff * avoideeMassScale);
		// 设置状态标志。
		if (!avoidingUnits)
			avoidingUnits = true;
	}

	// use a weighted combination of the desired- and the avoidance-directions
	// also linearly smooth it using the vector calculated the previous frame
	// 采用期望方向和避障方向的加权组合, 同时使用上一帧计算出的向量对其进行线性平滑处理
	// 两次线性混合 第一次期望方向和避障方向, 第二次 第一次计算结果与上一次避障方向混合
	avoidanceDir = (mix(desiredDir, avoidanceVec, DESIRED_DIR_WEIGHT)).SafeNormalize();
	avoidanceDir = (mix(avoidanceDir, lastAvoidanceDir, LAST_DIR_MIX_ALPHA)).SafeNormalize();

	if (DEBUG_DRAWING_ENABLED) {
		if (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end()) {
			const float3 p0 = owner->pos + (    UpVector * 20.0f);
			const float3 p1 =         p0 + (avoidanceVec * 40.0f);
			const float3 p2 =         p0 + (avoidanceDir * 40.0f);

			geometryLock.lock();
			const int avFigGroupID = geometricObjects->AddLine(p0, p1, 8.0f, 1, 4);
			const int adFigGroupID = geometricObjects->AddLine(p0, p2, 8.0f, 1, 4);

			geometricObjects->SetColor(avFigGroupID, 1, 0.3f, 0.3f, 0.6f);
			geometricObjects->SetColor(adFigGroupID, 1, 0.3f, 0.3f, 0.6f);
			geometryLock.unlock();
		}
	}
	// 最终平滑方向
	return (lastAvoidanceDir = avoidanceDir);
}



#if 0
// Calculates an approximation of the physical 2D-distance between given two objects.
// Old, no longer used since all separation tests are based on FOOTPRINT_RADIUS now.
float CGroundMoveType::Distance2D(CSolidObject* object1, CSolidObject* object2, float marginal)
{
	// calculate the distance in (x,z) depending
	// on the shape of the object footprints
	float dist2D = 0.0f;

	const float xs = ((object1->xsize + object2->xsize) * (SQUARE_SIZE >> 1));
	const float zs = ((object1->zsize + object2->zsize) * (SQUARE_SIZE >> 1));

	if (object1->xsize == object1->zsize || object2->xsize == object2->zsize) {
		// use xsize as a cylindrical radius.
		const float3 distVec = object1->midPos - object2->midPos;

		dist2D = distVec.Length2D() - xs + 2.0f * marginal;
	} else {
		// Pytagorean sum of the x and z distance.
		float3 distVec;

		const float xdiff = math::fabs(object1->midPos.x - object2->midPos.x);
		const float zdiff = math::fabs(object1->midPos.z - object2->midPos.z);

		distVec.x = xdiff - xs + 2.0f * marginal;
		distVec.z = zdiff - zs + 2.0f * marginal;

		if (distVec.x > 0.0f && distVec.z > 0.0f) {
			dist2D = distVec.Length2D();
		} else if (distVec.x < 0.0f && distVec.z < 0.0f) {
			dist2D = -distVec.Length2D();
		} else if (distVec.x > 0.0f) {
			dist2D = distVec.x;
		} else {
			dist2D = distVec.z;
		}
	}

	return dist2D;
}
#endif

// Creates a path to the goal.
unsigned int CGroundMoveType::GetNewPath()
{
	RECOIL_DETAILED_TRACY_ZONE;
	assert(!ThreadPool::inMultiThreadedSection);
	unsigned int newPathID = 0;

	#ifdef PATHING_DEBUG
	if (DEBUG_DRAWING_ENABLED) {
		bool printMoveInfo = (selectedUnitsHandler.selectedUnits.size() == 1)
			&& (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end());
		if (printMoveInfo) {
			LOG("%s useRawMovement (%d)", __func__, useRawMovement);
		}
	}
	#endif

	if (useRawMovement)
		return newPathID;

	#ifdef PATHING_DEBUG
	if (DEBUG_DRAWING_ENABLED) {
		bool printMoveInfo = (selectedUnitsHandler.selectedUnits.size() == 1)
			&& (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end());
		if (printMoveInfo) {
			LOG("%s Dist Away (%f) Goal Radius (%f)", __func__
					, (owner->pos - goalPos).SqLength2D()
					, Square(goalRadius + extraRadius));
		}
	}
	#endif

	// avoid frivolous requests if called from outside StartMoving*()
	if ((owner->pos - goalPos).SqLength2D() <= Square(goalRadius + extraRadius))
		return newPathID;

	if ((newPathID = pathManager->RequestPath(owner, owner->moveDef, owner->pos, goalPos, goalRadius + extraRadius, true)) != 0) {
		atGoal = false;
		atEndOfPath = false;
		lastWaypoint = false;

		earlyCurrWayPoint = currWayPoint = pathManager->NextWayPoint(owner, newPathID, 0,   owner->pos, std::max(WAYPOINT_RADIUS, currentSpeed * 1.05f), true);
		earlyNextWayPoint = nextWayPoint = pathManager->NextWayPoint(owner, newPathID, 0, currWayPoint, std::max(WAYPOINT_RADIUS, currentSpeed * 1.05f), true);

		pathController.SetRealGoalPosition(newPathID, goalPos);
		pathController.SetTempGoalPosition(newPathID, currWayPoint);
	} else {
		Fail(false);
	}

	return newPathID;
}

void CGroundMoveType::ReRequestPath(bool forceRequest) {
	RECOIL_DETAILED_TRACY_ZONE;
	if (forceRequest) {
		assert(!ThreadPool::inMultiThreadedSection);
		// StopEngine(false);
		StartEngine(false);
		wantRepath = false;
		lastRepathFrame = gs->frameNum;
		return;
	}

	if (!wantRepath) {
		wantRepath = true;
		wantRepathFrame = gs->frameNum;
		bestLastWaypointDist = std::numeric_limits<float>::infinity();
	}
}

bool CGroundMoveType::CanSetNextWayPoint(int thread) {
	ZoneScoped;

	if (pathID == 0)
		return false;
	if (!pathController.AllowSetTempGoalPosition(pathID, nextWayPoint))
		return false;
	if (atEndOfPath)
		return false;

	const float3& pos = owner->pos;
		  float3& cwp = earlyCurrWayPoint;
		  float3& nwp = earlyNextWayPoint;

	// QTPFS ONLY PATH
	if (pathManager->PathUpdated(pathID)) {
		// path changed while we were following it (eg. due
		// to terrain deformation) in between two waypoints
		// but still has the same ID; in this case (which is
		// specific to QTPFS) we don't go through GetNewPath
		//
		cwp = pathManager->NextWayPoint(owner, pathID, 0, pos, std::max(WAYPOINT_RADIUS, currentSpeed * 1.05f), true);
		nwp = pathManager->NextWayPoint(owner, pathID, 0, cwp, std::max(WAYPOINT_RADIUS, currentSpeed * 1.05f), true);

		currWayPointDist = cwp.distance2D(pos);
		SetWaypointDir(cwp, pos);
		wantRepath = false;
	
		pathManager->ClearPathUpdated(pathID);
	}

	if (earlyCurrWayPoint.y == -1.0f || earlyNextWayPoint.y == -1.0f)
		return true;

	if (DEBUG_DRAWING_ENABLED) {
		if (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end()) {
			// plot the vectors to {curr, next}WayPoint

			geometryLock.lock();
			const int cwpFigGroupID = geometricObjects->AddLine(pos + (UpVector * 20.0f), cwp + (UpVector * (pos.y + 20.0f)), 8.0f, 1, 4);
			const int nwpFigGroupID = geometricObjects->AddLine(pos + (UpVector * 20.0f), nwp + (UpVector * (pos.y + 20.0f)), 8.0f, 1, 4);

			geometricObjects->SetColor(cwpFigGroupID, 1, 0.3f, 0.3f, 0.6f);
			geometricObjects->SetColor(nwpFigGroupID, 1, 0.3f, 0.3f, 0.6f);
			geometryLock.unlock();
		}
	}

	float cwpDistSq = cwp.SqDistance2D(pos);
	// -1 to avoid units checking for corners to rotate slightly and fail to escape when at max
	// distance for allowSkip. The slide/corner checks are done upto 8 elmos.
	const bool allowSkip = (cwpDistSq < Square(SQUARE_SIZE - 1));
	if (!allowSkip) {
		const bool skipRequested = (earlyCurrWayPoint.y == -2.0f || earlyNextWayPoint.y == -2.0f);
		if (!skipRequested) {
			// perform a turn-radius check: if the waypoint lies outside
			// our turning circle, do not skip since we can steer toward
			// this waypoint and pass it without slowing down
			// note that the DIAMETER of the turning circle is calculated
			// to prevent sine-like "snaking" trajectories; units capable
			// of instant turns *and* high speeds also need special care
			const int dirSign = Sign(int(!reversing));

			const float absTurnSpeed = turnRate;
			const float framesToTurn = SPRING_CIRCLE_DIVS / absTurnSpeed;

			const float turnRadius = std::max((currentSpeed * framesToTurn) * math::INVPI2, currentSpeed * 1.05f) * 2.f;
			const float waypointDot = std::clamp(waypointDir.dot(flatFrontDir * dirSign), -1.0f, 1.0f);

			#if 1

			// #ifdef PATHING_DEBUG
			// if (DEBUG_DRAWING_ENABLED)
			// {
			// 	bool printMoveInfo = (selectedUnitsHandler.selectedUnits.size() == 1)
			// 		&& (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end());
			// 	if (printMoveInfo) {
			// 		LOG("%s absTurnSpeed %f, cwpDistSq=%f, skip=%d", __func__, absTurnSpeed, cwpDistSq, int(allowSkip));
			// 		LOG("%s turnradius %f = max(%f*%f*%f=%f, %f)", __func__
			// 				, turnRadius, currentSpeed, framesToTurn, math::INVPI2
			// 				, (currentSpeed * framesToTurn) * math::INVPI2
			// 				, currentSpeed * 1.05f);
			// 		LOG("%s currWayPointDist %f, turn radius = %f", __func__, currWayPointDist, turnRadius);
			// 	}
			// }
			// #endif

			// wp outside turning circle
			if (currWayPointDist > turnRadius)
				return false;

			// #ifdef PATHING_DEBUG
			// if (DEBUG_DRAWING_ENABLED)
			// {
			// 	bool printMoveInfo = (selectedUnitsHandler.selectedUnits.size() == 1)
			// 		&& (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end());
			// 	if (printMoveInfo) {
			// 		LOG("%s currWayPointDist %f, max=%f, wayDot=%f", __func__
			// 				, currWayPointDist
			// 				, std::max(SQUARE_SIZE * 1.0f, currentSpeed * 1.05f)
			// 				, waypointDot);
			// 	}
			// }
			// #endif

			// wp inside but ~straight ahead and not reached within one tick
			if (currWayPointDist > std::max(SQUARE_SIZE * 1.0f, currentSpeed * 1.05f) && waypointDot >= 0.995f)
				return false;

			#else

			if ((currWayPointDist > std::max(turnRadius * 2.0f, 1.0f * SQUARE_SIZE)) && (waypointDot >= 0.0f))
				return false;

			if ((currWayPointDist > std::max(turnRadius * 1.0f, 1.0f * SQUARE_SIZE)) && (waypointDot <  0.0f))
				return false;

			if (math::acosf(waypointDot) < ((turnRate / SPRING_CIRCLE_DIVS) * math::TWOPI))
				return false;
			#endif

		}

		// Check if the unit has overshot the current waypoint and on route to the next waypoint.
		// const float3& p0 = earlyCurrWayPoint, v0 = float3(p0.x - pos.x, 0.0f, p0.z - pos.z);
		// const float3& p1 = earlyNextWayPoint, v1 = float3(p1.x - pos.x, 0.0f, p1.z - pos.z);
		// bool unitIsBetweenWaypoints = (v0.dot(v1) <= -0.f);

		// The last waypoint on a bad path will never pass a range check so don't try.
		//bool doRangeCheck = !pathManager->NextWayPointIsUnreachable(pathID);
						// && !unitIsBetweenWaypoints;
		//if (doRangeCheck) {
			const float searchRadius = std::max(WAYPOINT_RADIUS, currentSpeed * 1.05f);
			const float3 targetPos = nwp;

			// check the between pos and cwp for obstacles
			// if still further than SS elmos from waypoint, disallow skipping
			const bool rangeTest = owner->moveDef->DoRawSearch(owner, owner->moveDef, pos, targetPos, 0, true, true, false, nullptr, nullptr, nullptr, thread);

			// {
			// bool printMoveInfo = (selectedUnitsHandler.selectedUnits.size() == 1)
			// 	&& (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end());
			// if (printMoveInfo)
			// 	LOG("%s: test move square (%f,%f)->(%f,%f) = %f (range=%d skip=%d)", __func__
			// 			, pos.x, pos.z, targetPos.x, targetPos.z, math::sqrtf(cwpDistSq)
			// 			, int(rangeTest), int(allowSkip));
			// }

			if (!rangeTest)
				return false;
		//}
	}

	{
		const float curGoalDistSq = (earlyCurrWayPoint - goalPos).SqLength2D();
		const float minGoalDistSq = (UNIT_HAS_MOVE_CMD(owner)) ?
			Square((goalRadius + extraRadius) * (numIdlingSlowUpdates + 1)):
			Square((goalRadius + extraRadius)                             );

		// trigger Arrived on the next Update (only if we have non-temporary waypoints)
		// note:
		//   coldet can (very rarely) interfere with this, causing it to remain false
		//   a unit would then keep moving along its final waypoint-direction forever
		//   if atGoal, so we require waypointDir to always be updated in FollowPath
		//   (checking curr == next is not perfect, becomes true a waypoint too early)
		//
		// atEndOfPath |= (currWayPoint == nextWayPoint);
		atEndOfPath |= (curGoalDistSq <= minGoalDistSq);

		if (!atEndOfPath) {
			lastWaypoint |= (earlyCurrWayPoint.same(earlyNextWayPoint)
						&& pathManager->CurrentWaypointIsUnreachable(pathID)
						&& (cwpDistSq <= minGoalDistSq));
			if (lastWaypoint) {
				// incomplete path and last valid waypoint has been reached.
				pathingFailed = true;
				return false;
			}
		}
	}

	if (atEndOfPath) {
		earlyCurrWayPoint = goalPos;
		earlyNextWayPoint = goalPos;
		return false;
	}

	return true;
}

void CGroundMoveType::SetNextWayPoint(int thread)
{
	RECOIL_DETAILED_TRACY_ZONE;
	assert(!useRawMovement);

	if (CanSetNextWayPoint(thread)) {
		#ifdef PATHING_DEBUG
		if (DEBUG_DRAWING_ENABLED) {
			bool printMoveInfo = (selectedUnitsHandler.selectedUnits.size() == 1)
				&& (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end());
			if (printMoveInfo) {
				LOG("%s setting next waypoint (%d:%d)", __func__, owner->id, owner->moveDef->pathType);
			}
		}
		#endif
		// Not sure this actually does anything.
		pathController.SetTempGoalPosition(pathID, earlyNextWayPoint);

		int32_t update = 1;
		while (update-- > 0) {
			earlyCurrWayPoint = earlyNextWayPoint;
			earlyNextWayPoint = pathManager->NextWayPoint(owner, pathID, 0, earlyCurrWayPoint, std::max(WAYPOINT_RADIUS, currentSpeed * 1.05f), true);
			update += (earlyCurrWayPoint.y == (-1.f) && earlyNextWayPoint.y != (-1.f));
		}

		if (limitSpeedForTurning > 0)
			--limitSpeedForTurning;

		// Prevent delay repaths because the waypoints have been updated.
		wantRepath = false;
	}

	if (earlyNextWayPoint.x == -1.0f && earlyNextWayPoint.z == -1.0f) {
		//Fail(false);
		pathingFailed = true;
		#ifdef PATHING_DEBUG
		if (DEBUG_DRAWING_ENABLED) {
			bool printMoveInfo = (selectedUnitsHandler.selectedUnits.size() == 1)
				&& (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end());
			if (printMoveInfo) {
				LOG("%s path failed", __func__);
			}
		}
		#endif
		return;
	}

	MoveTypes::CheckCollisionQuery colliderInfo(owner);
	
	const auto CWP_BLOCK_MASK = CMoveMath::SquareIsBlocked(*owner->moveDef, earlyCurrWayPoint, &colliderInfo);
	const auto NWP_BLOCK_MASK = CMoveMath::SquareIsBlocked(*owner->moveDef, earlyNextWayPoint, &colliderInfo);

	if ((CWP_BLOCK_MASK & CMoveMath::BLOCK_STRUCTURE) == 0 && (NWP_BLOCK_MASK & CMoveMath::BLOCK_STRUCTURE) == 0){
		#ifdef PATHING_DEBUG
		if (DEBUG_DRAWING_ENABLED) {
			bool printMoveInfo = (selectedUnitsHandler.selectedUnits.size() == 1)
				&& (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end());
			if (printMoveInfo) {
				LOG("%s path is clear", __func__);
			}
		}
		#endif
		return;
	}

	// this can happen if we crushed a non-blocking feature
	// and it spawned another feature which we cannot crush
	// (eg.) --> repath

	ReRequestPath(false);
	#ifdef PATHING_DEBUG
	if (DEBUG_DRAWING_ENABLED) {
		bool printMoveInfo = (selectedUnitsHandler.selectedUnits.size() == 1)
			&& (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end());
		if (printMoveInfo) {
			LOG("%s requesting new path", __func__);
		}
	}
	#endif
}

/*
Gives the position this unit will end up at with full braking
from current velocity.
*/
float3 CGroundMoveType::Here() const
{
	RECOIL_DETAILED_TRACY_ZONE;
	const float dist = BrakingDistance(currentSpeed, decRate);
	const int   sign = Sign(int(!reversing));

	const float3 pos2D = owner->pos * XZVector;
	const float3 dir2D = flatFrontDir * dist * sign;

	return (pos2D + dir2D);
}

void CGroundMoveType::StartEngine(bool callScript) {
	RECOIL_DETAILED_TRACY_ZONE;
	if (pathID == 0)
		pathID = GetNewPath();
	else {
		if (nextPathId != 0) {
			pathManager->DeletePath(nextPathId);
		}
		nextPathId = GetNewPath();

		// This can happen if the current path has not been resolved yet and the pathing system has
		// decided to optimize by updating the existing search request.
		if (nextPathId == pathID) {
			nextPathId = 0;
		}
	}

	if (pathID != 0) {
		// pathManager->UpdatePath(owner, pathID);

		if (callScript) {
			// makes no sense to call this unless we have a new path
			owner->script->StartMoving(reversing);
		}
	}
}

void CGroundMoveType::StopEngine(bool callScript, bool hardStop) {
	RECOIL_DETAILED_TRACY_ZONE;
	assert(!ThreadPool::inMultiThreadedSection);
	if (pathID != 0 || nextPathId != 0) {
		if (pathID != 0) {
			pathManager->DeletePath(pathID);
			pathID = 0;
		}
		if (nextPathId != 0) {
			pathManager->DeletePath(nextPathId);
			nextPathId = 0;
		}

		if (callScript)
			owner->script->StopMoving();
	}

	owner->SetVelocityAndSpeed(owner->speed * (1 - hardStop));

	currentSpeed *= (1 - hardStop);
	wantedSpeed = 0.0f;
	limitSpeedForTurning = 0;
	bestReattemptedLastWaypointDist = std::numeric_limits<decltype(bestReattemptedLastWaypointDist)>::infinity();
}

/* Called when the unit arrives at its goal. */
void CGroundMoveType::Arrived(bool callScript)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// can only "arrive" if the engine is active
	if (progressState == Active) {
		eventHandler.UnitArrivedAtGoal(owner);

		StopEngine(callScript);

		if (owner->team == gu->myTeam)
			Channels::General->PlayRandomSample(owner->unitDef->sounds.arrived, owner);

		// and the action is done
		progressState = Done;

		owner->commandAI->SlowUpdate();

		LOG_L(L_DEBUG, "[%s] unit %i arrived", __func__, owner->id);
	}
}

/*
Makes the unit fail this action.
No more trials will be done before a new goal is given.
*/
void CGroundMoveType::Fail(bool callScript)
{
	RECOIL_DETAILED_TRACY_ZONE;
	assert(!ThreadPool::inMultiThreadedSection);
	LOG_L(L_DEBUG, "[%s] unit %i failed", __func__, owner->id);

	StopEngine(callScript);

	// failure of finding a path means that
	// this action has failed to reach its goal.
	progressState = Failed;

	eventHandler.UnitMoveFailed(owner);
	eoh->UnitMoveFailed(*owner);
}




void CGroundMoveType::HandleObjectCollisions()
{
	RECOIL_DETAILED_TRACY_ZONE;
	CUnit* collider = owner;
	auto curThread = ThreadPool::GetThreadNum();

	resultantForces = ZeroVector;

	// handle collisions for even-numbered objects on even-numbered frames and vv.
	// (temporal resolution is still high enough to not compromise accuracy much?)
	if (collider->beingBuilt)
		return;

	const UnitDef* colliderUD = collider->unitDef;
	const MoveDef* colliderMD = collider->moveDef;

	resultantForces = ZeroVector;
	forceFromMovingCollidees = ZeroVector;
	forceFromStaticCollidees = ZeroVector;

	// NOTE:
	//   use the collider's MoveDef footprint as radius since it is
	//   always mobile (its UnitDef footprint size may be different)
	const float colliderFootPrintRadius = colliderMD->CalcFootPrintMaxInteriorRadius();
	const float colliderAxisStretchFact = colliderMD->CalcFootPrintAxisStretchFactor();

	HandleUnitCollisions(collider, {collider->speed.w, colliderFootPrintRadius, colliderAxisStretchFact}, colliderUD, colliderMD, curThread);
	HandleFeatureCollisions(collider, {collider->speed.w, colliderFootPrintRadius, colliderAxisStretchFact}, colliderUD, colliderMD, curThread);

	if (forceStaticObjectCheck) {
		MoveTypes::CheckCollisionQuery colliderInfo(collider);
		positionStuck |= !colliderMD->TestMoveSquare(colliderInfo, owner->pos, owner->speed, true, false, true, nullptr, nullptr, curThread);
		positionStuck |= !colliderMD->TestMoveSquare(colliderInfo, owner->pos, owner->speed, false, true, false, nullptr, nullptr, curThread);
		forceStaticObjectCheck = false;
	}

	// auto canAssignForce = [colliderMD, collider, curThread](const float3& force) {
	// 	if (force.same(ZeroVector))
	// 		return false;

	// 	float3 pos = collider->pos + force;
	// 	return colliderMD->TestMoveSquare(collider, pos, force, true, false, true, nullptr, nullptr, curThread)
	// 			&& colliderMD->TestMoveSquare(collider, pos, force, false, true, false, nullptr, nullptr, curThread);
	// };

	// Try to apply all collision forces, but if that will collide with static parts of the map,
	// then only apply forces from static objects/terrain. This prevent units from pushing each
	// other into buildings far enough that the pathing systems can't get them out again.
	float3 tryForce = forceFromStaticCollidees + forceFromMovingCollidees;
	float maxPushForceSq = maxSpeed*maxSpeed*modInfo.maxCollisionPushMultiplier;
	if (tryForce.SqLength() > maxPushForceSq)
		(tryForce.Normalize()) *= maxSpeed;

	UpdatePos(owner, tryForce, resultantForces, curThread);

	if (resultantForces.same(ZeroVector) && positionStuck){
		resultantForces = forceFromStaticCollidees;
		if (resultantForces.SqLength() > maxPushForceSq)
			(resultantForces.Normalize()) *= maxSpeed;
	}
}

bool CGroundMoveType::HandleStaticObjectCollision(
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
) {
	RECOIL_DETAILED_TRACY_ZONE;
	// while being built, units that overlap their factory yardmap should not be moved at all
	assert(!collider->beingBuilt);

	if (checkTerrain && (!collider->IsMoving() || collider->IsInAir()))
		return false;

	// for factories, check if collidee's position is behind us (which means we are likely exiting)
	//
	// NOTE:
	//   allow units to move _through_ idle open factories by extending the collidee's footprint such
	//   that insideYardMap is true in a larger area (otherwise pathfinder and coldet would disagree)
	//   the transition from radius- to footprint-based handling is discontinuous --> cannot mix them
	// TODO:
	//   increase cost of squares inside open factories so PFS is less likely to path through them
	//
	#if 0
	const int xext = ((collidee->xsize >> 1) + std::max(1, colliderMD->xsizeh));
	const int zext = ((collidee->zsize >> 1) + std::max(1, colliderMD->zsizeh));

	const bool insideYardMap =
		(collider->pos.x >= (collidee->pos.x - xext * SQUARE_SIZE)) &&
		(collider->pos.x <= (collidee->pos.x + xext * SQUARE_SIZE)) &&
		(collider->pos.z >= (collidee->pos.z - zext * SQUARE_SIZE)) &&
		(collider->pos.z <= (collidee->pos.z + zext * SQUARE_SIZE));
	const bool exitingYardMap =
		((collider->frontdir.dot(separationVector) > 0.0f) &&
		 (collider->   speed.dot(separationVector) > 0.0f));
	#endif

	const float3& pos = collider->pos;
	const float3& vel = collider->speed;
	const float3& rgt = collider->rightdir;

	// RHS magic constant is the radius of a square (sqrt(2*(SQUARE_SIZE>>1)*(SQUARE_SIZE>>1)))
	constexpr float squareRadius = 5.656854249492381f;

	float3 strafeVec;
	float3 bounceVec;
	float3 summedVec;

	if (checkYardMap || checkTerrain) {
		float3 sqrSumPosition; // .y is always 0
		float2 sqrPenDistance; // .x = sum, .y = count

		float intersectDistance = 0.f;
		float intersectSqrCount = 0.f;
		float3 intersectSqrSumPosition;

		const float3 rightDir2D = (rgt * XZVector).SafeNormalize();
		const float3 speedDir2D = (vel * XZVector).SafeNormalize();

		const int xmid = (pos.x + vel.x) / SQUARE_SIZE;
		const int zmid = (pos.z + vel.z) / SQUARE_SIZE;

		const int xsquare = (pos.x / SQUARE_SIZE);
		const int zsquare = (pos.z / SQUARE_SIZE);

		const int realMinX = xsquare + (-colliderMD->xsizeh);
		const int realMinZ = zsquare + (-colliderMD->zsizeh);
		const int realMaxX = xsquare +   colliderMD->xsizeh ;
		const int realMaxZ = zsquare +   colliderMD->zsizeh ;

		// mantis{3614,4217}
		//   we cannot nicely bounce off terrain when checking only the center square
		//   however, testing more squares means CD can (sometimes) disagree with PFS
		//   in narrow passages --> still possible, but have to ensure we allow only
		//   lateral (non-obstructing) bounces
		const int xsh = colliderMD->xsizeh * (checkYardMap || (checkTerrain && colliderMD->allowTerrainCollisions));
		const int zsh = colliderMD->zsizeh * (checkYardMap || (checkTerrain && colliderMD->allowTerrainCollisions));
		const int intersectSize = colliderMD->xsize;

		const int xmin = std::min(-1, -xsh), xmax = std::max(1, xsh);
		const int zmin = std::min(-1, -zsh), zmax = std::max(1, zsh);

		if (DEBUG_DRAWING_ENABLED){
			geometryLock.lock();
			geometricObjects->AddLine(pos + (UpVector * 25.0f), pos + (UpVector * 100.0f), 3, 1, 4);
			geometryLock.unlock();
		}

		MoveTypes::CheckCollisionQuery colliderInfo(owner);

		// check for blocked squares inside collider's MoveDef footprint zone
		// interpret each square as a "collidee" and sum up separation vectors
		//
		// NOTE:
		//   assumes the collider's footprint is still always axis-aligned
		// NOTE:
		//   the pathfinders only care about the CENTER square wrt terrain!
		//   this means paths can come closer to impassable terrain than is
		//   allowed by collision detection (more probable if edges between
		//   passable and impassable areas are hard instead of gradients or
		//   if a unit is not affected by slopes) --> can be solved through
		//   smoothing the cost-function, eg. blurring heightmap before PFS
		//   sees it
		//
		for (int z = zmin; z <= zmax; z++) {
			for (int x = xmin; x <= xmax; x++) {
				const int xabs = xmid + x;
				const int zabs = zmid + z;

				if ( checkTerrain &&  (CMoveMath::GetPosSpeedMod(*colliderMD, xabs, zabs) > 0.f))
					continue;
				if ( checkYardMap && ((CMoveMath::SquareIsBlocked(*colliderMD, xabs, zabs, &colliderInfo) & CMoveMath::BLOCK_STRUCTURE) == 0))
					continue;

				const float3 squarePos = float3(xabs * SQUARE_SIZE + (SQUARE_SIZE >> 1), pos.y, zabs * SQUARE_SIZE + (SQUARE_SIZE >> 1));
				const float3 squareVec = pos - squarePos;


				const float  squareColRadiusSum = colliderRadius + squareRadius;
				const float   squareSepDistance = squareVec.Length2D() + 0.1f;
				const float   squarePenDistance = std::min(squareSepDistance - squareColRadiusSum, 0.0f);
				// const float  squareColSlideSign = -Sign(squarePos.dot(rightDir2D) - pos.dot(rightDir2D));

				if (x >= realMinX && x <= realMaxX && z >= realMinZ && z <= realMaxZ){
					if (intersectSize > 1) {
						intersectDistance = std::min(intersectDistance, squarePenDistance);
						intersectSqrSumPosition += (squarePos * XZVector);
						intersectSqrCount++;
					}
					if (checkYardMap && !positionStuck)
						positionStuck = true;
				}

				// ignore squares behind us (relative to velocity vector)
				if (squareVec.dot(vel) > 0.0f)
					continue;

				// this tends to cancel out too much on average
				// strafeVec += (rightDir2D * sqColSlideSign);
				bounceVec += (rightDir2D * (rightDir2D.dot(squareVec / squareSepDistance)));

				sqrPenDistance += float2(squarePenDistance, 1.0f);
				sqrSumPosition += (squarePos * XZVector);
			}
		}

		// This pushes units directly away from static objects so they don't intersect.
		if (intersectSqrCount > 0.f) {
			intersectSqrSumPosition *= (1.0f / intersectSqrCount);
			const float pushSpeed = std::min(-intersectDistance, maxSpeed);
			const float3 pushOutVec = ((pos - intersectSqrSumPosition) * XZVector).SafeNormalize() * pushSpeed;

			forceFromStaticCollidees += pushOutVec;
		}

		// This directs units to left/right around the static object.
		if (sqrPenDistance.y > 0.0f /*&& -deepestPenDistance <= squareRadius*2*/) {
			sqrSumPosition *= (1.0f / sqrPenDistance.y);
			sqrPenDistance *= (1.0f / sqrPenDistance.y);

			const float strafeSign = -Sign(sqrSumPosition.dot(rightDir2D) - pos.dot(rightDir2D));
			const float bounceSign =  Sign(rightDir2D.dot(bounceVec));
			const float strafeScale = std::min(std::max(currentSpeed*0.0f, maxSpeedDef), std::max(0.1f, -sqrPenDistance.x * 0.5f));
			const float bounceScale = std::min(std::max(currentSpeed*0.0f, maxSpeedDef), std::max(0.1f, -sqrPenDistance.x * 0.5f));

			// in FPS mode, normalize {strafe,bounce}Scale and multiply by maxSpeedDef
			// (otherwise it would be possible to slide along map edges at above-normal
			// speeds, etc.)
			const float fpsStrafeScale = (strafeScale / (strafeScale + bounceScale)) * maxSpeedDef;
			const float fpsBounceScale = (bounceScale / (strafeScale + bounceScale)) * maxSpeedDef;

			// bounceVec always points along rightDir by construction
			strafeVec = (rightDir2D * strafeSign) * mix(strafeScale, fpsStrafeScale, owner->UnderFirstPersonControl());
			bounceVec = (rightDir2D * bounceSign) * mix(bounceScale, fpsBounceScale, owner->UnderFirstPersonControl());
			summedVec = strafeVec + bounceVec;
			forceFromStaticCollidees += summedVec;
			limitSpeedForTurning = 2;
		}

		// note:
		//   in many cases this does not mean we should request a new path
		//   (and it can be counter-productive to do so since we might not
		//   even *get* one)
		return (canRequestPath && (summedVec != ZeroVector));
	}

	{
		const float  colRadiusSum = colliderRadius + collideeRadius;
		const float   sepDistance = separationVector.Length() + 0.1f;
		const float   penDistance = std::min(sepDistance - colRadiusSum, 0.0f);
		const float  colSlideSign = -Sign(collidee->pos.dot(rgt) - pos.dot(rgt));

		const float strafeScale = std::min(currentSpeed, std::max(0.0f, -penDistance * 0.5f)) * (1 - checkYardMap * false);
		const float bounceScale = std::min(currentSpeed, std::max(0.0f, -penDistance       )) * (1 - checkYardMap *  true);

		strafeVec = (             rgt * colSlideSign) * strafeScale;
		bounceVec = (separationVector /  sepDistance) * bounceScale;
		summedVec = strafeVec + bounceVec;
		forceFromStaticCollidees += summedVec;
		limitSpeedForTurning = 2;

		// same here
		return (canRequestPath && (penDistance < 0.0f));
	}
}

void CGroundMoveType::HandleUnitCollisions(
	CUnit* collider,
	const float3& colliderParams, // .x := speed, .y := radius, .z := fpstretch
	const UnitDef* colliderUD,
	const MoveDef* colliderMD,
	int curThread
) {
	RECOIL_DETAILED_TRACY_ZONE;
	// NOTE: probably too large for most units (eg. causes tree falling animations to be skipped)
	// const float3 crushImpulse = collider->speed * collider->mass * Sign(int(!reversing));

	const bool allowUCO = modInfo.allowUnitCollisionOverlap;
	const bool allowCAU = modInfo.allowCrushingAlliedUnits;
	const bool allowPEU = modInfo.allowPushingEnemyUnits;
	const bool allowSAT = modInfo.allowSepAxisCollisionTest;
	const bool forceSAT = (colliderParams.z > 0.1f);

	const float3 crushImpulse = owner->speed * owner->mass * Sign(int(!reversing));

	// Push resistant units when stopped impacting pathing and also cannot be pushed, so it is important that such
	// units are not going to prevent other units from moving around them if they are near narrow pathways.
	const float colliderSeparationDist = (pushResistant && pushResistanceBlockActive) ? 0.f : colliderUD->separationDistance;

	// Account for units that are larger than one's self.
	const float maxCollisionRadius = colliderParams.y + moveDefHandler.GetLargestFootPrintSizeH();
	const float searchRadius = colliderParams.x + maxCollisionRadius + colliderSeparationDist;

	MoveTypes::CheckCollisionQuery colliderInfo(collider);
	if ( !colliderMD->overrideUnitWaterline )
		colliderInfo.DisableHeightChecks();

	// copy on purpose, since the below can call Lua
	QuadFieldQuery qfQuery;
	qfQuery.threadOwner = curThread;
	quadField.GetUnitsExact(qfQuery, collider->pos, searchRadius);

	for (CUnit* collidee: *qfQuery.units) {
		if (collidee == collider) continue;
		if (collidee->IsSkidding()) continue;
		if (collidee->IsFlying()) continue;

		const UnitDef* collideeUD = collidee->unitDef;
		const MoveDef* collideeMD = collidee->moveDef;

		const bool colliderMobile = (colliderMD != nullptr); // always true
		const bool collideeMobile = (collideeMD != nullptr); // maybe true

		const bool unloadingCollidee = (collidee->unloadingTransportId == collider->id);
		const bool unloadingCollider = (collider->unloadingTransportId == collidee->id);

		if (unloadingCollider)
			collider->requestRemoveUnloadTransportId = true;

		// don't push/crush either party if the collidee does not block the collider (or vv.)
		if (colliderMobile && CMoveMath::IsNonBlocking(collidee, &colliderInfo))
			continue;

		// disable collisions between collider and collidee
		// if collidee is currently inside any transporter,
		// or if collider is being transported by collidee
		if (collider->GetTransporter() == collidee) continue;
		if (collidee->GetTransporter() != nullptr) continue;
		// also disable collisions if either party currently
		// has an order to load units (TODO: do we want this
		// for unloading as well?)
		if (collider->loadingTransportId == collidee->id) continue;
		if (collidee->loadingTransportId == collider->id) continue;

		const float collDist = (collideeMobile) ? collideeMD->CalcFootPrintMaxInteriorRadius() : collidee->CalcFootPrintMaxInteriorRadius();
		const float2 collideeParams = {collidee->speed.w, collDist};
		const float4 separationVect = {collider->pos - collidee->pos, Square(colliderParams.y + collideeParams.y)};

		const int collisionFunc = (allowSAT && (forceSAT || (collideeMobile && collideeMD->CalcFootPrintAxisStretchFactor() > 0.1f)));
		const bool isCollision = (checkCollisionFuncs[collisionFunc](separationVect, collider, collidee, colliderMD, collideeMD));

		// check for separation
		float separationDist = 0.f;
		if (!isCollision && collideeMobile) {
			const bool useCollideeSeparationDistance = !( collidee->moveType->IsPushResistant() && collidee->moveType->IsPushResitanceBlockActive() );
			const float collideeSeparationDist = (useCollideeSeparationDistance) ? colliderUD->separationDistance : 0.f;
			separationDist = std::max(colliderSeparationDist, collideeUD->separationDistance);
			const float separation = colliderParams.y + collideeParams.y + separationDist; 
			const bool isSeparation = static_cast<float3>(separationVect).SqLength2D() <= Square(separation);
			if (!isSeparation)
				continue;
		} else if (!isCollision)
			continue;

		if (unloadingCollider) {
			collider->requestRemoveUnloadTransportId = false;
			continue;
		}

		if (unloadingCollidee)
			continue;

		// NOTE:
		//   we exclude aircraft (which have NULL moveDef's) landed
		//   on the ground, since they would just stack when pushed
		bool pushCollider = colliderMobile;
		bool pushCollidee = collideeMobile;

		const bool alliedCollision =
			teamHandler.Ally(collider->allyteam, collidee->allyteam) &&
			teamHandler.Ally(collidee->allyteam, collider->allyteam);

		if (isCollision) {
			bool crushCollidee = false;

			// const bool collideeYields = (collider->IsMoving() && !collidee->IsMoving());
			// const bool ignoreCollidee = (collideeYields && alliedCollision);

			crushCollidee |= (!alliedCollision || allowCAU);
			crushCollidee &= ((colliderParams.x * collider->mass) > (collideeParams.x * collidee->mass));

			if (crushCollidee && !CMoveMath::CrushResistant(*colliderMD, collidee)) {
				auto& events = Sim::registry.get<UnitCrushEvents>(owner->entityReference);
				events.value.emplace_back(collider, collidee, crushImpulse);
			}

			// Only trigger this event once for each colliding pair of units.
			if (collider->id < collidee->id){
				auto& events = Sim::registry.get<UnitCollisionEvents>(owner->entityReference);
				events.value.emplace_back(collider, collidee);
			}
		}

		if (collideeMobile)
			HandleUnitCollisionsAux(collider, collidee, this, static_cast<CGroundMoveType*>(collidee->moveType));

		// NOTE:
		//   allowPushingEnemyUnits is (now) useless because alliances are bi-directional
		//   ie. if !alliedCollision, pushCollider and pushCollidee BOTH become false and
		//   the collision is treated normally --> not what we want here, but the desired
		//   behavior (making each party stop and block the other) has many corner-cases
		//   so instead have collider respond as though collidee is semi-static obstacle
		//   this also happens when both parties are pushResistant
		pushCollider = pushCollider && (alliedCollision || allowPEU || !collider->blockEnemyPushing);
		pushCollidee = pushCollidee && (alliedCollision || allowPEU || !collidee->blockEnemyPushing);
		pushCollider = pushCollider && (!collider->beingBuilt && !collider->UsingScriptMoveType() && !collider->moveType->IsPushResistant());
		pushCollidee = pushCollidee && (!collidee->beingBuilt && !collidee->UsingScriptMoveType() && !collidee->moveType->IsPushResistant());

		const bool isStatic = (!collideeMobile && !collideeUD->IsAirUnit()) || (!pushCollider && !pushCollidee);
		if (isCollision && isStatic) {
			// building (always axis-aligned, possibly has a yardmap)
			// or semi-static collidee that should be handled as such
			//
			// since all push-resistant units use the BLOCK_STRUCTURE
			// mask when stopped, avoid the yardmap || terrain branch
			// of HSOC which is not well suited to both parties moving
			// and can leave them inside stuck each other's footprints
			const bool allowNewPath = (!atEndOfPath && !atGoal);
			const bool checkYardMap = ((pushCollider || pushCollidee) || collideeUD->IsFactoryUnit());

			if (HandleStaticObjectCollision(collider, collidee, colliderMD,  colliderParams.y, collideeParams.y,  separationVect, allowNewPath, checkYardMap, false, curThread)) {
				ReRequestPath(false);
			}

			continue;
		}

		const bool moveCollider = ((pushCollider || !pushCollidee) && colliderMobile);
		if (moveCollider) {
			if (isCollision)
				forceFromMovingCollidees += CalculatePushVector(colliderParams, collideeParams, allowUCO, separationVect, collider, collidee);
			else {
				// push units away from each other though they are not colliding.
				const float3 colliderParams2 = {colliderParams.x, colliderParams.y + separationDist * 0.5f, colliderParams.z};
				const float2 collideeParams2 = {collidee->speed.w, collDist + separationDist * 0.5f};
				const float4 separationVect2 = {static_cast<float3>(separationVect), Square(colliderParams.y + collideeParams.y)};
				forceFromMovingCollidees += CalculatePushVector(colliderParams2, collideeParams2, allowUCO, separationVect2, collider, collidee);
			}
		}
	}
}

float3 CGroundMoveType::CalculatePushVector(const float3 &colliderParams, const float2 &collideeParams, const bool allowUCO, const float4 &separationVect, CUnit *collider, CUnit *collidee)
{
    const float colliderRelRadius = colliderParams.y / (colliderParams.y + collideeParams.y);
    const float collideeRelRadius = collideeParams.y / (colliderParams.y + collideeParams.y);
    const float collisionRadiusSum = allowUCO ? (colliderParams.y * colliderRelRadius + collideeParams.y * collideeRelRadius) : (colliderParams.y + collideeParams.y);

    const float sepDistance = separationVect.Length() + 0.1f;
    const float penDistance = std::max(collisionRadiusSum - sepDistance, 1.0f);
    const float sepResponse = std::min(SQUARE_SIZE * 2.0f, penDistance * 0.5f);

    const float3 sepDirection = separationVect / sepDistance;
    const float3 colResponseVec = sepDirection * XZVector * sepResponse;

    const float
        m1 = collider->mass,
        m2 = collidee->mass,
        v1 = std::max(1.0f, colliderParams.x),
        v2 = std::max(1.0f, collideeParams.x),
        c1 = 1.0f + (1.0f - math::fabs(collider->frontdir.dot(-sepDirection))) * 5.0f,
        c2 = 1.0f + (1.0f - math::fabs(collidee->frontdir.dot(sepDirection))) * 5.0f,
        // weighted momenta
        s1 = m1 * v1 * c1,
        s2 = m2 * v2 * c2,
        // relative momenta
        r1 = s1 / (s1 + s2 + 1.0f),
        r2 = s2 / (s1 + s2 + 1.0f);

    // far from a realistic treatment, but works
    const float colliderMassScale = std::clamp(1.0f - r1, 0.01f, 0.99f) * (allowUCO ? (1.0f / colliderRelRadius) : 1.0f);
    // const float collideeMassScale = std::clamp(1.0f - r2, 0.01f, 0.99f) * (allowUCO? (1.0f / collideeRelRadius): 1.0f);

    // try to prevent both parties from being pushed onto non-traversable
    // squares (without resetting their position which stops them dead in
    // their tracks and undoes previous legitimate pushes made this frame)
    //
    // if pushCollider and pushCollidee are both false (eg. if each party
    // is pushResistant), treat the collision as regular and push both to
    // avoid deadlocks
    const float colliderSlideSign = Sign(separationVect.dot(collider->rightdir));

    const float3 colliderPushVec = colResponseVec * colliderMassScale; // * int(!ignoreCollidee);
    const float3 colliderSlideVec = collider->rightdir * colliderSlideSign * (1.0f / penDistance) * r2;
    const float3 colliderMoveVec = colliderPushVec + colliderSlideVec;

    return colliderMoveVec;
}

void CGroundMoveType::HandleFeatureCollisions(
	CUnit* collider,
	const float3& colliderParams,
	const UnitDef* colliderUD,
	const MoveDef* colliderMD,
	int curThread
) {
	RECOIL_DETAILED_TRACY_ZONE;
	const bool allowSAT = modInfo.allowSepAxisCollisionTest;
	const bool forceSAT = (colliderParams.z > 0.1f);

	const float3 crushImpulse = owner->speed * owner->mass * Sign(int(!reversing));
	MoveTypes::CheckCollisionQuery colliderInfo(collider);

	// copy on purpose, since DoDamage below can call Lua
	QuadFieldQuery qfQuery;
	qfQuery.threadOwner = curThread;
	quadField.GetFeaturesExact(qfQuery, collider->pos, colliderParams.x + (colliderParams.y * 2.0f));

	for (CFeature* collidee: *qfQuery.features) {
		// const FeatureDef* collideeFD = collidee->def;

		// use the collidee's Feature (not FeatureDef) footprint as radius
		const float2 collideeParams = {0.0f, collidee->CalcFootPrintMaxInteriorRadius()};
		const float4 separationVect = {collider->pos - collidee->pos, Square(colliderParams.y + collideeParams.y)};

		if (!checkCollisionFuncs[allowSAT && (forceSAT || (collidee->CalcFootPrintAxisStretchFactor() > 0.1f))](separationVect, collider, collidee, colliderMD, nullptr))
			continue;


		if (CMoveMath::IsNonBlocking(collidee, &colliderInfo))
			continue;

		if (!CMoveMath::CrushResistant(*colliderMD, collidee)){
			auto& events = Sim::registry.get<FeatureCrushEvents>(owner->entityReference);
			events.value.emplace_back(collider, collidee, crushImpulse);
		}
		#if 0
		if (pathController.IgnoreCollision(collider, collidee))
			continue;
		#endif

		{
			auto& events = Sim::registry.get<FeatureCollisionEvents>(owner->entityReference);
			events.value.emplace_back(collider, collidee);
		}

		if (!collidee->IsMoving()) {
			if (HandleStaticObjectCollision(collider, collidee, colliderMD,  colliderParams.y, collideeParams.y,  separationVect, (!atEndOfPath && !atGoal), true, false, curThread)) {
				ReRequestPath(false);
			}

			continue;
		}

		const float  sepDistance    = separationVect.Length() + 0.1f;
		const float  penDistance    = std::max((colliderParams.y + collideeParams.y) - sepDistance, 1.0f);
		const float  sepResponse    = std::min(SQUARE_SIZE * 2.0f, penDistance * 0.5f);

		const float3 sepDirection   = separationVect / sepDistance;
		const float3 colResponseVec = sepDirection * XZVector * sepResponse;

		// multiply the collidee's mass by a large constant (so that heavy
		// features do not bounce light units away like jittering pinballs;
		// collideeMassScale ~= 0.01 suppresses large responses)
		const float
			m1 = collider->mass,
			m2 = collidee->mass * 10000.0f,
			v1 = std::max(1.0f, colliderParams.x),
			v2 = 1.0f,
			c1 = (1.0f - math::fabs( collider->frontdir.dot(-sepDirection))) * 5.0f,
			c2 = (1.0f - math::fabs(-collider->frontdir.dot( sepDirection))) * 5.0f,
			s1 = m1 * v1 * c1,
			s2 = m2 * v2 * c2,
 			r1 = s1 / (s1 + s2 + 1.0f),
 			r2 = s2 / (s1 + s2 + 1.0f);

		const float colliderMassScale = std::clamp(1.0f - r1, 0.01f, 0.99f);
		const float collideeMassScale = std::clamp(1.0f - r2, 0.01f, 0.99f);

		forceFromMovingCollidees += colResponseVec * colliderMassScale;

		{
			auto& events = Sim::registry.get<FeatureMoveEvents>(owner->entityReference);
			events.value.emplace_back(collider, collidee, -colResponseVec * collideeMassScale);
		}
	}
}



void CGroundMoveType::LeaveTransport()
{
	RECOIL_DETAILED_TRACY_ZONE;
	oldPos = owner->pos + UpVector * 0.001f;
}

void CGroundMoveType::Connect() {
	RECOIL_DETAILED_TRACY_ZONE;
	Sim::registry.emplace_or_replace<GroundMoveType>(owner->entityReference, owner->id);
	Sim::registry.emplace_or_replace<FeatureCollisionEvents>(owner->entityReference);
	Sim::registry.emplace_or_replace<UnitCollisionEvents>(owner->entityReference);
	Sim::registry.emplace_or_replace<FeatureCrushEvents>(owner->entityReference);
	Sim::registry.emplace_or_replace<UnitCrushEvents>(owner->entityReference);
	Sim::registry.emplace_or_replace<FeatureMoveEvents>(owner->entityReference);
	Sim::registry.emplace_or_replace<UnitMovedEvent>(owner->entityReference);
	Sim::registry.emplace_or_replace<ChangeHeadingEvent>(owner->entityReference, owner->id);
	Sim::registry.emplace_or_replace<ChangeMainHeadingEvent>(owner->entityReference, owner->id);
	// LOG("%s: loading %s as %d", __func__, owner->unitDef->name.c_str(), entt::to_integral(owner->entityReference));
}

void CGroundMoveType::Disconnect() {
	RECOIL_DETAILED_TRACY_ZONE;
	Sim::registry.remove<GroundMoveType>(owner->entityReference);
	Sim::registry.remove<FeatureCollisionEvents>(owner->entityReference);
	Sim::registry.remove<UnitCollisionEvents>(owner->entityReference);
	Sim::registry.remove<FeatureCrushEvents>(owner->entityReference);
	Sim::registry.remove<UnitCrushEvents>(owner->entityReference);
	Sim::registry.remove<FeatureMoveEvents>(owner->entityReference);
	Sim::registry.remove<UnitMovedEvent>(owner->entityReference);
	Sim::registry.remove<ChangeHeadingEvent>(owner->entityReference);
	Sim::registry.remove<ChangeMainHeadingEvent>(owner->entityReference);
}

void CGroundMoveType::KeepPointingTo(CUnit* unit, float distance, bool aggressive) { KeepPointingTo(unit->pos, distance, aggressive); }
void CGroundMoveType::KeepPointingTo(float3 pos, float distance, bool aggressive) {
	mainHeadingPos = pos;
	useMainHeading = aggressive;

	if (!useMainHeading)
		return;
	if (owner->weapons.empty())
		return;

	const CWeapon* frontWeapon = owner->weapons.front();

	if (!frontWeapon->weaponDef->waterweapon)
		mainHeadingPos.y = std::max(mainHeadingPos.y, 0.0f);

	float3 dir1 = frontWeapon->mainDir;
	float3 dir2 = mainHeadingPos - owner->pos;

	// in this case aligning is impossible
	if (dir1 == UpVector)
		return;

	dir1 = (dir1 * XZVector).SafeNormalize();
	dir2 = (dir2 * XZVector).SafeNormalize();

	if (dir2 == ZeroVector)
		return;

	const short heading =
		GetHeadingFromVector(dir2.x, dir2.z) -
		GetHeadingFromVector(dir1.x, dir1.z);

	if (owner->heading == heading)
		return;

	// NOTE:
	//   by changing the progress-state here (which seems redundant),
	//   SlowUpdate can suddenly request a new path for us even after
	//   StopMoving (which clears pathID; CAI often calls StopMoving
	//   before unit is at goalPos!)
	//   for this reason StopMoving always updates goalPos so internal
	//   GetNewPath's are no-ops (while CAI does not call StartMoving)
	if (frontWeapon->TestRange(mainHeadingPos, SWeaponTarget(mainHeadingPos, true)))
		return;

	progressState = Active;
}


/**
* @brief Orients owner so that weapon[0]'s arc includes mainHeadingPos
*/
void CGroundMoveType::SetMainHeading() {
	if (!useMainHeading || owner->weapons.empty()) {
		ChangeHeading(owner->heading);
		return;
	}

	const CWeapon* frontWeapon = owner->weapons.front();

	const float3 dir1 = ((       frontWeapon->mainDir) * XZVector).SafeNormalize();
	const float3 dir2 = ((mainHeadingPos - owner->pos) * XZVector).SafeNormalize();

	// ASSERT_SYNCED(dir1);
	// ASSERT_SYNCED(dir2);

	if (dir2 == ZeroVector)
		return;

	short newHeading =
		GetHeadingFromVector(dir2.x, dir2.z) -
		GetHeadingFromVector(dir1.x, dir1.z);

	// ASSERT_SYNCED(newHeading);

	if (progressState == Active) {
		if (owner->heading != newHeading) {
			// start or continue turning
			ChangeHeading(newHeading);
		} else {
			// stop turning
			progressState = Done;
		}

		return;
	}

	if (owner->heading == newHeading)
		return;

	if (frontWeapon->TestRange(mainHeadingPos, SWeaponTarget(mainHeadingPos, true)))
		return;

	progressState = Active;
}

bool CGroundMoveType::OnSlope(float minSlideTolerance) {
	RECOIL_DETAILED_TRACY_ZONE;
	const UnitDef* ud = owner->unitDef;
	const MoveDef* md = owner->moveDef;
	const float3& pos = owner->pos;

	if (ud->slideTolerance < minSlideTolerance)
		return false;
	if (owner->FloatOnWater() && owner->IsInWater())
		return false;
	if (!pos.IsInBounds())
		return false;

	// if minSlideTolerance is LEQ 0, do not multiply maxSlope by ud->slideTolerance
	// (otherwise the unit could stop on an invalid path location, and be teleported
	// back)
	const float slopeMul = mix(ud->slideTolerance, 1.0f, (minSlideTolerance <= 0.0f));
	const float curSlope = CGround::GetSlope(pos.x, pos.z);
	const float maxSlope = md->maxSlope * slopeMul;

	return (curSlope > maxSlope);
}



const float3& CGroundMoveType::GetGroundNormal(const float3& p) const
{
	RECOIL_DETAILED_TRACY_ZONE;
	// ship or hovercraft; return (CGround::GetNormalAboveWater(p));
	if (owner->IsInWater() && !owner->IsOnGround())
		return UpVector;

	return (CGround::GetNormal(p.x, p.z));
}

float CGroundMoveType::GetGroundHeight(const float3& p) const
{
	RECOIL_DETAILED_TRACY_ZONE;
	MoveDef *md = owner->moveDef;

	// in [minHeight, maxHeight]
	const float gh = CGround::GetHeightReal(p.x, p.z);
	
	// in [-waterline, maxHeight], note that waterline
	// can be much deeper than ground in shallow water
	if (owner->FloatOnWater()) {
		MoveDef *md = owner->moveDef;
		const float wh = ((md->overrideUnitWaterline) ? -md->waterline : -owner->unitDef->waterline) * (gh <= 0.0f);

		return (std::max(gh, wh));
	}

	return gh;
}

void CGroundMoveType::AdjustPosToWaterLine()
{
	RECOIL_DETAILED_TRACY_ZONE;
	if (owner->IsFalling())
		return;
	if (owner->IsFlying())
		return;

	if (modInfo.allowGroundUnitGravity) {
		if (owner->FloatOnWater()) {
			MoveDef *md = owner->moveDef;
			owner->Move(UpVector * (std::max(CGround::GetHeightReal(owner->pos.x, owner->pos.z), -md->waterline) - owner->pos.y), true);
		} else {
			owner->Move(UpVector * (std::max(CGround::GetHeightReal(owner->pos.x, owner->pos.z), owner->pos.y) - owner->pos.y), true);
		}
	} else {
		owner->Move(UpVector * (GetGroundHeight(owner->pos) - owner->pos.y), true);
	}
}

bool CGroundMoveType::UpdateDirectControl()
{
	RECOIL_DETAILED_TRACY_ZONE;
	const CPlayer* myPlayer = gu->GetMyPlayer();
	const FPSUnitController& selfCon = myPlayer->fpsController;
	const FPSUnitController& unitCon = owner->fpsControlPlayer->fpsController;
	const bool wantReverse = (unitCon.back && !unitCon.forward);

	float turnSign = 0.0f;

	currWayPoint = owner->frontdir * XZVector * mix(100.0f, -100.0f, wantReverse);
	earlyCurrWayPoint = currWayPoint = (owner->pos + currWayPoint).cClampInBounds();

	if (unitCon.forward || unitCon.back) {
		ChangeSpeed((maxSpeed * unitCon.forward) + (maxReverseSpeed * unitCon.back), wantReverse, true);
	} else {
		// not moving forward or backward, stop
		ChangeSpeed(0.0f, false, true);
	}

	if (unitCon.left ) { ChangeHeading(owner->heading + turnRate); turnSign =  1.0f; }
	if (unitCon.right) { ChangeHeading(owner->heading - turnRate); turnSign = -1.0f; }

	// local client is controlling us
	if (selfCon.GetControllee() == owner)
		camera->SetRotY(camera->GetRot().y + turnRate * turnSign * TAANG2RAD);

	return wantReverse;
}

// 这个函数是单位移动的最终物理仲裁者。
// 在它被调用之前，上层逻辑已经计算出了一个“理想的”移动向量 moveDir。
// UpdatePos 的核心职责就是验证这个理想移动是否可行，即检查它是否会导致单位穿过不可通行的地形或静态障碍物（如建筑、岩石）。
// 如果不可行，它会计算出一个修正后的、实际可行的移动向量。
// const CUnit* unit: 指向需要更新位置的单位
// const float3& moveDir: 输入参数，代表本帧理想的移动位移向量
// float3& resultantMove: 输出参数。这个函数会把最终计算出的、实际可行的位移向量写入这个变量
// int thread: 当前的线程ID，用于多线程环境下的缓存
// const: 表示这个函数不会修改 CGroundMoveType 类的成员变量
void CGroundMoveType::UpdatePos(const CUnit* unit, const float3& moveDir, float3& resultantMove, int thread) const {
	RECOIL_DETAILED_TRACY_ZONE;
	// 保存单位在本帧开始时的位置
	const float3 prevPos = unit->pos;
	// 计算出理想情况下，单位移动后应该到达的新位置。
	const float3 newPos = unit->pos + moveDir;
	// 始化输出参数。函数首先做一个乐观的假设：移动是完全可行的。
	// 因此，将输出 resultantMove 初始化为理想的 moveDir。后续的代码如果检测到碰撞，将会修改这个 resultantMove 的值。
	resultantMove = moveDir;

	// The series of tests done here will benefit from using the same cached results.
	MoveDef* md = unit->moveDef;
	// 获取一个线程专属的临时编号。这是一个重要的性能优化。在同一帧内，对于同一个线程，这个编号是唯一的。
	// 后续的碰撞检测函数会用这个编号作为“缓存键”，将对静态障碍物的检测结果缓存起来。
	// 这样，如果多个单位在同一帧内对同一个建筑进行碰撞检测，这个昂贵的计算只需要执行一次。
	int tempNum = gs->GetMtTempNum(thread);
	// 创建一个“碰撞查询对象”。这是一个结构体，它打包了进行碰撞检测所需的所有关于移动单位的信息（如 MoveDef、当前位置、是否在水中等）。
	// 这样做的好处是，后续调用碰撞检测函数时，只需要传递这一个对象，而不需要传递一大串零散的参数，使代码更整洁、高效。
	MoveTypes::CheckCollisionQuery virtualObject(unit);
	// 创建一个“查询状态跟踪器”。这个对象专门用于处理复杂的潜水单位，它会跟踪单位的水下状态变化，并决定是否需要刷新碰撞缓存（即更新 tempNum）。
	MoveDefs::CollisionQueryStateTrack queryState;
	// 检查单位是否是“复杂潜水单位”（即可以潜入水下并有自定义水线的单位）。
	const bool isSubmersible = md->IsComplexSubmersible();
	// 这是一个关键的性能优化。如果单位不是潜水单位（即普通的地面单位），那么在进行水平移动的碰撞检测时，我们不需要考虑它和障碍物在Y轴（高度）上的关系。
	// 调用 DisableHeightChecks() 可以让后续的碰撞检测算法跳过所有与高度相关的计算，从而大幅提升性能。
	if (!isSubmersible)
		virtualObject.DisableHeightChecks();
	// 它的功能非常简单，就是将一个世界坐标 float3 pos 转换为它所在的地图方格的二维整数坐标 int2。
	// 这是通过将X和Z坐标分别除以每个方格的大小 SQUARE_SIZE 并取整来实现的。
	auto toMapSquare = [](float3 pos) {
		return int2({int(pos.x / SQUARE_SIZE), int(pos.z / SQUARE_SIZE)});
	};
	// 它将一个二维的地图方格坐标 int2 square 转换为一个一维的索引ID。
	// 这是典型的将二维数组坐标映射到一维数组索引的计算方法，mapDims.mapx 是地图在X方向上的宽度。
	auto toSquareId = [](int2 square) {
		return (square.y * mapDims.mapx) + square.x;
	};
	// 这个函数的核心任务是回答一个问题：“一个单位移动到 pos 这个位置是否安全？” 它通过执行一系列检查来得出结论。
	auto isSquareOpen = [this, md, unit, &tempNum, thread, &toMapSquare, &virtualObject, &queryState, &isSubmersible](float3 pos) {
		// 边界检查: 首先，它将世界坐标 pos 转换为地图方格坐标 checkSquare，
		// 然后检查这个方格是否在地图边界之内。如果超出了边界，直接返回 false
		int2 checkSquare = toMapSquare(pos);

		if ( checkSquare.x < 0
			|| checkSquare.y < 0
			|| checkSquare.x >= mapDims.mapx
			|| checkSquare.y >= mapDims.mapy) {
				return false;
			}
		// 潜水单位特殊处理: 如果单位是“复杂潜水单位”，情况会更复杂，因为它的碰撞状态取决于它的深度和水的状态。
		if (isSubmersible){
			// 这个函数会根据单位在 checkSquare 的新位置来更新 virtualObject 的内部状态（比如它的Y坐标、是否在水中等）
			md->UpdateCheckCollisionQuery(virtualObject, queryState, checkSquare);
			// 如果 UpdateCheckCollisionQuery 发现单位的垂直状态（如从陆地进入水中）发生了显著变化，它会设置 refreshCollisionCache 标志。
			if (queryState.refreshCollisionCache)
				tempNum = gs->GetMtTempNum(thread);
		}

		// separate calls because terrain is only checked for in the centre square, while
		// static objects are checked for in the whole footprint.
		// 最终的安全检查: 这是最关键的一步，它将地形检查和物体检查结合起来
		// pathController.IgnoreTerrain(*md, pos): 首先检查路径控制器是否允许忽略地形（例如，单位在空中时）。
		// || (或): 如果不忽略地形...
		// unit->moveDef->TestMoveSquare(...): 调用 MoveDef 的函数来只检查地形。参数 true, false, true 
		// 分别表示 testTerrain=true, testObjects=false, centerOnly=true。
		// 这意味着它只检查单位中心点所在的那个方格的地形是否可通过（坡度、地形类型等）。
		// && (与): 只有当地形检查通过时，才会进行物体检查。
		// 这个函数只检查静态物体（建筑、岩石等）。
		// 与地形检查不同，它会检查单位的整个“脚印”（footprint）覆盖的所有方格，确保没有任何一个方格被静态物体阻挡。
		// 它使用了 tempNum 作为缓存键来提高性能。
		bool result = ( pathController.IgnoreTerrain(*md, pos) ||
				 unit->moveDef->TestMoveSquare(virtualObject, pos, (pos - unit->pos), true, false, true, nullptr, nullptr, thread)
			   )
				&& unit->moveDef->TestMovePositionForObjects(&virtualObject, pos, tempNum, thread);
		// return result;: 返回最终的布尔结果。只有当地形和物体检查都通过时，result 才为 true，表示该位置是开放和安全的。
		return result;
	};
	// 它的作用与 toMapSquare 相反，是将一个地图方格坐标 square 转换回一个世界坐标 float3。
	// 它计算出该方格的中心点附近的一个位置（加1是为了避免边界问题），
	// Y坐标设为0。这个函数在后续检查对角线移动时被用来获取相邻方格的世界坐标。
	auto toPosition = [](int2 square) {
		return float3({float(square.x * SQUARE_SIZE + 1), 0.f, float(square.y * SQUARE_SIZE + 1)});
	};
	// 获取单位移动前的位置所在的地图方格坐标。
	const int2 prevSquare = toMapSquare(prevPos);
	// 获取单位理想移动后的位置所在的地图方格坐标。
	const int2 newSquare = toMapSquare(newPos);
	// 将目标方格的二维坐标转换为一维ID。
	const int newPosStartSquare = toSquareId(newSquare);
	// 如果单位没有卡住，并且这次移动没有跨越到新的方格，那么就没有必要进行昂贵的碰撞检测，函数直接返回。这极大地减少了不必要的计算
	if (!positionStuck && toSquareId(prevSquare) == newPosStartSquare) { return; }
	// 对理想的目标位置 newPos 进行第一次初步安全检查。
	// 取反。如果 isSquareOpen 返回 true（开放），那么 isSquareBlocked 就是 false（未被阻挡）
	bool isSquareBlocked = !isSquareOpen(newPos);
	// 只有在初步检查表明目标方格本身是开放的情况下，才需要进入这个代码块，进一步检查移动路径的问题，特别是对角线移动
	if (!isSquareBlocked) {
		// 计算起始方格和目标方格之间的坐标差。
		const int2 fullDiffSquare = newSquare - prevSquare;
		//  这是检测对角线移动的关键。如果X坐标和Z坐标都发生了变化，那就意味着单位正在尝试进行一次对角线移动（例如，从 (5,5) 移动到 (6,6)）。
		if (fullDiffSquare.x != 0 && fullDiffSquare.y != 0) {
			// 接下来的代码是专门为了防止单位“挤”过两个互为对角的障碍物的
			// . . . .
 			// . A # .   A = 单位当前位置 (prevSquare)
  			// . # D .   D = 单位目标位置 (newSquare)
  			// . . . .   # = 障碍物
			// 单位想从A移动到D。虽然A和D本身都是可以通过的空地，
			// 但如果单位直接走对角线，它的身体（碰撞体积）很可能会“蹭”到或穿过那两个障碍物的角。
			// 这段代码就是要阻止这种不合法的移动。
			// (fullDiffSquare.x < 0) 是一个布尔表达式，如果为真（向左移动），结果是1；为假（向右移动），结果是0。
			// 1 - (2 * 1) 等于 -1
			// 1 - (2 * 0) 等于 +1
			// 所以，diffSquare 最终会是一个方向向量，其分量为 (+1, +1), (+1, -1), (-1, +1) 或 (-1, -1)，精确地指明了对角线的方向
			const int2 diffSquare{1 - (2 * (fullDiffSquare.x < 0)), 1 - (2 * (fullDiffSquare.y < 0))};

			// We have a diagonal move. Make sure the unit cannot press through a corner.
			// 计算出形成那个“角”的两个相邻方格的坐标。
			// 回到我们的例子，newSquare 是 D。diffSquare 是 (+1, +1)
			// checkSqrX 就是 (Dx - 1, Dy)，即 D 左边的那个 #
			// checkSqrY 就是 (Dx, Dy - 1)，即 D 下边的那个 #
			const int2 checkSqrX({newSquare.x - diffSquare.x, newSquare.y});
			const int2 checkSqrY({newSquare.x, newSquare.y - diffSquare.y});
			// !isSquareOpen(toPosition(checkSqrX)): 检查X方向的相邻方格是否被阻挡
			// !isSquareOpen(toPosition(checkSqrY)): 检查Y方向的相邻方格是否被阻挡
			// && (与): 只有当两个相邻的方格都同时被阻挡时，这次对角线移动才被最终判定为非法（isSquareBlocked 变为 true）。
			// 如果只有一个被阻挡，单位仍然可以通过“蹭”着一个障碍物移动过去。
			isSquareBlocked = !isSquareOpen(toPosition(checkSqrX)) && !isSquareOpen(toPosition(checkSqrY));
			// 如果最终判定这次移动是被阻挡的（无论是目标点本身被挡，还是对角线穿角被挡)
			if (isSquareBlocked)
				//  将最终的移动向量 resultantMove 设置为零向量。这意味着单位本帧将无法移动，完全被卡住
				resultantMove = ZeroVector;
		}
	}
	else {
	// NOTE:
	//   does not check for structure blockage, coldet handles that
	//   entering of impassable terrain is *also* handled by coldet
	//
	//   the loop below tries to evade "corner" squares that would
	//   block us from initiating motion and is needed for when we
	//   are not *currently* moving but want to get underway to our
	//   first waypoint (HSOC coldet won't help then)
	//
	//   allowing movement through blocked squares when pathID != 0
	//   relies on assumption that PFS will not search if start-sqr
	//   is blocked, so too fragile
	//
	// TODO: look to move as much of this to MT to improve perf.

	// 注意：
	// 不检查结构物阻塞，碰撞检测会处理这一点
	// 进入不可通行地形的情况也由碰撞检测处理
	// 下面的循环尝试避开会阻碍我们启动移动的 "角落" 方格，
	// 这在我们当前没有移动但想要开始向第一个路径点移动时是必要的
	// （此时 HSOC 碰撞检测不会起作用）
	// 当 pathID != 0 时允许通过被阻塞的方格，这依赖于一个假设：
	// 如果起始方格被阻塞，寻路系统 (PFS) 将不会进行搜索，这一假设过于脆弱
	// 待办：考虑将此部分尽可能移至多线程 (MT) 以提升性能
		

		// 初始化一个标志变量 updatePos。如果后续的探测找到了一个可行的移动方案，这个标志就会被设为 true。
		bool updatePos = false;
		// 计算出单位在本帧的理想移动速率（标量）。这个值后面会用来限制“滑动”的距离，防止单位因为侧滑而移动得比它本来的速度还快。
		const float speed = moveDir.Length2D();
		//  const float3& newPos: 单位最初的理想目标点。
		//  float3 posOffset: 探测偏移量。这是最重要的参数，代表了“往左/右平移多少”的向量
		//  float maxDisplacement = 0.f: 最大允许位移，用于限制最终移动距离
		auto tryToMove =
				[this, &isSquareOpen, &prevPos, &newPosStartSquare, &resultantMove]
				(const float3& newPos, float3 posOffset, float maxDisplacement = 0.f)
			{
			// units are moved in relation to their previous position.
			// 计算探测点的位移向量。它不是简单地使用 posOffset，
			// 而是计算从单位当前的位置 prevPos 到**“理想目标点 newPos + 探测偏移量 posOffset”** 的总位移。
			float3 offsetFromPrev = (newPos + posOffset) - prevPos;
			// 如果设置了最大位移限制，并且计算出的探测位移超过了这个限制。
			if ((maxDisplacement > 0.f) && offsetFromPrev.SqLength2D() > (maxDisplacement*maxDisplacement)) {
				// 就将这个位移向量的长度缩减到最大允许值。这确保了单位的“滑动”速度不会超过它本来的前进速度。
				offsetFromPrev.SafeNormalize2D() *= maxDisplacement;
			}
			// 计算出最终要进行安全检查的探测点世界坐标。
			float3 posToTest = prevPos + offsetFromPrev;
			// 计算探测点所在的地图方格ID。
			int curSquare = int(posToTest.z / SQUARE_SIZE)*mapDims.mapx + int(posToTest.x / SQUARE_SIZE);
			// 这是一个优化。如果探测点仍然在最初被判定为阻塞的那个目标方格内，那就没必要再检查了，
			// 肯定还是阻塞的。只有当探测点已经移动到了一个新的方格时，才需要进行检查。
			if (curSquare != newPosStartSquare) {
				// 调用 isSquareOpen 函数，对这个新的探测点进行完整的安全检查（地形+静态物体）。
				bool updatePos = isSquareOpen(posToTest);
				if (updatePos) {
					// 如果检查通过，说明找到了一个安全的“逃逸点”！
					// 将计算出的、安全的滑动位移向量 offsetFromPrev 赋值给函数的最终输出变量 resultantMove。
					resultantMove = offsetFromPrev;
					return true;
				}
			}
			return false;
		};
		// 探测循环
		for (int n = 1; n <= SQUARE_SIZE; n++) {
			// unit->rightdir * n: 计算出一个向单位右侧平移 n 个单位的偏移向量
			updatePos = tryToMove(newPos, unit->rightdir * n);
			if (updatePos) { break; }
			// 计算出一个向单位左侧平移 n 个单位的偏移向量。
			updatePos = tryToMove(newPos, unit->rightdir * -n);
			if (updatePos) { break; }
		}
		// if (!updatePos): 检查“贴墙滑动”探测是否失败。updatePos 只有在 tryToMove 成功找到一个开放方格时才会被设为 true
		if (!updatePos)
			resultantMove = ZeroVector;
		else { // 如果探测成功 (updatePos 为 true)，则进入这个代码块，对找到的“滑动”方案 resultantMove 进行最后的精加工。
			// 计算出应用这个“滑动”位移后，单位将到达的新位置
			const float3 openPos = prevPos + resultantMove;
			// 获取这个新位置所在的地图方格坐标
			const int2 openSquare = toMapSquare(openPos);
			// 计算出从原始方格到这个新方格的坐标差
			const int2 fullDiffSquare = openSquare - prevSquare;
			// 再次进行对角线移动检查。这是一个非常重要的二次验证。
			if (fullDiffSquare.x != 0 && fullDiffSquare.y != 0) {
				// axis-aligned slide to avoid clipping around corners and potentially into traps.
				// 轴对齐滑动 (Axis-Aligned Slide) 逻辑
				// 获取单位当前的朝向，表示为0-3的整数，代表东南西北等方向。
				const unsigned int facing = GetFacingFromHeading(unit->heading);
				// 定义了两个基础的坐标轴向量：Z轴 (0,0,1) 和 X轴 (1,0,0)。
				constexpr float3 vecs[2] =
					{ { 0.f, 0.f, 1.f}
					, { 1.f, 0.f, 0.f}
				};
				// 根据单位的朝向，选择一个与之最垂直的坐标轴作为“滑动轴”。
				const float3 aaSlideAxis = vecs[(facing - 1) % 2];
				// 提取出 resultantMove 在滑动轴上的分量大小。
				const float displacement = (facing % 2 == 0) ? resultantMove.x : resultantMove.z;
				// 计算出滑动的方向（+1 或 -1）
				const float side = 1.f - (2.f * (displacement < 0.f));
				// 计算最终的、轴对齐的滑动向量
				// std::min(displacement*side, speed): 确保滑动的距离不会超过单位本来的移动速度 speed。
				const float3 offset = aaSlideAxis * std::min(displacement*side, speed) * side;
				// 计算出应用这个纯粹的轴对齐滑动后，单位将到达的最终位置。
				const float3 posToTest = prevPos + offset;
				// 对这个最终的、经过轴对齐修正的位置进行最后一次安全检查。
				updatePos = isSquareOpen(posToTest);

			// 	{bool printMoveInfo = (selectedUnitsHandler.selectedUnits.size() == 1)
			// 		&& (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end());
			// {	bool printMoveInfo = (unit->id == 19432);
			// 	if (printMoveInfo) {
			// 		LOG("%s: unit %d: facing(%f,%f,%f) [%d:%d] right(%f,%f,%f) disp=%f"
			// 				, __func__, owner->id
			// 				, float(unit->frontdir.x), float(unit->frontdir.y), float(unit->frontdir.z), int(unit->heading), facing
			// 				, aaSlideAxis.x, aaSlideAxis.y, aaSlideAxis.z
			// 				, displacement);
			// 		LOG("%s: unit %d: resultantVec=(%f,%f,%f) prevPos=(%f,%f,%f) offset=(%f,%f,%f) posToTest=(%f,%f,%f) result=%d"
			// 				, __func__, owner->id
			// 				, resultantMove.x, resultantMove.y, resultantMove.z
			// 				, prevPos.x, prevPos.y, prevPos.z
			// 				, offset.x, offset.y, offset.z
			// 				, posToTest.x, posToTest.y, posToTest.z
			// 				, int(updatePos));
			// 	}}
				// 如果这个轴对齐的滑动是安全的。 就将这个安全的、纯粹的滑动向量 offset 作为最终的移动结果。
				if (updatePos) {
					resultantMove = offset;
				} else { // 如果连轴对齐的滑动都不安全。 说明单位被彻底堵死了，最终移动向量为零。

					resultantMove = ZeroVector;
				}
			} else if (resultantMove.SqLength2D() > speed*speed) { // 触发时机: 如果“滑动”探测成功了，并且这个滑动不是对角线移动 检查这个侧向滑动的距离是否超过了单位本来的前进速度。
				// 再次调用 tryToMove，但这次传入了 speed 作为 maxDisplacement 参数。这会强制将滑动的距离限制在 speed 之内。
				updatePos = tryToMove(prevPos, resultantMove, speed);
				// 如果连限制了距离的滑动都不安全，那就彻底放弃移动。
				if (!updatePos)
					resultantMove = ZeroVector;
			}
		}
	}
}

// UpdateOwnerPos 的核心任务就是：接收这个“理想的”新速度向量，
// 然后通过一个关键的 UpdatePos 函数进行碰撞规避和地形检查，最后计算出单位在本帧实际可以移动的位移量，并将其应用。
void CGroundMoveType::UpdateOwnerPos(const float3& oldSpeedVector, const float3& newSpeedVector) {
	RECOIL_DETAILED_TRACY_ZONE;
	// // oldSpeedVector 在 flatFrontDir 投影长度
	/*const float*/ oldSpeed = oldSpeedVector.dot(flatFrontDir); 
	/*const float*/ newSpeed = newSpeedVector.dot(flatFrontDir);
	// 作用: 将理想的新速度向量 newSpeedVector 赋给一个名为 moveRequest 的常量。
	// 这个名字更直观地表达了它的意图：这是单位“请求”在本帧内移动的位移向量。
	const float3 moveRequest = newSpeedVector;

	// if being built, the nanoframe might not be exactly on
	// the ground and would jitter from gravity acting on it
	// --> nanoframes can not move anyway, just return early
	// (units that become reverse-built will continue moving)
	// 作用: 守护条件。如果单位正在建造中，它不能移动。直接返回以跳过所有移动逻辑。
	// 注释解释了原因：建造中的纳米框架可能不完全贴合地面，如果施加重力会导致其抖动
	if (owner->beingBuilt)
		return;
	// 这是一个优化。只有当计算出的新速度向量与旧的速度向量不同时，才需要更新单位的速度状态。
	if (!oldSpeedVector.same(newSpeedVector)) {
		// 作用: 调用单位的成员函数，将 owner->speed（三维速度向量）和 owner->speed.w（速率标量）更新为新的值。
		owner->SetVelocityAndSpeed(newSpeedVector);
	}

	if (!moveRequest.same(ZeroVector)) { // 只有在单位有移动请求时（即速度不为零），才执行后续复杂的碰撞和位置更新逻辑。
		// 声明一个三维向量，用于接收 UpdatePos 的计算结果。
		float3 resultantVel; 
		bool limitDisplacment = true;
		float maxDisplacementSq = -1.f;
		// 这是整个函数中最关键的调用
		// 作用: UpdatePos 函数接收单位“想要”移动的位移向量 moveRequest，然后在内部进行精细的地形和静态障碍物碰撞检测。
		// 它会检查这个移动是否会导致单位穿墙或进入不可通行的地形。
		// 输出: 它会返回一个经过修正的、实际可行的位移向量 resultantVel。
		// 	如果 moveRequest 是安全的，resultantVel 会等于 moveRequest
		//  如果 moveRequest 会导致碰撞，resultantVel 会被修改（例如，变成沿着墙壁滑动的向量），甚至可能变成零向量（如果完全被堵住）。
		UpdatePos(owner, moveRequest, resultantVel, ThreadPool::GetThreadNum());
		// bool isMoveColliding = ...: 检查 UpdatePos 返回的实际位移 resultantVel 是否与我们请求的位移 moveRequest 不同。
		// 如果不同，就意味着发生了碰撞或被地形阻挡。
		bool isMoveColliding = !resultantVel.same(moveRequest);
		// 如果检测到碰撞，就调用 ReRequestPath(false) 来请求一次新的寻路。
		// 这是一种重要的自适应行为。当前的移动被阻挡，很可能意味着原有的路径已经失效（例如，路上突然出现了一个残骸）。
		// 通过请求新路径，单位可以尝试找到绕过这个新障碍的方法。
		if (isMoveColliding) {
			// Sometimes now regular collisions won't happen due to this code preventing that.
			// so units need to be able to get themselves out of stuck situations. So adding
			// a rerequest path check here.
			// 由于此代码的阻止，常规碰撞有时不会发生
			// 因此，单位需要能够能够自行摆脱被困状态。所以在此处添加了一个重新请求路径的检查。
			ReRequestPath(false);
		}
		// 检查 UpdatePos 返回的实际位移是否为零向量。如果不为零，说明至少还有一个可移动的方向。
		bool isThereAnOpenSquare = !resultantVel.same(ZeroVector);

		if (isThereAnOpenSquare){
			// resultantForces = resultantVel;: 如果有路可走，就将这个安全的位移向量 resultantVel 赋值给 resultantForces。
			// 这个 resultantForces 变量会在稍后的 UpdatePreCollisions 中被用来实际移动单位。
			resultantForces = resultantVel;
			// owner->Move(resultantVel, true);
			// : 如果单位之前被标记为“卡住”（positionStuck），但现在又能移动了，就清除这个标志。
			if (positionStuck) {
				positionStuck = false;
			}
		} else if (positionStuck) { // 触发时机: UpdatePos 返回了零向量（完全没路可走），并且单位之前已经被标记为“卡住”。
			// Unit is stuck an the an open square could not be found. Just allow the unit to move
			// so that it gets unstuck eventually. Hopefully this won't look silly in practice, but
			// it is better than a unit being permanently stuck on something.
			// 这是一个强制解卡逻辑。注释解释说：既然单位已经被卡住，并且找不到任何开放的方格，那么就强行允许它按照原始的 moveRequest 移动。
			// 这可能会导致它暂时穿过障碍物的边缘，但这被认为是比让单位永久卡死更好的选择
			resultantForces = moveRequest;
			// owner->Move(moveRequest, true);
		}

		// NOTE:
		//   this can fail when gravity is allowed (a unit catching air
		//   can easily end up on an impassable square, especially when
		//   terrain contains micro-bumps) --> more likely at lower g's
		// assert(owner->moveDef->TestMoveSquare(owner, owner->pos, owner->speed, true, false, true));
	}

	// reversing = UpdateOwnerSpeed(math::fabs(oldSpeed), math::fabs(newSpeed), newSpeed);
}

bool CGroundMoveType::UpdateOwnerSpeed(float oldSpeedAbs, float newSpeedAbs, float newSpeedRaw)
{
	RECOIL_DETAILED_TRACY_ZONE;
	const bool oldSpeedAbsGTZ = (oldSpeedAbs > 0.01f);
	const bool newSpeedAbsGTZ = (newSpeedAbs > 0.01f);
	const bool newSpeedRawLTZ = (newSpeedRaw < 0.0f );

	owner->UpdatePhysicalStateBit(CSolidObject::PSTATE_BIT_MOVING, newSpeedAbsGTZ);

	if (!oldSpeedAbsGTZ &&  newSpeedAbsGTZ)
		owner->script->StartMoving(newSpeedRawLTZ);
	if ( oldSpeedAbsGTZ && !newSpeedAbsGTZ)
		owner->script->StopMoving();

	// Push resistant units need special handling, when they start/stop.
	// Not much point while they are on the move because their squares get recognised as moving,
	// not structure, and are ignored by collision and pathing.
	bool changeInMotion = IsPushResistant() && oldSpeedAbsGTZ != newSpeedAbsGTZ;
	if (changeInMotion){
		if (newSpeedAbsGTZ) {
			owner->UnBlock();
			pushResistanceBlockActive = false;
		} else {
			owner->Block();
			pushResistanceBlockActive = true;
			RegisterUnitForUnitTrapCheck(owner);
		}

		// this has to be done manually because units don't trigger it with block commands
		const int bx = owner->mapPos.x, sx = owner->xsize;
		const int bz = owner->mapPos.y, sz = owner->zsize;
		const int xminSqr = bx, xmaxSqr = bx + sx;
		const int zminSqr = bz, zmaxSqr = bz + sz;

		pathManager->TerrainChange(xminSqr, zminSqr, xmaxSqr, zmaxSqr, TERRAINCHANGE_OBJECT_INSERTED);
	}

	currentSpeed = newSpeedAbs;
	deltaSpeed   = 0.0f;

	return newSpeedRawLTZ;
}


bool CGroundMoveType::WantReverse(const float3& wpDir, const float3& ffDir) const
{
	RECOIL_DETAILED_TRACY_ZONE;
	if (!canReverse)
		return false;

	// these values are normally non-0, but LuaMoveCtrl
	// can override them and we do not want any div0's
	if (maxReverseSpeed <= 0.0f)
		return false;
	if (maxSpeed <= 0.0f)
		return true;

	if (accRate <= 0.0f)
		return false;
	if (decRate <= 0.0f)
		return false;
	if (turnRate <= 0.0f)
		return false;

	if (wpDir.dot(ffDir) >= 0.0f)
		return false;

	const float goalDist   = (goalPos - owner->pos).Length2D();                  // use *final* WP for ETA calcs; in elmos
	const float goalFwdETA = (goalDist / maxSpeed);                              // in frames (simplistic)
	const float goalRevETA = (goalDist / maxReverseSpeed);                       // in frames (simplistic)

	const float waypointAngle = std::clamp(wpDir.dot(owner->frontdir), -1.0f, 0.0f);  // clamp to prevent NaN's; [-1, 0]
	const float turnAngleDeg  = math::acosf(waypointAngle) * math::RAD_TO_DEG;   // in degrees; [90.0, 180.0]
	const float fwdTurnAngle  = (turnAngleDeg / 360.0f) * SPRING_CIRCLE_DIVS;    // in "headings"
	const float revTurnAngle  = SPRING_MAX_HEADING - fwdTurnAngle;               // 180 deg - angle

	// values <= 0 preserve default behavior
	if (maxReverseDist > 0.0f && minReverseAngle > 0.0f)
		return (currWayPointDist <= maxReverseDist && turnAngleDeg >= minReverseAngle);

	// units start accelerating before finishing the turn, so subtract something
	const float turnTimeMod      = 5.0f;
	const float fwdTurnAngleTime = std::max(0.0f, (fwdTurnAngle / turnRate) - turnTimeMod); // in frames
	const float revTurnAngleTime = std::max(0.0f, (revTurnAngle / turnRate) - turnTimeMod);

	const float apxFwdSpdAfterTurn = std::max(0.0f, currentSpeed - 0.125f * (fwdTurnAngleTime * decRate));
	const float apxRevSpdAfterTurn = std::max(0.0f, currentSpeed - 0.125f * (revTurnAngleTime * decRate));

	const float fwdDecTime = ( reversing * apxFwdSpdAfterTurn) / decRate;
	const float revDecTime = (!reversing * apxRevSpdAfterTurn) / decRate;
	const float fwdAccTime = (maxSpeed        - !reversing * apxFwdSpdAfterTurn) / accRate;
	const float revAccTime = (maxReverseSpeed -  reversing * apxRevSpdAfterTurn) / accRate;

	const float fwdETA = goalFwdETA + fwdTurnAngleTime + fwdAccTime + fwdDecTime;
	const float revETA = goalRevETA + revTurnAngleTime + revDecTime + revAccTime;

	return (fwdETA > revETA);
}



void CGroundMoveType::InitMemberPtrs(MemberData* memberData)
{
	RECOIL_DETAILED_TRACY_ZONE;
	memberData->bools[0].second = &atGoal;
	memberData->bools[1].second = &atEndOfPath;
	memberData->bools[2].second = &pushResistant;

	memberData->shorts[0].second = &minScriptChangeHeading;

	memberData->floats[0].second = &turnRate;
	memberData->floats[1].second = &turnAccel;
	memberData->floats[2].second = &accRate;
	memberData->floats[3].second = &decRate;
	memberData->floats[4].second = &myGravity;
	memberData->floats[5].second = &maxReverseDist,
	memberData->floats[6].second = &minReverseAngle;
	memberData->floats[7].second = &maxReverseSpeed;
	memberData->floats[8].second = &sqSkidSpeedMult;
}

bool CGroundMoveType::SetMemberValue(unsigned int memberHash, void* memberValue)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// try the generic members first
	if (AMoveType::SetMemberValue(memberHash, memberValue))
		return true;

	// set pointers for this GMT instance
	InitMemberPtrs(&gmtMemberData);

	// special cases
	if (memberHash == gmtMemberData.floats[MAXREVERSESPEED_MEMBER_IDX].first) {
		*(gmtMemberData.floats[MAXREVERSESPEED_MEMBER_IDX].second) = *(reinterpret_cast<float*>(memberValue)) / GAME_SPEED;
		return true;
	}

	// note: <memberHash> should be calculated via HsiehHash
	// todo: use template lambdas in C++14
	{
		const auto pred = [memberHash](const std::pair<unsigned int, bool*>& p) { return (memberHash == p.first); };
		const auto iter = std::find_if(gmtMemberData.bools.begin(), gmtMemberData.bools.end(), pred);
		if (iter != gmtMemberData.bools.end()) {
			*(iter->second) = *(reinterpret_cast<bool*>(memberValue));
			return true;
		}
	}
	{
		const auto pred = [memberHash](const std::pair<unsigned int, short*>& p) { return (memberHash == p.first); };
		const auto iter = std::find_if(gmtMemberData.shorts.begin(), gmtMemberData.shorts.end(), pred);
		if (iter != gmtMemberData.shorts.end()) {
			*(iter->second) = short(*(reinterpret_cast<float*>(memberValue))); // sic (see SetMoveTypeValue)
			return true;
		}
	}
	{
		const auto pred = [memberHash](const std::pair<unsigned int, float*>& p) { return (memberHash == p.first); };
		const auto iter = std::find_if(gmtMemberData.floats.begin(), gmtMemberData.floats.end(), pred);
		if (iter != gmtMemberData.floats.end()) {
			*(iter->second) = *(reinterpret_cast<float*>(memberValue));
			return true;
		}
	}

	return false;
}

