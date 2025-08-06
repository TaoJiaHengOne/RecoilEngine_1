/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#include "MoveMath.h"
#include "Sim/Misc/ModInfo.h"
#include "Sim/MoveTypes/MoveDefHandler.h"

#include "System/Misc/TracyDefs.h"

/*
Calculate speed-multiplier for given height and slope data.
*/
float CMoveMath::GroundSpeedMod(const MoveDef& moveDef, float height, float slope)
{
	RECOIL_DETAILED_TRACY_ZONE;
	float speedMod = 0.0f;

	// slope too steep or square too deep?
	if (slope > moveDef.maxSlope)
		return speedMod;
	if (-height > moveDef.depth)
		return speedMod;

	// slope-mod
	/*
	这是与方向版本的关键区别：
	- 不考虑移动方向：直接用 slope * moveDef.slopeMod
	- 所有坡度都造成惩罚：无论上坡还是下坡，只要有坡度就会降低速度
	- 公式解析：
		- slope * moveDef.slopeMod: 坡度越大，惩罚越大
		- 1.0f + ...: 基础值1加上惩罚值
		- 1.0f / (...): 倒数形式，分母越大结果越小（速度越慢）	
	*/
	speedMod = 1.0f / (1.0f + slope * moveDef.slopeMod);
	// 在水中应用额外惩罚系数
	speedMod *= ((height < 0.0f)? waterDamageCost: 1.0f);
	//  应用深度相关的速度修正。
	speedMod *= moveDef.GetDepthMod(height);

	return speedMod;
}

float CMoveMath::GroundSpeedMod(const MoveDef& moveDef, float height, float slope, float dirSlopeMod)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// Directional speed is now equal to regular except when:
	// 1) Climbing out of places which are below max depth.
	// 2) Climbing hills is slower.
	//  初始化速度修正系数为0，默认表示不可通行。
	float speedMod = 0.0f;
	//  如果当前坡度超过单位的最大可通行坡度限制，直接返回0（完全不可通行）。
	if (slope > moveDef.maxSlope)
		return speedMod;
	//  如果当前位置的水深（用负高度表示）超过单位的最大涉水深度，返回0（不可通行）。
	//  注意：-height将负的水下高度转换为正的深度值。
	// is this square below our maxWaterDepth?
	if ((-height) > moveDef.depth)
		return speedMod;

	// slope-mod (speedMod is not increased or decreased by downhill slopes)
	/*
	这是函数的核心逻辑：
	- slope * dirSlopeMod: 将坡度与方向修正相乘
		- 如果dirSlopeMod > 0（上坡），结果为正，造成速度惩罚
		- 如果dirSlopeMod < 0（下坡），结果为负
	- std::max(0.0f, ...): 确保只有上坡时才有速度惩罚，下坡时不给速度加成
	- ... * moveDef.slopeMod: 乘以单位的坡度敏感系数
	- 1.0f / (1.0f + ...): 倒数公式，值越大速度越慢	
	*/
	speedMod = 1.0f / (1.0f + std::max(0.0f, slope * dirSlopeMod) * moveDef.slopeMod);
	// 果在水中（height < 0），应用水中移动的额外惩罚系数waterDamageCost，否则不变。
	speedMod *= ((height < 0.0f)? waterDamageCost: 1.0f);
	// 应用单位定义中的深度修正函数，通常随深度增加而降低速度。
	speedMod *= moveDef.GetDepthMod(height);
 	// 返回最终的速度修正系数（0-1之间，越接近1速度越快）。
	return speedMod;
}

