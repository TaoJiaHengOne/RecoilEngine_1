/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */
/* 本文件是 Spring 引擎的一部分 (遵循 GPL v2 或更高版本协议)，详见 LICENSE.html */


#include "Ground.h"
#include "ReadMap.h"                      // 包含了地图数据读取类 CReadMap 的定义。
#include "Sim/Misc/GlobalConstants.h"     // 包含了全局常量，如 SQUARE_SIZE。
#include "Sim/Misc/GlobalSynced.h"        // 包含了同步的全局变量，如 gs (GlobalSynced)。
#include "System/SpringMath.h"            // 包含了数学相关的函数和定义。

#include <cassert>                        // 用于断言 assert。
#include <limits>                         // 用于访问数值极限，如 std::numeric_limits。

#include "System/Misc/TracyDefs.h"        // 包含了 Tracy 性能分析器的宏定义。

#undef far // 避免与 windef.h 中的 far 宏冲突
#undef near // 避免与 windef.h 中的 near 宏冲突

/**
 * @brief 在角点高度图上进行插值计算
 * @param x 世界坐标x
 * @param z 世界坐标z
 * @param cornerHeightMap 指向角点高度图数据的指针
 * @return float 插值后的高度
 */
static inline float InterpolateCornerHeight(float x, float z, const float* cornerHeightMap)
{
	// 注意:
	// 这不是一个双线性插值。它是在构成地面方块的两个三角形上进行插值：
	//
	// TL __________ TR (左上角, 右上角)
	//    |        /|
	//    | dx+dz / |
	//    | \<1  /  | (左上三角形)
	//    |     /   |
	//    |    /    |
	//    |   /     |
	//    |  / dx+dz|
	//    | /  \>=1 | (右下三角形)
	//    |/        |
	// BL ---------- BR (左下角, 右下角)
	//
	// 将世界坐标转换为高度图格子坐标
	x = std::clamp(x, 0.0f, float3::maxxpos) / SQUARE_SIZE;
	z = std::clamp(z, 0.0f, float3::maxzpos) / SQUARE_SIZE;

	const int ix = x; // 整数部分，即方格的x索引
	const int iz = z; // 整数部分，即方格的z索引
	const int hs = ix + iz * mapDims.mapxp1; // 高度图中左上角顶点的一维数组索引

	const float dx = x - ix; // 小数部分，即在方格内的x偏移
	const float dz = z - iz; // 小数部分，即在方格内的z偏移

	float h = 0.0f; // 计算出的高度

	if (dx + dz < 1.0f) {
		// 如果点在左上三角形内
		const float h00 = cornerHeightMap[hs + 0                 ]; // 左上顶点高度
		const float h10 = cornerHeightMap[hs + 1                 ]; // 右上顶点高度
		const float h01 = cornerHeightMap[hs + 0 + mapDims.mapxp1]; // 左下顶点高度

		// 根据x和z的偏移量，在三角形平面上进行线性插值
		const float xdif = dx * (h10 - h00);
		const float zdif = dz * (h01 - h00);

		h = h00 + xdif + zdif;
	} else {
		// 如果点在右下三角形内
		const float h10 = cornerHeightMap[hs + 1                 ]; // 右上顶点高度
		const float h01 = cornerHeightMap[hs + 0 + mapDims.mapxp1]; // 左下顶点高度
		const float h11 = cornerHeightMap[hs + 1 + mapDims.mapxp1]; // 右下顶点高度

		// 根据x和z的偏移量，在三角形平面上进行线性插值（相对于右下顶点）
		const float xdif = (1.0f - dx) * (h01 - h11);
		const float zdif = (1.0f - dz) * (h10 - h11);

		h = h11 + xdif + zdif;
	}

	return h;
}


/**
 * @brief 检查一条线段是否与指定的地图方格发生碰撞
 * @param heightmap 高度图数据
 * @param normalmap 法线图数据
 * @param from 线段起点
 * @param to 线段终点
 * @param xs 方格的x索引
 * @param ys 方格的y索引 (在引擎中通常用y表示z轴)
 * @return float 如果碰撞，返回碰撞点到起点的距离；否则返回负数。
 */
static inline float LineGroundSquareCol(
	const float* heightmap,
	const float3* normalmap,
	const float3& from,
	const float3& to,
	const int xs,
	const int ys
) {
	RECOIL_DETAILED_TRACY_ZONE; // Tracy性能分析区域宏
	const bool inMap = (xs >= 0) && (ys >= 0) && (xs <= mapDims.mapxm1) && (ys <= mapDims.mapym1);
//	assert(inMap); // 断言确保方格在地图内
	if (!inMap)
		return -1.0f;

	// 地形网格每个方格由两个直角等腰三角形组成，
	// 所以我们必须检查两个面（三角形）是否存在交点。
	// 对于每个三角形，我们选择一个代表性的顶点。

	// 检查左上三角形
	{
		float3 cornerVertex;
		cornerVertex.x = xs * SQUARE_SIZE;
		cornerVertex.z = ys * SQUARE_SIZE;
		cornerVertex.y = heightmap[ys * mapDims.mapxp1 + xs];

		// 将 <to - cornerVertex> 向量投影到左上角面的法线上
		// 如果 <to> 点在地形下方，这个值会是负数
		const float3 faceNormalTL = normalmap[(ys * mapDims.mapx + xs) * 2    ];
		float toFacePlaneDist = (to - cornerVertex).dot(faceNormalTL);

		if (toFacePlaneDist <= 0.0f) {
			// 将 <from - cornerVertex> 向量投影到左上角面的法线上
			const float fromFacePlaneDist = (from - cornerVertex).dot(faceNormalTL);

			if (fromFacePlaneDist != toFacePlaneDist) {
				// 计算线段与三角平面的交点
				const float alpha = fromFacePlaneDist / (fromFacePlaneDist - toFacePlaneDist);
				const float3 col = mix(from, to, alpha);

				// 检查交点是否在左上三角形的边界内
				if ((col.x >= cornerVertex.x) && (col.z >= cornerVertex.z) && (col.x + col.z <= cornerVertex.x + cornerVertex.z + SQUARE_SIZE))
					return col.distance(from); // 返回碰撞点到起点的距离
			}
		}
	}

	// 检查右下三角形
	{
		float3 cornerVertex;
		cornerVertex.x = (xs + 1) * SQUARE_SIZE;
		cornerVertex.z = (ys + 1) * SQUARE_SIZE;
		cornerVertex.y = heightmap[(ys + 1) * mapDims.mapxp1 + (xs + 1)];

		// 将 <to - cornerVertex> 向量投影到右下角面的法线上
		const float3 faceNormalBR = normalmap[(ys * mapDims.mapx + xs) * 2 + 1];
		float toFacePlaneDist = (to - cornerVertex).dot(faceNormalBR);

		if (toFacePlaneDist <= 0.0f) {
			// 将 <from - cornerVertex> 向量投影到右下角面的法线上
			const float fromFacePlaneDist = (from - cornerVertex).dot(faceNormalBR);

			if (fromFacePlaneDist != toFacePlaneDist) {
				// 计算线段与三角平面的交点
				const float alpha = fromFacePlaneDist / (fromFacePlaneDist - toFacePlaneDist);
				const float3 col = mix(from, to, alpha);

				// 检查交点是否在右下三角形的边界内
				if ((col.x <= cornerVertex.x) && (col.z <= cornerVertex.z) && (col.x + col.z >= cornerVertex.x + cornerVertex.z - SQUARE_SIZE))
					return col.distance(from); // 返回碰撞点到起点的距离
			}
		}
	}

	return -2.0f; // 未发生碰撞
}



/* // (此段代码已被注释掉)
void CGround::CheckColSquare(CProjectile* p, int x, int y)
{
	if (!(x >= 0 && y >= 0 && x < mapDims.mapx && y < mapDims.mapy))
		return;

	float xp = p->pos.x;
	float yp = p->pos.y;
	float zp = p->pos.z;

	const float* hm = readMap->GetCornerHeightMapSynced();
	const float3* fn = readMap->GetFaceNormalsSynced();
	const int hmIdx = (y * mapDims.mapx + x);
	const float xt = x * SQUARE_SIZE;
	const float& yt0 = hm[ y      * mapDims.mapxp1 + x    ];
	const float& yt1 = hm[(y + 1) * mapDims.mapxp1 + x + 1];
	const float zt = y * SQUARE_SIZE;

	const float3& fn0 = fn[hmIdx * 2    ];
	const float3& fn1 = fn[hmIdx * 2 + 1];
	const float dx0 = (xp -  xt     );
	const float dy0 = (yp -  yt0    );
	const float dz0 = (zp -  zt     );
	const float dx1 = (xp - (xt + 2));
	const float dy1 = (yp -  yt1    );
	const float dz1 = zp - (zt + 2);
	const float d0 = dx0 * fn0.x + dy0 * fn0.y + dz0 * fn0.z;
	const float d1 = dx1 * fn1.x + dy1 * fn1.y + dz1 * fn1.z;
	const float s0 = xp + zp - xt - zt - p->radius;
	const float s1 = xp + zp - xt - zt - SQUARE_SIZE * 2 + p->radius;

	if ((d0 <= p->radius) && (s0 < SQUARE_SIZE))
		p->Collision();

	if ((d1 <= p->radius) && (s1 > -SQUARE_SIZE))
		p->Collision();

	return;
}
*/

/**
 * @brief 将线段的起点钳制到地图最大高度平面以下
 * @param from 线段起点 (会被修改)
 * @param to 线段终点 (会被修改)
 * @return bool 如果线段被钳制，返回true
 */
inline static bool ClampInMapHeight(float3& from, float3& to)
{
	RECOIL_DETAILED_TRACY_ZONE;
	const float heightAboveMapMax = from.y - readMap->GetCurrMaxHeight();

	if (heightAboveMapMax <= 0.0f)
		return false;

	const float3 dir = to - from;

	if (dir.y >= 0.0f) {
		// 如果起点和终点都在地图最大高度之上，则不可能碰撞
		from = -OnesVector;
		to   = -OnesVector;
		return true;
	}

	// 将起点移动到与地图最大高度平面的交点处
	from += (dir * (-heightAboveMapMax / dir.y));
	return true;
}


float CGround::LineGroundCol(float3 from, float3 to, bool synced)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// 获取地形数据指针
	const float* hm  = readMap->GetSharedCornerHeightMap(synced);
	const float3* nm = readMap->GetSharedFaceNormals(synced);

	const float3 pfrom = from; // 保存原始起点

	// 性能优化 -> 跳过不可能与地形碰撞的部分，因为它在地图当前最大高度之上
	ClampInMapHeight(from, to);

	// 处理射线起点在地图边界外的特殊情况：
	// 需要沿着射线将 <from> 移动到最近的地图边界上
	// (如果 <from> 和 <to> 都在边界外，射线仍可能击中地图)
	// 简单地钳制 <from> 会改变射线的方向，因此我们保存被跳过的距离
	ClampLineInMap(from, to);

	// ClampLineInMap & ClampInMapHeight 在线段完全在地图外时，
	// 会设置 `from == to == vec(-1,-1,-1)`
	if (from == to)
		return -1.0f;

	const float skippedDist = pfrom.distance(from); // 计算被跳过的距离

	if (synced) {
		// TODO: 在非同步模式下也这样做吗？
		// 检查我们的起点是否在地下（假设地面对于炮弹等是不可穿越的）
		const int sx = from.x / SQUARE_SIZE;
		const int sz = from.z / SQUARE_SIZE;

		if (from.y <= hm[sz * mapDims.mapxp1 + sx])
			return 0.0f + skippedDist; // 如果起点就在地下，立即返回碰撞
	}

	// DDA (Digital Differential Analyzer) / Voxel Traversal 算法开始
	const float dx = to.x - from.x;
	const float dz = to.z - from.z;
	const int dirx = (dx > 0.0f) ? 1 : -1; // x方向的步进方向
	const int dirz = (dz > 0.0f) ? 1 : -1; // z方向的步进方向

	// 将起点和终点钳制到地图格子坐标范围内
	const float ffsx = std::clamp(from.x / SQUARE_SIZE, 0.0f, static_cast<float>(mapDims.mapx));
	const float ffsz = std::clamp(from.z / SQUARE_SIZE, 0.0f, static_cast<float>(mapDims.mapy));
	const float ttsx = std::clamp(  to.x / SQUARE_SIZE, 0.0f, static_cast<float>(mapDims.mapx));
	const float ttsz = std::clamp(  to.z / SQUARE_SIZE, 0.0f, static_cast<float>(mapDims.mapy));
	const int fsx = ffsx; // 起点方格x索引
	const int fsz = ffsz; // 起点方格z索引
	const int tsx = ttsx; // 终点方格x索引
	const int tsz = ttsz; // 终点方格z索引

	bool stopTrace = false; // 循环停止标志

	if ((fsx == tsx) && (fsz == tsz)) {
		// 如果起点和终点在同一个方格内
		const float ret = LineGroundSquareCol(hm, nm,  from, to,  fsx, fsz);

		if (ret >= 0.0f)
			return (ret + skippedDist);

		return -1.0f;
	}

	if (fsx == tsx) {
		// 如果射线平行于z轴
		int zp = fsz;
		for (unsigned int i = 0, n = Square(mapDims.mapyp1); (Square(i) <= n && zp != tsz); i++) {
			const float ret = LineGroundSquareCol(hm, nm,  from, to,  fsx, zp);
			if (ret >= 0.0f)
				return (ret + skippedDist);
			zp += dirz;
		}
		return -1.0f;
	}

	if (fsz == tsz) {
		// 如果射线平行于x轴
		int xp = fsx;
		for (unsigned int i = 0, n = Square(mapDims.mapxp1); (Square(i) <= n && xp != tsx); i++) {
			const float ret = LineGroundSquareCol(hm, nm,  from, to,  xp, fsz);
			if (ret >= 0.0f)
				return (ret + skippedDist);
			xp += dirx;
		}
		return -1.0f;
	}

	{
		// 一般情况，使用DDA算法遍历格子
		const float rdsx = SQUARE_SIZE / dx; // x方向上前进一步需要的射线长度比例
		const float rdsz = SQUARE_SIZE / dz; // z方向上前进一步需要的射线长度比例

		// 在负方向上需要移动测试点
		const float testposx = (dx > 0.0f) ? 0.0f : 1.0f;
		const float testposz = (dz > 0.0f) ? 0.0f : 1.0f;

		int curx = fsx;
		int curz = fsz;

		for (unsigned int i = 0, n = Square(mapDims.mapxp1) + Square(mapDims.mapyp1); !stopTrace; i++) {
			// 测试与当前方格的碰撞
			const float ret = LineGroundSquareCol(hm, nm,  from, to,  curx, curz);

			if (ret >= 0.0f)
				return (ret + skippedDist);

			// 检查是否已到达终点方格
			const bool endReached = ((curx == tsx && curz == tsz) || (Square(i) > n));
			const bool beyondEnd = (((curx - tsx) * dirx > 0) || ((curz - tsz) * dirz > 0));

			assert(!beyondEnd); // 不应该超出终点

			stopTrace = (endReached || beyondEnd);

			// 计算到达x和z方向下一个边界所需的射线长度比例
			int nextx = curx + dirx;
			int nextz = curz + dirz;
			float xn = (nextx + testposx - ffsx) * rdsx;
			float zn = (nextz + testposz - ffsz) * rdsz;

			// 处理浮点精度问题和边界情况
			if ((nextx - tsx) * dirx > 0) { xn = 1337.0f; nextx = tsx; }
			if ((nextz - tsz) * dirz > 0) { zn = 1337.0f; nextz = tsz; }

			// 步进到x或z方向上更近的下一个方格
			if (xn >= 1.0f && zn >= 1.0f) {
				curx = nextx;
				curz = nextz;
			} else if (xn < zn) {
				assert(curx != nextx);
				curx = nextx;
			} else {
				assert(curz != nextz);
				curz = nextz;
			}
		}
	}

	return -1.0f;
}

float CGround::LineGroundCol(const float3 pos, const float3 dir, float len, bool synced)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// 调用重载版本
	return (LineGroundCol(pos, pos + dir * std::max(len, 0.0f), synced));
}


float CGround::LinePlaneCol(const float3 pos, const float3 dir, float len, float hgt)
{
	RECOIL_DETAILED_TRACY_ZONE;
	const float3 end = pos + dir * std::max(len, 0.0f);

	// 如果起点在平面下或终点在平面上，不可能相交
	if (pos.y < hgt)
		return -1.0f;
	if (end.y > hgt)
		return -1.0f;

	// 如果射线平行于或远离平面，不可能相交
	if (dir.y >= 0.0f)
		return (std::numeric_limits<float>::max()); // 返回一个极大值表示无限远

	// 计算交点距离
	return ((pos.y - hgt) / -dir.y);
}


float CGround::LineGroundWaterCol(const float3 pos, const float3 dir, float len, bool testWater, bool synced)
{
	RECOIL_DETAILED_TRACY_ZONE;
	const float terraDist = LineGroundCol(pos, dir, len, synced); // 与地面的碰撞距离
	if (!testWater)
		return terraDist;

	const float waterDist = LinePlaneCol(pos, dir, len, GetWaterLevel(pos.x, pos.z, synced)); // 与水面的碰撞距离
	if (waterDist < 0.0f)
		return terraDist; // 如果不与水面相交，则只考虑地面

	const float3 end = pos + dir * waterDist;

	// 检查水面交点是否在地图范围内
	if (end.x < 0.0f || end.x > float3::maxxpos)
		return terraDist;
	if (end.z < 0.0f || end.z > float3::maxzpos)
		return terraDist;

	if (terraDist < 0.0f)
		return waterDist; // 如果不与地面相交，则只考虑水面

	return std::min(terraDist, waterDist); // 返回与地面或水面中更近的那个交点
}


float CGround::GetApproximateHeight(float x, float z, bool synced)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// 使用中心点高度图
	const float* heightMap = readMap->GetSharedCenterHeightMap(synced);

	// 获取(x,z)所在的方格索引
	const int xsquare = std::clamp(int(x) / SQUARE_SIZE, 0, mapDims.mapxm1);
	const int zsquare = std::clamp(int(z) / SQUARE_SIZE, 0, mapDims.mapym1);
	// 返回该方格的高度，不进行插值
	return heightMap[zsquare * mapDims.mapx + xsquare];
}


float CGround::GetApproximateHeightUnsafe(int x, int z, bool synced)
{
	RECOIL_DETAILED_TRACY_ZONE;
	const float* heightMap = readMap->GetSharedCenterHeightMap(synced);
	// 不安全的版本，直接用索引访问
	return heightMap[z * mapDims.mapx + x];
}

const float* CGround::GetApproximateHeightUnsafePtr(int x, int z, bool synced)
{
	RECOIL_DETAILED_TRACY_ZONE;
	const float* heightMap = readMap->GetSharedCenterHeightMap(synced);
	// 返回指向高度数据的指针
	return &heightMap[z * mapDims.mapx + x];
}

float CGround::GetHeightAboveWater(float x, float z, bool synced)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// 返回真实地形高度与水面高度的差值，但不小于0
	return std::max(0.0f, GetHeightReal(x, z, synced) - GetWaterLevel(x, z, synced));
}

float CGround::GetHeightReal(float x, float z, bool synced)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// 调用辅助函数，在角点高度图上进行插值
	return InterpolateCornerHeight(x, z, readMap->GetSharedCornerHeightMap(synced));
}

float CGround::GetOrigHeight(float x, float z)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// 在原始高度图上进行插值
	return InterpolateCornerHeight(x, z, readMap->GetOriginalHeightMapSynced());
}


const float3& CGround::GetNormal(float x, float z, bool synced)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// 获取(x,z)所在的方格索引
	const int xsquare = std::clamp(int(x) / SQUARE_SIZE, 0, mapDims.mapxm1);
	const int zsquare = std::clamp(int(z) / SQUARE_SIZE, 0, mapDims.mapym1);

	// 从中心点法线图中获取法线
	const float3* normalMap = readMap->GetSharedCenterNormals(synced);
	return normalMap[xsquare + zsquare * mapDims.mapx];
}

const float3& CGround::GetNormalAboveWater(float x, float z, bool synced)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// 如果地形在水下，则返回垂直向上的法线
	if (GetHeightReal(x, z, synced) <= 0.0f)
		return UpVector;

	return (GetNormal(x, z, synced));
}


float CGround::GetSlope(float x, float z, bool synced)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// 获取(x,z)所在的半分辨率方格索引
	const int xhsquare = std::clamp(int(x) / (2 * SQUARE_SIZE), 0, mapDims.hmapx - 1);
	const int zhsquare = std::clamp(int(z) / (2 * SQUARE_SIZE), 0, mapDims.hmapy - 1);
	const float* slopeMap = readMap->GetSharedSlopeMap(synced);

	// 从坡度图中获取坡度值
	return slopeMap[xhsquare + zhsquare * mapDims.hmapx];
}


float3 CGround::GetSmoothNormal(float x, float z, bool synced)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// 获取中心方格索引，并确保周围有邻居
	const int sx = std::clamp(int(math::floor(x / SQUARE_SIZE)), 1, mapDims.mapx - 2);
	const int sz = std::clamp(int(math::floor(z / SQUARE_SIZE)), 1, mapDims.mapy - 2);

	const float dx = (x / SQUARE_SIZE) - sx; // x方向偏移
	const float dz = (z / SQUARE_SIZE) - sz; // z方向偏移

	int sx2; // 相邻x索引
	int sz2; // 相邻z索引
	float fx; // x方向权重
	float fz; // z方向权重

	// 根据偏移量确定相邻的格子和插值权重
	if (dz > 0.5f) {
		sz2 = sz + 1;
		fz = dz - 0.5f;
	} else {
		sz2 = sz - 1;
		fz = 0.5f - dz;
	}

	if (dx > 0.5f) {
		sx2 = sx + 1;
		fx = dx - 0.5f;
	} else {
		sx2 = sx - 1;
		fx = 0.5f - dx;
	}

	const float ifz = 1.0f - fz;
	const float ifx = 1.0f - fx;

	const float3* normalMap = readMap->GetSharedCenterNormals(synced);

	// 对周围四个中心点的法线进行双线性插值
	const float3& n1 = normalMap[sz  * mapDims.mapx + sx ] * ifx * ifz;
	const float3& n2 = normalMap[sz  * mapDims.mapx + sx2] * fx * ifz;
	const float3& n3 = normalMap[sz2 * mapDims.mapx + sx ] * ifx * fz;
	const float3& n4 = normalMap[sz2 * mapDims.mapx + sx2] * fx * fz;

	// 返回归一化后的平滑法线
	return ((n1 + n2 + n3 + n4).Normalize());
}



float CGround::SimTrajectoryGroundColDist(const float3& trajStartPos, const float3& trajStartDir, const float3& acc, const float2& args)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// args.x := speed (速度), args.y := length (长度)
	// 获取轨迹在地图边界内的交点
	const float2 ips = GetMapBoundaryIntersectionPoints(trajStartPos, trajStartDir * XZVector * args.y);

	// 如果完全在地图外
	if (ips.y < 0.0f)
		return -1.0;

	const float minDist = args.y * std::max(0.0f, ips.x); // 最小检测距离
	const float maxDist = args.y * std::min(1.0f, ips.y); // 最大检测距离

	float3 pos = trajStartPos;
	float3 vel = trajStartDir * args.x;

	// 模拟一个虚拟投射物的轨迹，从 <pos> 以速度 <trajStartDir * speed> 发射；
	// 假设 <pos> 起点在地图内。
	// 先步进到最小检测距离
	while (pos.SqDistance2D(trajStartPos) < Square(minDist)) {
		vel += acc;
		pos += vel;
	}
	// 持续步进直到投射物位置低于地面高度
	while (pos.y >= GetHeightReal(pos)) {
		vel += acc;
		pos += vel;
	}

	// 如果碰撞点超出了最大检测距离，则视为未碰撞
	if (pos.SqDistance2D(trajStartPos) >= Square(maxDist))
		return -1.0f;

	// 返回碰撞点在xz平面上的距离
	return (math::sqrt(pos.SqDistance2D(trajStartPos)));
}

float CGround::TrajectoryGroundCol(const float3& trajStartPos, const float3& trajTargetDir, float length, float linCoeff, float qdrCoeff)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// trajTargetDir 应该是从 <trajStartPos> 到目标的归一化xz向量
	const float3 dir = {trajTargetDir.x, linCoeff, trajTargetDir.z};
	const float3 alt = UpVector * qdrCoeff; // 抛物线的高度分量

	// 将检测限制在线段在地图内的部分
	const float2 ips = GetMapBoundaryIntersectionPoints(trajStartPos, dir * length);

	// 完全在地图外
	if (ips.y < 0.0f)
		return -1.0;

	const float minDist = length * std::max(0.0f, ips.x);
	const float maxDist = length * std::min(1.0f, ips.y);

	// 沿着轨迹步进，检查每一点是否在地下
	for (float dist = minDist; dist < maxDist; dist += SQUARE_SIZE) {
		const float3 pos = (trajStartPos + dir * dist) + (alt * dist * dist);

		#if 1
		// 使用近似高度进行快速检查
		if (GetApproximateHeight(pos) > pos.y)
			return dist;
		#else
		// 使用真实插值高度进行精确检查（较慢）
		if (GetHeightReal(pos) > pos.y)
			return dist;
		#endif
	}

	return -1.0f;
}



int CGround::GetSquare(const float3& pos) {
	RECOIL_DETAILED_TRACY_ZONE;
	// 将世界坐标转换为地图方格索引，并钳制在范围内
	const int x = std::clamp((int(pos.x) / SQUARE_SIZE), 0, mapDims.mapxm1);
	const int z = std::clamp((int(pos.z) / SQUARE_SIZE), 0, mapDims.mapym1);

	// 返回一维数组索引
	return (x + z * mapDims.mapx);
};