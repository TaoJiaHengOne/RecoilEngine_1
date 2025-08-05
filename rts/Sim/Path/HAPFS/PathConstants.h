/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */
/* 本文件是 Spring 引擎的一部分 (遵循 GPL v2 或更高版本协议)，详见 LICENSE.html */

#ifndef PATH_CONSTANTS_H
#define PATH_CONSTANTS_H

#include <limits>
#include <array>
#include "Sim/Misc/GlobalConstants.h"
#include "Sim/MoveTypes/MoveDefHandler.h"

// 定义路径成本的无穷大值，用于表示不可达的路径。
static constexpr float PATHCOST_INFINITY = std::numeric_limits<float>::infinity();

// NOTE:
//   PF and PE both use a PathNodeBuffer of size MAX_SEARCHED_NODES,
//   thus MAX_SEARCHED_NODES_{PF, PE} MUST be <= MAX_SEARCHED_NODES
// 注意:
//   高精度寻路(PF)和低精度路径估算(PE)都使用大小为 MAX_SEARCHED_NODES 的路径节点缓冲区，
//   因此 MAX_SEARCHED_NODES_PF 和 MAX_SEARCHED_NODES_PE 必须小于或等于 MAX_SEARCHED_NODES。
static constexpr unsigned int MAX_SEARCHED_NODES    = 65536U; // A*搜索允许探索的最大节点数。
static constexpr unsigned int MAX_SEARCHED_NODES_PF = MAX_SEARCHED_NODES; // 高精度寻路(PF)允许搜索的最大节点数。
static constexpr unsigned int MAX_SEARCHED_NODES_PE = MAX_SEARCHED_NODES; // 路径估算器(PE)允许搜索的最大节点数。

// PathManager distance thresholds (to use PF or PE)
// 路径管理器(PathManager)的距离阈值 (用于决定使用高精度寻路PF或低精度估算PE)。
static constexpr float MAXRES_SEARCH_DISTANCE =  50.0f; // 使用高精度寻路(PF)的最大距离阈值。
static constexpr float MEDRES_SEARCH_DISTANCE = 200.0f; // 使用中分辨率路径估算(PE)的最大距离阈值。
// path-refinement lookahead distances (MED to MAX and LOW to MED)
// 路径优化前瞻距离 (从中分辨率到高分辨率，以及从低分辨率到中分辨率)。
static const float MAXRES_SEARCH_DISTANCE_EXT = (MAXRES_SEARCH_DISTANCE * 0.4f) * SQUARE_SIZE;
static const float MEDRES_SEARCH_DISTANCE_EXT = (MEDRES_SEARCH_DISTANCE * 0.4f) * SQUARE_SIZE;

// how many recursive refinement attempts NextWayPoint should make
// NextWayPoint 函数应尝试的递归路径优化最大次数。
static constexpr unsigned int MAX_PATH_REFINEMENT_DEPTH = 4;

// 路径估算器(PE)缓存文件的版本号，用于判断缓存是否过时。
static constexpr unsigned int PATHESTIMATOR_VERSION = 109;

// 定义中分辨率和低分辨率路径估算器的宏观块大小。
static constexpr unsigned int MEDRES_PE_BLOCKSIZE = 16; // 中分辨���路径估算器(PE)的块大小 (16x16 方格)。
static constexpr unsigned int LOWRES_PE_BLOCKSIZE = 32; // 低分辨率路径估算器(PE)的块大小 (32x32 方格)。

// 每次更新时，增量更新的地图方格数量上限。
static constexpr unsigned int SQUARES_TO_UPDATE = 8000;
// 在进行路径优化时，允许搜索的最大节点数。
static constexpr unsigned int MAX_SEARCHED_NODES_ON_REFINE = 8000;

// 路径热力图的缩放比例。
static constexpr unsigned int PATH_HEATMAP_XSCALE =  1; // wrt. mapDims.hmapx // 相对于 mapDims.hmapx
static constexpr unsigned int PATH_HEATMAP_ZSCALE =  1; // wrt. mapDims.hmapy // 相对于 mapDims.hmapy
// 路径流图的缩放比例。
static constexpr unsigned int PATH_FLOWMAP_XSCALE = 32; // wrt. mapDims.mapx // 相对于 mapDims.mapx
static constexpr unsigned int PATH_FLOWMAP_ZSCALE = 32; // wrt. mapDims.mapy // 相对于 mapDims.mapy


// PE-only flags (indices)
// 仅用于路径估算器(PE)的标志 (作为索引使用)。
static constexpr unsigned int PATHDIR_LEFT       = 0; // +x (LEFT *TO* RIGHT) // +x (从左到右)
static constexpr unsigned int PATHDIR_LEFT_UP    = 1; // +x+z
static constexpr unsigned int PATHDIR_UP         = 2; // +z (UP *TO* DOWN)   // +z (从上到下)
static constexpr unsigned int PATHDIR_RIGHT_UP   = 3; // -x+z
static constexpr unsigned int PATHDIR_RIGHT      = 4; // -x (RIGHT *TO* LEFT) // -x (从右到左)
static constexpr unsigned int PATHDIR_RIGHT_DOWN = 5; // -x-z
static constexpr unsigned int PATHDIR_DOWN       = 6; // -z (DOWN *TO* UP)   // -z (从下到上)
static constexpr unsigned int PATHDIR_LEFT_DOWN  = 7; // +x-z
static constexpr unsigned int PATH_DIRECTIONS    = 8; // 定义总的路径方向数量。


// Directional flags for direction masking
// 用于方向掩码的标志位。
static constexpr unsigned int PATHDIR_LEFT_MASK       = 0x01; // +x (LEFT *TO* RIGHT) // +x (从左到右)
static constexpr unsigned int PATHDIR_LEFT_UP_MASK    = 0x02; // +x+z
static constexpr unsigned int PATHDIR_UP_MASK         = 0x04; // +z (UP *TO* DOWN)   // +z (从上到下)
static constexpr unsigned int PATHDIR_RIGHT_UP_MASK   = 0x08; // -x+z
static constexpr unsigned int PATHDIR_RIGHT_MASK      = 0x10; // -x (RIGHT *TO* LEFT) // -x (从右到左)
static constexpr unsigned int PATHDIR_RIGHT_DOWN_MASK = 0x20; // -x-z
static constexpr unsigned int PATHDIR_DOWN_MASK       = 0x40; // -z (DOWN *TO* UP)   // -z (从下到上)
static constexpr unsigned int PATHDIR_LEFT_DOWN_MASK  = 0x80; // +x-z
static constexpr unsigned int PATH_DIRECTIONS_MASK    = 0xff; // 包含所有方向的掩码。

// see GetBlockVertexOffset(); costs are bi-directional and only
// calculated for *half* the outgoing edges (while costs for the
// other four directions are stored at the adjacent vertices)
// This mask covers the four outgoing edges any given block will
// update during block vertex cost calculations.
// 见 GetBlockVertexOffset() 函数；成本是双向的，因此只为*一半*的出边计算成本
// (另外四个方向的成本存储在相邻的顶点中)。
// 这个掩码覆盖了任何给定块在计算块顶点成本时将要更新的四个出边。
static constexpr unsigned int PATH_DIRECTIONS_HALF_MASK = 0x0f;


// 定义四个基本方向（上、下、左、右）的索引数组。
static constexpr unsigned int PATHDIR_CARDINALS[4] = {PATHDIR_LEFT, PATHDIR_RIGHT, PATHDIR_UP, PATHDIR_DOWN};
// 定义路径方向顶点的数量，等于总方向数。
static constexpr unsigned int PATH_DIRECTION_VERTICES = PATH_DIRECTIONS;
// 定义高精度寻路时节点间的采样间距。
static constexpr unsigned int PATH_NODE_SPACING = 2;

// note: because the spacing between nodes is 2 (not 1) we
// must also make sure not to skip across any intermediate
// impassable squares (!) but without reducing the spacing
// factor which would drop performance four-fold --> messy
// 注意: 因为节点间的间距是2 (而不是1)，我们必须确保不会跳过任何中间的
// 不可通行方格 (!)，但又不能减小间距因子，否则性能会下降四倍 --> 很麻烦。
static_assert(PATH_NODE_SPACING == 2, "");


// these give the changes in (x, z) coors
// when moving one step in given direction
//
// NOTE: the choices of +1 for LEFT and UP are *not* arbitrary
// (they are related to GetBlockVertexOffset) and also need to
// be consistent with the PATHOPT_* flags (for PathDir2PathOpt)
// 这些值给出了在给定方向上移动一步时 (x, z) 坐标的变化。
//
// 注意: 为 LEFT 和 UP 选择 +1 并*不是*任意的
// (它们与 GetBlockVertexOffset 相关)，并且还需要
// 与 PATHOPT_* 标志保持一致 (用于 PathDir2PathOpt)。
static constexpr int2 PE_DIRECTION_VECTORS[] = {
	{+1,  0}, // PATHDIR_LEFT
	{+1, +1}, // PATHDIR_LEFT_UP
	{ 0, +1}, // PATHDIR_UP
	{-1, +1}, // PATHDIR_RIGHT_UP
	{-1,  0}, // PATHDIR_RIGHT
	{-1, -1}, // PATHDIR_RIGHT_DOWN
	{ 0, -1}, // PATHDIR_DOWN
	{+1, -1}, // PATHDIR_LEFT_DOWN
};

//FIXME why not use PATHDIR_* consts and merge code with top one
//FIXME 为什么不使用 PATHDIR_* 常量并与上面的代码合并？
static constexpr int2 PF_DIRECTION_VECTORS_2D[] = {
	{ 0,                           0                         },
	{+1 * int(PATH_NODE_SPACING),  0 * int(PATH_NODE_SPACING)}, // PATHOPT_LEFT
	{-1 * int(PATH_NODE_SPACING),  0 * int(PATH_NODE_SPACING)}, // PATHOPT_RIGHT
	{ 0,                           0                         }, // PATHOPT_LEFT | PATHOPT_RIGHT
	{ 0 * int(PATH_NODE_SPACING), +1 * int(PATH_NODE_SPACING)}, // PATHOPT_UP
	{+1 * int(PATH_NODE_SPACING), +1 * int(PATH_NODE_SPACING)}, // PATHOPT_LEFT | PATHOPT_UP
	{-1 * int(PATH_NODE_SPACING), +1 * int(PATH_NODE_SPACING)}, // PATHOPT_RIGHT | PATHOPT_UP
	{ 0,                           0                         }, // PATHOPT_LEFT | PATHOPT_RIGHT | PATHOPT_UP
	{ 0 * int(PATH_NODE_SPACING), -1 * int(PATH_NODE_SPACING)}, // PATHOPT_DOWN
	{+1 * int(PATH_NODE_SPACING), -1 * int(PATH_NODE_SPACING)}, // PATHOPT_LEFT | PATHOPT_DOWN
	{-1 * int(PATH_NODE_SPACING), -1 * int(PATH_NODE_SPACING)}, // PATHOPT_RIGHT | PATHOPT_DOWN
	{ 0,                           0                         },
	{ 0,                           0                         },
	{ 0,                           0                         },
	{ 0,                           0                         },
	{ 0,                           0                         },
};


// PF and PE flags (used in nodeMask[])
// 用于高精度寻路(PF)和路径估算器(PE)的标志 (在 nodeMask[] 中使用)。
// 表示从左边的节点移动到了当前节点。
static constexpr unsigned int PATHOPT_LEFT      =   1; // +x
// 表示从右边的节点移动到了当前节点。
static constexpr unsigned int PATHOPT_RIGHT     =   2; // -x
// 表示从上方的节点移动到了当前节点。
static constexpr unsigned int PATHOPT_UP        =   4; // +z
// 表示从下方的节点移动到了当前节点。
static constexpr unsigned int PATHOPT_DOWN      =   8; // -z
// 表示该节点位于“开放列表”（Open Set）中。这意味着它已经被发现，但它的邻居节点尚未被完全探索。
static constexpr unsigned int PATHOPT_OPEN      =  16;
// 表示该节点位于“关闭列表”（Closed Set）中。这意味着它已经被探索过，算法不会再次访问它。
static constexpr unsigned int PATHOPT_CLOSED    =  32;
// 表示该节点不可通行。寻路算法会完全忽略被标记为阻塞的节点。
static constexpr unsigned int PATHOPT_BLOCKED   =  64;
// 表示该节点的数据已过时。这通常是因为地图发生了变化（例如地形被炸毁）。带有此标志的节点在被访问时，其相关的预计算数据需要被重新计算。
static constexpr unsigned int PATHOPT_OBSOLETE  = 128;
// 这个值 (二进制的 11111111) 并不是一个状态标志，而是定义了这套8位掩码所能表示的最大值，可用于验证或检查。
static constexpr unsigned int PATHOPT_SIZE      = 255; // size of PATHOPT bitmask // PATHOPT 位掩码的大小。
// 它的主要用途是作为一个掩码，用于从一个节点的 nodeMask 状态值中提取出方向信息。
static constexpr unsigned int PATHOPT_CARDINALS = (PATHOPT_RIGHT | PATHOPT_LEFT | PATHOPT_UP | PATHOPT_DOWN);

// 从路径方向索引(PATHDIR_*)到路径选项位掩码(PATHOPT_*)的转换查询表。
static constexpr unsigned int DIR2OPT[] = {
	(PATHOPT_LEFT                ),
	(PATHOPT_LEFT  | PATHOPT_UP  ),
	(                PATHOPT_UP  ),
	(PATHOPT_RIGHT | PATHOPT_UP  ),
	(PATHOPT_RIGHT               ),
	(PATHOPT_RIGHT | PATHOPT_DOWN),
	(PATHOPT_DOWN                ),
	(PATHOPT_LEFT  | PATHOPT_DOWN),
};


// 从路径选项位掩码(PATHOPT_*)到路径方向索引(PATHDIR_*)的转换查询表。
static constexpr unsigned int OPT2DIR[] = {
	0,
	PATHDIR_LEFT,       // PATHOPT_LEFT
	PATHDIR_RIGHT,      // PATHOPT_RIGHT
	0,                  // PATHOPT_LEFT  | PATHOPT_RIGHT
	PATHDIR_UP,         // PATHOPT_UP
	PATHDIR_LEFT_UP,    // PATHOPT_LEFT  | PATHOPT_UP
	PATHDIR_RIGHT_UP,   // PATHOPT_RIGHT | PATHOPT_UP
	0,                  // PATHOPT_LEFT  | PATHOPT_RIGHT | PATHOPT_UP
	PATHDIR_DOWN,       // PATHOPT_DOWN
	PATHDIR_LEFT_DOWN,  // PATHOPT_LEFT  | PATHOPT_DOWN
	PATHDIR_RIGHT_DOWN, // PATHOPT_RIGHT | PATHOPT_DOWN
	0,
	0,
	0,
	0,
	0,
};

// converts a PATHDIR* index to a PATHOPT* bitmask and vice versa
// 将 PATHDIR* 索引转换为 PATHOPT* 位掩码，反之亦然。
static constexpr unsigned int PathDir2PathOpt(unsigned int pathDir)    { return DIR2OPT[pathDir]; }
static constexpr unsigned int PathOpt2PathDir(unsigned int pathOptDir) { return OPT2DIR[pathOptDir]; }


// transition costs between vertices are bi-directional
// (cost(A-->B) == cost(A<--B)) so we only need to store
// (PATH_DIRECTIONS >> 1) values
// 顶点之间的转移成本是双向的
// (cost(A-->B) == cost(A<--B))，所以我们只需要存储
// (PATH_DIRECTIONS >> 1) 个值。
static inline int GetBlockVertexOffset(const MoveDef& md, unsigned int pathDir, unsigned int numBlocks) {
	int bvo = pathDir;

	if (!md.allowDirectionalPathing) {
		switch (pathDir) {
			case PATHDIR_RIGHT:      { bvo = int(PATHDIR_LEFT    ) -                                         PATH_DIRECTION_VERTICES; } break;
			case PATHDIR_RIGHT_DOWN: { bvo = int(PATHDIR_LEFT_UP ) - (numBlocks * PATH_DIRECTION_VERTICES) - PATH_DIRECTION_VERTICES; } break;
			case PATHDIR_DOWN:       { bvo = int(PATHDIR_UP      ) - (numBlocks * PATH_DIRECTION_VERTICES)                          ; } break;
			case PATHDIR_LEFT_DOWN:  { bvo = int(PATHDIR_RIGHT_UP) - (numBlocks * PATH_DIRECTION_VERTICES) + PATH_DIRECTION_VERTICES; } break;
			default: {} break;
		}
	}

	return bvo;
}

// 定义用于访问成本数组的索引枚举。
enum {
	NODE_COST_F = 0, // 节点的F值 (G+H)
	NODE_COST_G = 1, // 节点的G值 (从起点到此节点的实际成本)
	NODE_COST_H = 2, // 节点的H值 (从此节点到终点的估算成本)
};

#endif