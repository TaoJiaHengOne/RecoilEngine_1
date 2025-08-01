/* 本文件是 Spring 引擎的一部分 (遵循 GPL v2 或更高版本协议)，详见 LICENSE.html */

#ifndef PATH_DATATYPES_H // 防止头文件被重复包含
#define PATH_DATATYPES_H

#include <queue>
#include <vector>
#include <algorithm> // 用于 std::fill

#include "PathConstants.h"
#include "System/type2.h"
#include <cinttypes>

#include "System/Cpp11Compat.hpp"

/// @brief 路径节点
/// 代表A*搜索图中的一个节点，可以是一个单独的方格（用于PF高精度寻路）或一个方格块（用于PE低精度估算）。
struct PathNode {
	PathNode()
		: fCost(0.0f)
		, gCost(0.0f)
		, nodeNum(0)
		, nodePos(0, 0)
		, exitOnly(false)
	{}

	float fCost; // 节点的F值 (gCost + hCost)
	float gCost; // 节点的G值 (从起点到此节点的实际成本)

	int nodeNum;     // 节点的一维索引
	ushort2 nodePos; // 节点的二维坐标
	bool exitOnly;   // 节点是否位于“仅允许离开”的区域

	// 重载比较运算符，用于优先队列排序
	inline bool operator <  (const PathNode& pn) const { return (fCost < pn.fCost); }
	inline bool operator >  (const PathNode& pn) const { return (fCost > pn.fCost); }
	// 重载相等运算符，用于节点比较
	inline bool operator == (const PathNode& pn) const { return (nodeNum == pn.nodeNum); }
};


/// @brief 定义节点优先级的函子
/// 需要保证排序是稳定的。
struct lessCost {
	inline bool operator() (const PathNode* lhs, const PathNode* rhs) const {
		// fCost = gCost + hCost.
		// 当 fCost 相同时，优先选择离目标更近的节点。因为我们手头没有 hCost，
		// 但知道 hCost = fCost - gCost，所以当 fCost 相同时，gCost 越大意味着 hCost 越小。
		// 这就是为什么我们在比较 gCost 时要反转左右两侧的原因。最后使用 nodeNum 作为稳定排序的决胜局。
		return std::tie(lhs->fCost, rhs->gCost, lhs->nodeNum) > std::tie(rhs->fCost, lhs->gCost, rhs->nodeNum);
	}
};


/**
 * @brief 路径节点缓冲池
 * 一个固定大小的内存池，用于存储 PathNode 对象，以避免在搜索过程中频繁进行动态内存分配。
 */
struct PathNodeBuffer {
public:
	PathNodeBuffer() { Clear(); }

	// 清空缓冲池
	void Clear() {
		for (unsigned int i = 0; i < MAX_SEARCHED_NODES; i++) {
			buffer[i] = {};
		}

		SetSize(0);
	}

	// 设置当前已使用的节点数量
	void SetSize(unsigned int i) { idx = i; }
	// 获取当前已使用的节点数量
	unsigned int GetSize() const { return idx; }

	// 获取指定索引处的节点指针
	const PathNode* GetNode(unsigned int i) const { return &buffer[i]; }
	      PathNode* GetNode(unsigned int i)       { return &buffer[i]; }

private:
	/// 最近添加的节点的索引
	unsigned int idx = 0;

	// 实际的节点存储数组
	PathNode buffer[MAX_SEARCHED_NODES];
};


/**
 * @brief 路径节点状态缓冲区
 * 这是一个核心数据结构，为特定寻路器实例的网格中的每一个节点（方格或块）存储其状态信息（如成本、掩码等）。
 */
struct PathNodeStateBuffer {
	PathNodeStateBuffer() {
		#if !defined(_MSC_FULL_VER) || _MSC_FULL_VER > 180040000 // 确保 ::max() 是 constexpr
		// 编译时断言：确保 nodeMask 的基础类型足够大，能够容纳所有的 PATHOPT 位掩码
		static_assert(PATHOPT_SIZE <= std::numeric_limits<std::uint8_t>::max(), "nodeMask a basic type too small to hold PATHOPT bitmask");
		#endif

		Clear();
	}

	// 禁止拷贝构造，因为这个对象非常大，拷贝成本极高
	PathNodeStateBuffer(const PathNodeStateBuffer& pnsb) = delete;
	// 允许移动构造
	PathNodeStateBuffer(PathNodeStateBuffer&& pnsb) { *this = std::move(pnsb); }

	// 禁止拷贝赋值
	PathNodeStateBuffer& operator = (const PathNodeStateBuffer& pnsb) = delete;
	// 允许移动赋值，通过转移所有权来高效地移动数据
	PathNodeStateBuffer& operator = (PathNodeStateBuffer&& pnsb) {
		fCost = std::move(pnsb.fCost);
		gCost = std::move(pnsb.gCost);

		nodeMask = std::move(pnsb.nodeMask);
		nodeLinksObsoleteFlags = std::move(pnsb.nodeLinksObsoleteFlags);
		peNodeOffsets = std::move(pnsb.peNodeOffsets);

		extraCosts[ true] = std::move(pnsb.extraCosts[ true]);
		extraCosts[false] = std::move(pnsb.extraCosts[false]);

		extraCostsOverlay[ true] = pnsb.extraCostsOverlay[ true];
		extraCostsOverlay[false] = pnsb.extraCostsOverlay[false];

		pnsb.extraCostsOverlay[ true] = nullptr;
		pnsb.extraCostsOverlay[false] = nullptr;

		maxCosts[NODE_COST_F] = pnsb.maxCosts[NODE_COST_F];
		maxCosts[NODE_COST_G] = pnsb.maxCosts[NODE_COST_G];
		maxCosts[NODE_COST_H] = pnsb.maxCosts[NODE_COST_H];

		ps = pnsb.ps;
		br = pnsb.br;
		mr = pnsb.mr;

		er[ true] = pnsb.er[ true];
		er[false] = pnsb.er[false];
		return *this;
	}


	// 获取缓冲区的大小（节点总数）
	unsigned int GetSize() const { return fCost.size(); }

	// 根据缓冲区分辨率和地图分辨率调整内部各向量的大小
	/// @param bufRes 寻路器工作网格的分辨率。例如，对于一个在1024x1024地图上工作的BLOCK_SIZE=16的路径估算器，bufRes 就是 (64, 64)。
	/// @param mapRes  (Map Resolution): 游戏地图的完整精细分辨率，例如 (1024, 1024)。
	/// @note 这个函数会清空所有内部向量，并重新分
	void Resize(const int2& bufRes, const int2& mapRes) {
		// 计算出每个缓冲区格子对应多少个地图格子。在上面的例子中，ps 会是 (1024/64, 1024/64) = (16, 16)，这正好等于 BLOCK_SIZE
		ps = mapRes / bufRes;
		// br (Buffer Resolution): 直接缓存传入的缓冲区分辨率
		br = bufRes;
		// (Map Resolution): 直接缓存传入的地图分辨率
		mr = mapRes;

		fCost.resize(br.x * br.y, PATHCOST_INFINITY);
		gCost.resize(br.x * br.y, PATHCOST_INFINITY);
		// 量的大小，并将每个节点的初始掩码设置为0。这个掩码用于存储节点的状态（如开放、关闭、阻塞等），0代表默认的“未访问”状态。
		nodeMask.resize(br.x * br.y, 0);
		// 调整 nodeLinksObsoleteFlags 向量的大小，并将初始值设为0。这个标志用于标记节点之间的连接是否因地形变化而失效
		nodeLinksObsoleteFlags.resize(br.x * br.y, 0);

		// 按需创建
		// extraCosts[ true].resize(br.x * br.y, 0.0f);
		// extraCosts[false].resize(br.x * br.y, 0.0f);

		#if 0
		// 这部分在 PathEstimator 中完成，PF 不需要这些
		if (bufRes != mapRes)
			peNodeOffsets.resize(numPathTypes);
		#endif
	}

	// 清空所有内部向量，释放内存
	void Clear() {
		fCost.clear();
		gCost.clear();

		nodeMask.clear();
		nodeLinksObsoleteFlags.clear();
		peNodeOffsets.clear();

		extraCosts[ true].clear();
		extraCosts[false].clear();

		extraCostsOverlay[ true] = nullptr;
		extraCostsOverlay[false] = nullptr;

		maxCosts[NODE_COST_F] = 0.0f;
		maxCosts[NODE_COST_G] = 0.0f;
		maxCosts[NODE_COST_H] = 0.0f;

		ps        = {0, 0};
		br        = {0, 0};
		mr        = {0, 0};
		er[ true] = {1, 1};
		er[false] = {1, 1};
	}

	// 重置单个节点的状态，用于高效清理
	void ClearSquare(int idx) {
		// assert(idx >= 0 && idx < fCost.size());
		fCost[idx] = PATHCOST_INFINITY;
		gCost[idx] = PATHCOST_INFINITY;
		// 清除所有位，除了 PATHOPT_OBSOLETE（过时）标志
		nodeMask[idx] &= PATHOPT_OBSOLETE;
	}


	/// 我们持有的已分配内存区域的大小（不包括 sizeof(*this)）
	unsigned int GetMemFootPrint() const {
		unsigned int memFootPrint = 0;

		if (!peNodeOffsets.empty())
			memFootPrint += (peNodeOffsets.size() * (sizeof(std::vector<short2>) + peNodeOffsets[0].size() * sizeof(short2)));

		memFootPrint += (nodeMask.size() * sizeof(decltype(nodeMask)::value_type));
		memFootPrint += (nodeLinksObsoleteFlags.size() * sizeof(decltype(nodeLinksObsoleteFlags)::value_type));
		memFootPrint += ((fCost.size() + gCost.size()) * sizeof(decltype(fCost)::value_type));
		memFootPrint += ((extraCosts[true].size() + extraCosts[false].size()) * sizeof(float));
		return memFootPrint;
	}

	// 设置/获取搜索过程中遇到的最大成本值（用于调试或归一化）
	void SetMaxCost(unsigned int t, float c) { maxCosts[t] = c; }
	float GetMaxCost(unsigned int t) const { return maxCosts[t]; }

	/// @param xhm 和 @param zhm 总是以高度图坐标传入
	// 获取指定坐标节点的额外成本（例如由Lua脚本动态设置的成本）
	float GetNodeExtraCost(unsigned int xhm, unsigned int zhm, bool synced) const {
		// 降采样因子
		const int2 dsf = {mr.x / er[synced].x, mr.y / er[synced].y};

		const float* eco = nullptr;
		const float* ecd = nullptr;

		// 如果有成本覆盖层，则优先使用
		if ((eco = extraCostsOverlay[synced]) != nullptr)
			return eco[ (zhm / dsf.y) * er[synced].x  +  (xhm / dsf.x) ];

		// 如果向量为空，则 data() 返回 null
		if ((ecd = extraCosts[synced].data()) != nullptr)
			return ecd[ (zhm / ps.y) * br.x  +  (xhm / ps.x) ];

		return 0.0f;
	}

	// 获取额外成本覆盖层的指针
	const float* GetNodeExtraCosts(bool synced) const {
		return extraCostsOverlay[synced];
	}

	// 设置单个节点的额外成本
	void SetNodeExtraCost(unsigned int xhm, unsigned int zhm, float cost, bool synced) {
		auto& ecv = extraCosts[synced];

		if (ecv.empty())
			ecv.resize(br.x * br.y, 0.0f); // 按需分配

		ecv[ (zhm / ps.y) * br.x  +  (xhm / ps.x) ] = cost;
	}

	// 设置一个完整的额外成本覆盖层
	void SetNodeExtraCosts(const float* costs, unsigned int sx, unsigned int sz, bool synced) {
		extraCostsOverlay[synced] = costs;

		er[synced].x = sx;
		er[synced].y = sz;
	}

public:
	// 存储所有节点 F 值的向量
	std::vector<float> fCost;
	// 存储所有节点 G 值的向量
	std::vector<float> gCost;

	/// 节点的位掩码，包含 PATHOPT_{OPEN, ..., OBSOLETE} 等标志
	std::vector<std::uint8_t> nodeMask;

	/// 标记节点的某些方向链接为过时的标志
	std::vector<std::uint8_t> nodeLinksObsoleteFlags;

	/// 用于路径估算器(PE)，为每种路径类型维护一个数组，
	/// 存储每个块内最佳可达点的偏移量（相对于块中心）。
	/// 格式: peNodeOffsets[pathType][blockIdx]
	std::vector< std::vector<short2> > peNodeOffsets;

private:
	// 节点的覆盖成本修正值（如果不为零，会修改 GetPath() 和 GetNextWaypoint() 的行为）
	//
	// <extraCosts[false]> (非同步成本) 不能在同步代码中读取，
	// 因为AI和非同步Lua对其有写权限。
	// <extraCosts[true]> (同步成本) 不能在任何非同步代码中写入。
	//
	// 注意：如果激活了多个本地AI实例，
	// 每个实例必须撤销其更改，否则这些更改将对其他AI可见。
	// 注意：非公开，按需创建。
	std::vector<float> extraCosts[2];

	// 如果非空，这些指针将覆盖对应的 extraCosts 向量
	// (注意它们可以有介于 1 和 mapDims.map{x,y} 之间的任意分辨率)
	const float* extraCostsOverlay[2];

private:
	// 搜索过程中的最大成本记录
	float3 maxCosts;

	// 分辨率相关的参数
	int2 ps;    ///< 补丁大小 (例如 PF 为 1, PE 为 BLOCK_SIZE)；当 extraCosts 不为 NULL 时忽略
	int2 br;    ///< 缓冲区分辨率 (等于 mr / ps)；当 extraCosts 不为 NULL 时忽略
	int2 mr;    ///< 高度图分辨率 (等于 mapDims.map{x,y})
	int2 er[2]; ///< extraCosts 的分辨率
};



// 看起来像 std::vector，但内部持有一个固定大小的缓冲区。
// 用作 PathPriorityQueue 数据类型的底层数组。
class PathVector {
public:
	typedef int size_type;
	typedef PathNode* value_type;
	typedef PathNode* reference;
	typedef PathNode** iterator;
	typedef const PathNode* const_reference;
	typedef const PathNode* const* const_iterator;

		// gcc 4.3 需要这些概念定义，所以提供它们
		value_type& operator [] (size_type idx) { return buf[idx]; }
		const value_type& operator [] (size_type idx) const { return buf[idx]; }

		typedef iterator pointer;
		typedef const_iterator const_pointer;
		typedef int difference_type;

		typedef PathNode** reverse_iterator;
		typedef const PathNode* const* const_reverse_iterator;

		// 概念兼容的hack：不要使用这些函数，它们会使程序中止
		reverse_iterator rbegin() { return 0; }
		reverse_iterator rend() { return 0; }
		const_reverse_iterator rbegin() const { return 0; }
		const_reverse_iterator rend() const { return 0; }
		PathVector(int, const value_type&): bufPos(-1) { abort(); }
		PathVector(iterator, iterator): bufPos(-1) { abort(); }
		void insert(iterator, const value_type&) { abort(); }
		void insert(iterator, const size_type&, const value_type&) { abort(); }
		void insert(iterator, iterator, iterator) { abort(); }
		void erase(iterator, iterator) { abort(); }
		void erase(iterator) { abort(); }
		void erase(iterator, iterator, iterator) { abort(); }
		void swap(PathVector&) { abort(); }
	// 概念hack结束

	PathVector(): bufPos(-1) {
#ifdef DEBUG
		// 出于性能原因，仅在DEBUG构建中执行此操作
		// 这有助于发现逻辑错误
		std::fill(std::begin(buf), std::end(buf), nullptr);
#endif
	}

	// 模拟 std::vector 的接口
	inline void push_back(PathNode* os) { buf[++bufPos] = os; }
	inline void pop_back() { --bufPos; }
	inline PathNode* back() const { return buf[bufPos]; }
	inline const value_type& front() const { return buf[0]; }
	inline value_type& front() { return buf[0]; }
	inline bool empty() const { return (bufPos < 0); }
	inline size_type size() const { return bufPos + 1; }
	inline size_type max_size() const { return (1 << 30); }
	inline iterator begin() { return &buf[0]; }
	inline iterator end() { return &buf[bufPos + 1]; }
	inline const_iterator begin() const { return &buf[0]; }
	inline const_iterator end() const { return &buf[bufPos + 1]; }
	inline void clear() { bufPos = -1; }

private:
	// 缓冲区当前位置
	int bufPos;

	// 实际的指针数组
	PathNode* buf[MAX_SEARCHED_NODES];
};


/**
 * @brief 路径优先队列
 * 继承自 std::priority_queue，用于A*算法的开放集合（Open Set）。
 */
class PathPriorityQueue: public std::priority_queue<PathNode*, PathVector, lessCost> {
public:
	/// 比 "while (!q.empty()) { q.pop(); }" 更快
	void Clear() { c.clear(); }
};

#endif // PATH_DATATYPES_H