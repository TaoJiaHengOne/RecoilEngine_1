/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#ifndef GROUNDBLOCKINGOBJECTMAP_H
#define GROUNDBLOCKINGOBJECTMAP_H

#include <array>
#include <vector>

#include "Sim/Objects/SolidObject.h"
#include "System/creg/creg_cond.h"
#include "System/float3.h"

class CGroundBlockingObjectMap
{
	CR_DECLARE_STRUCT(CGroundBlockingObjectMap)

private:
	template<typename T, uint32_t S = 8> struct ArrayCell {
	public:
		CR_DECLARE_STRUCT(ArrayCell)

		void Clear() {
			arr.fill(nullptr);

			numObjs = 0;
			vecIndx = 0; // point to dummy
		}

		bool InsertUnique(T* o) { return (!Contains(o) && Insert(o)); }
		bool Insert(T* o) {
			if (Full())
				return false;

			arr[numObjs++] = o;
			return true;
		}

		bool Erase(T* o) {
			const auto ae = arr.begin() + numObjs;
			const auto it = std::find(arr.begin(), ae, o);

			if (it == ae)
				return false;

			*it = arr[--numObjs];
			arr[numObjs] = nullptr;
			return true;
		}

		bool Empty() const { return (numObjs == 0); }
		bool Full() const { return (numObjs == S); }
		bool Contains(const T* o) const {
			const auto ab = arr.begin();
			const auto ae = arr.begin() + numObjs;

			return (std::find(ab, ae, o) != ae);
		}

		T* operator [] (size_t i) const { return arr[i]; }

		uint32_t GetNumObjs() const { return numObjs; }
		uint32_t GetVecIndx() const { return vecIndx; }
		uint32_t SetVecIndx(uint32_t i) { return (vecIndx = i); }
	
	private:
		uint32_t numObjs = 0;
		uint32_t vecIndx = 0;

		std::array<T*, S> arr;
	};

	typedef ArrayCell<CSolidObject> ArrCell;
	typedef std::vector<CSolidObject*> VecCell;

public:
	struct BlockingMapCell {
	public:
		BlockingMapCell() = delete;
		BlockingMapCell(const ArrCell& ac, const VecCell* vc): arrCell(ac), vecCell(vc) {
		}

		VecCell::value_type operator [] (size_t i) const {
			assert(i < size());
			assert(i < arrCell.GetNumObjs() || (i - arrCell.GetNumObjs()) < vecCell[ arrCell.GetVecIndx() ].size());

			return ((i < arrCell.GetNumObjs())? arrCell[i]: vecCell[ arrCell.GetVecIndx() ][ i - arrCell.GetNumObjs() ]);
		}

		size_t size() const { return (arrCell.GetNumObjs() + vsize()); }
		size_t vsize() const { return ((arrCell.Full())? vecCell[ arrCell.GetVecIndx() ].size(): 0); }

		bool empty() const { return (arrCell.Empty()); }

	private:
		const ArrCell& arrCell;
		const VecCell* vecCell;
	};


	void Init(unsigned int numSquares) {
		arrCells.resize(numSquares);
		vecCells.reserve(32);
		vecIndcs.reserve(32);

		// add dummy
		if (vecCells.empty())
			vecCells.emplace_back();
	}
	void Kill() {
		// reuse inner vectors when reloading
		// vecCells.clear();
		for (auto& v: arrCells) {
			v.Clear();
		}
		for (auto& v: vecCells) {
			v.clear();
		}

		vecIndcs.clear();
	}

	unsigned int CalcChecksum() const;

	void AddGroundBlockingObject(CSolidObject* object);
	void AddGroundBlockingObject(CSolidObject* object, const YardMapStatus& mask);
	void RemoveGroundBlockingObject(CSolidObject* object);

	void OpenBlockingYard(CSolidObject* object);
	void CloseBlockingYard(CSolidObject* object);
	bool CanOpenYard(const CSolidObject* object) const { return CheckYard(object, YARDMAP_YARDINV); }
	bool CanCloseYard(const CSolidObject* object) const { return CheckYard(object, YARDMAP_YARD); }


	// these retrieve either the first object in
	// a given cell, or NULL if the cell is empty
	CSolidObject* GroundBlocked(int x, int z) const;
	CSolidObject* GroundBlocked(const float3& pos) const;

	// same as GroundBlocked(), but does not bounds-check mapSquare
	CSolidObject* GroundBlockedUnsafe(unsigned int mapSquare) const {
		const BlockingMapCell& cell = GetCellUnsafeConst(mapSquare);

		if (cell.empty())
			return nullptr;

		return cell[0];
	}


	bool GroundBlocked(int x, int z, const CSolidObject* ignoreObj) const;
	bool GroundBlocked(const float3& pos, const CSolidObject* ignoreObj) const;

	bool ObjectInCell(unsigned int mapSquare, const CSolidObject* obj) const {
		if (mapSquare >= arrCells.size())
			return false;

		const ArrCell& ac = GetArrCell(mapSquare);
		const VecCell* vc = nullptr;

		if (ac.Contains(obj))
			return true;

		return (((vc = &GetVecCell(mapSquare)) != &vecCells[0]) && (std::find(vc->begin(), vc->end(), obj) != vc->end()));
	}


	BlockingMapCell GetCellUnsafeConst(const float3& pos) const;
	BlockingMapCell GetCellUnsafeConst(unsigned int mapSquare) const {
		assert(mapSquare < arrCells.size());
		// avoid vec-cell lookup unless needed
		return {GetArrCell(mapSquare), vecCells.data()};
	}

private:
	bool CheckYard(const CSolidObject* yardUnit, const YardMapStatus& mask) const;

	const ArrCell& GetArrCell(unsigned int mapSquare) const { return           arrCells[mapSquare]               ; }
	      ArrCell& GetArrCell(unsigned int mapSquare)       { return           arrCells[mapSquare]               ; }
	const VecCell& GetVecCell(unsigned int mapSquare) const { return vecCells[ arrCells[mapSquare].GetVecIndx() ]; }
	      VecCell& GetVecCell(unsigned int mapSquare)       { return vecCells[ arrCells[mapSquare].GetVecIndx() ]; }

	bool CellInsertUnique(unsigned int sqr, CSolidObject* o);
	bool CellErase(unsigned int sqr, CSolidObject* o);

private:
	std::vector<ArrCell> arrCells;
	std::vector<VecCell> vecCells;
	std::vector<uint32_t> vecIndcs;
};

extern CGroundBlockingObjectMap groundBlockingObjectMap;

#endif

/*
  GroundBlockingObjectMap 是Spring RTS引擎中的空间分割数据结构，它的核心作用是高效管理地图上所有阻挡移动的对象。以下是其详细工作原理：

  1. 核心设计理念

  空间分割（Spatial Partitioning）
  // 将地图划分为网格，每个网格存储该位置的阻挡对象列表
  std::vector<ArrCell> arrCells;  // 主数据结构，大小为 mapx * mapy

  地图被划分为与地形方格对应的网格系统，每个网格(Cell)存储所有占据该位置的固体对象指针。

  2. 混合存储优化策略

  ArrayCell + VectorCell 双层设计

  template<typename T, uint32_t S = 8> struct ArrayCell {
      std::array<T*, S> arr;        // 固定大小数组（默认8个）
      uint32_t numObjs = 0;         // 当前对象数量
      uint32_t vecIndx = 0;         // 溢出vector的索引
  };

  设计优势：
  - 常见情况优化：大多数网格包含0-8个对象，用固定数组避免动态分配开销
  - 极端情况处理：当对象超过8个时，溢出到动态vector中
  - 内存局部性：频繁访问的小数据在数组中，减少缓存未命中

  3. 溢出处理机制

  Vector池化复用
  std::vector<VecCell> vecCells;    // vector池，复用vector容器
  std::vector<uint32_t> vecIndcs;   // 可用vector索引栈

  溢出流程：
  1. ArrayCell满载(8个对象)时，从vecIndcs栈中取一个空闲vector索引
  2. 如果没有空闲索引，创建新的vector并添加到vecCells
  3. 后续对象插入到对应的vector中

  回收流程：
  1. 当vector变空时，将其索引压入vecIndcs栈
  2. ArrayCell的vecIndx重置为0（指向dummy vector）
  3. Vector容器被保留以供下次复用

  4. 统一访问接口

  BlockingMapCell包装器
  struct BlockingMapCell {
      const ArrCell& arrCell;
      const VecCell* vecCell;

      VecCell::value_type operator[](size_t i) const {
          return ((i < arrCell.GetNumObjs()) ?
                  arrCell[i] :
                  vecCell[arrCell.GetVecIndx()][i - arrCell.GetNumObjs()]);
      }
  };

  这个包装器让外部代码无需关心内部的混合存储结构，可以像访问普通数组一样访问所有对象。

  5. 对象管理操作

  添加对象流程
  void AddGroundBlockingObject(CSolidObject* object) {
      // 1. 计算对象占据的方格范围
      const int xminSqr = bx, xmaxSqr = bx + sx;
      const int zminSqr = bz, zmaxSqr = bz + sz;

      // 2. 在每个占据的方格中插入对象引用
      for (int zSqr = zminSqr; zSqr < zmaxSqr; zSqr++) {
          for (int xSqr = xminSqr; xSqr < xmaxSqr; xSqr++) {
              CellInsertUnique(zSqr * mapDims.mapx + xSqr, object);
          }
      }
  }

  庭院图（YardMap）支持
  - 支持复杂建筑的精细阻挡控制
  - YARDMAP_BLOCKED: 完全阻挡
  - YARDMAP_EXITONLY: 仅允许离开（工厂出口）
  - YARDMAP_UNBUILDABLE: 禁止建造

  6. 性能优化特性

  查询优化
  CSolidObject* GroundBlockedUnsafe(unsigned int mapSquare) const {
      const BlockingMapCell& cell = GetCellUnsafeConst(mapSquare);
      if (cell.empty()) return nullptr;
      return cell[0];  // 大多数情况只需要知道是否被阻挡
  }

  批量操作支持
  - 支持工厂开放/关闭yard的批量更新
  - 与路径管理器集成，自动通知地形变化

  7. 内存效率分析

  空间复杂度
  - 基础：mapx × mapy × sizeof(ArrayCell) ≈ mapx × mapy × 40字节
  - 对于1024×1024地图：约40MB基础开销
  - 额外vector仅在需要时分配

  时间复杂度
  - 插入/删除：O(k)，k为单个网格中的对象数（通常≤8）
  - 查询：O(1)访问网格，O(k)遍历对象列表
  - 范围查询：O(area × k)

  8. 与移动系统集成

  碰撞检测查询流程
  // 在RangeIsBlocked中的使用
  const CGroundBlockingObjectMap::BlockingMapCell& cell =
      groundBlockingObjectMap.GetCellUnsafeConst(zOffset + x);

  for (size_t i = 0, n = cell.size(); i < n; i++) {
      CSolidObject* collidee = cell[i];
      ret |= ObjectBlockType(collidee, collider);
  }

  这个设计使得Spring引擎能够高效处理数千个单位的实时碰撞检测，同时保持良好的内存使用效率。混合存储策略特别适合RTS游戏中"大部分区域稀疏，少数区域密集"的对象分布特点
*/