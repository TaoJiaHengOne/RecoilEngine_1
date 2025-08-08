# PathManager中央管理器

## 目录

1. [核心职责](#1-核心职责)
2. [分层路径调度](#2-分层路径调度)
3. [路径生命周期管理](#3-路径生命周期管理)
4. [多线程架构](#4-多线程架构)
5. [性能优化](#5-性能优化)

---

## 1. 核心职责

PathManager是HAPFS系统的核心调度器，负责：

- **智能路径分发**: 根据距离选择最优搜索策略
- **分层路径细化**: 将宏观路径转换为精确路径
- **资源管理**: 管理所有寻路器实例和内存资源
- **多线程协调**: 调度并行路径计算任务

## 2. 分层路径调度

### 2.1 路径请求分发策略

```cpp
IPath::SearchResult CPathManager::ArrangePath(
    MultiPath* newPath,
    const MoveDef* moveDef,
    float3 startPos,
    float3 goalPos,
    float goalRadius,
    bool synced
) {
    // 计算启发式距离
    float heurGoalDist2D = pfDef->Heuristic(startPos.x, startPos.z, 
                                           goalPos.x, goalPos.z, BLOCK_SIZE);
    
    IPath::SearchResult result = IPath::Error;
    
    // 1. 尝试原始路径(直线检测)
    if (heurGoalDist2D <= MAXRES_SEARCH_DISTANCE * modInfo.pfRawDistMult) {
        result = maxResPF->DoRawSearch(moveDef, pfDef, owner, startPos, path);
        if (result == IPath::Ok) return result;
    }
    
    // 2. 按分辨率从高到低尝试
    for (int resLevel = PATH_MAX_RES; resLevel >= PATH_LOW_RES; resLevel--) {
        if (heurGoalDist2D <= searchDistances[resLevel]) {
            if (resLevel == PATH_MAX_RES) {
                result = maxResPF->GetPath(moveDef, pfDef, owner, startPos, path);
            } else {
                CPathEstimator* pe = GetPathEstimator(resLevel);
                result = pe->GetPath(moveDef, pfDef, owner, startPos, path);
            }
            
            if (result == IPath::Ok) break;
        }
    }
    
    // 3. 最后尝试无约束搜索
    if (result != IPath::Ok && heurGoalDist2D > MEDRES_SEARCH_DISTANCE) {
        result = medResPE->GetPath(moveDef, pfDef, owner, startPos, path);
    }
    
    return result;
}
```

### 2.2 路径细化算法

```cpp
void CPathManager::LowRes2MaxRes(
    MultiPath& multiPath,
    const MoveDef& moveDef, 
    const CSolidObject* owner,
    float3 startPos,
    bool synced
) {
    // 选择细化源路径
    IPath::Path& lowResPath = multiPath.medResPath.path.size() > 0 ? 
                             multiPath.medResPath : multiPath.lowResPath;
    
    // 移除过近的路径点
    while (!lowResPath.path.empty() && 
           startPos.SqDistance2D(lowResPath.path.back()) < 
           Square(MAXRES_SEARCH_DISTANCE_EXT)) {
        lowResPath.path.pop_back();
    }
    
    // 确定细化目标
    float3 goalPos = lowResPath.path.empty() ? 
                     lowResPath.pathGoal : lowResPath.path.back();
    
    // 设置搜索约束
    CCircularSearchConstraint rangedGoalDef(
        startPos, goalPos, 0.0f, 2.0f, 
        Square(MAXRES_SEARCH_DISTANCE)
    );
    
    // 执行高分辨率细化搜索
    IPath::SearchResult result = maxResPF->GetPath(
        moveDef, rangedGoalDef, owner, startPos, 
        multiPath.maxResPath, MAX_SEARCHED_NODES_ON_REFINE
    );
    
    // 递归细化剩余路径
    if (!lowResPath.path.empty()) {
        LowRes2MaxRes(multiPath, moveDef, owner, goalPos, synced);
    }
}
```

## 3. 路径生命周期管理

### 3.1 MultiPath数据结构

```cpp
struct MultiPath {
    IPath::Path lowResPath;                // 32x32块路径
    IPath::Path medResPath;                // 16x16块路径  
    IPath::Path maxResPath;                // 1x1方格路径
    IPath::SearchResult searchResult;     // 搜索结果
    CCircularSearchConstraint peDef;       // 搜索定义
    const MoveDef* moveDef;                // 移动定义
    CSolidObject* caller;                  // 请求者
    unsigned int pathID;                   // 路径ID
    bool finalGoal;                        // 是否为最终目标
    float searchTime;                      // 搜索耗时
};
```

### 3.2 路径映射表管理

```cpp
class CPathManager {
private:
    std::map<unsigned int, MultiPath> pathMap;  // 路径映射表
    mutable std::mutex pathMapMutex;            // 线程安全保护
    unsigned int nextPathID = 1;               // 路径ID生成器
    
public:
    // 线程安全的路径访问
    MultiPath GetMultiPathMT(unsigned int pathID) const {
        std::lock_guard<std::mutex> lock(pathMapMutex);
        auto it = pathMap.find(pathID);
        return (it != pathMap.end()) ? it->second : MultiPath{};
    }
    
    void UpdateMultiPathMT(unsigned int pathID, const MultiPath& path) {
        std::lock_guard<std::mutex> lock(pathMapMutex);
        pathMap[pathID] = path;
    }
    
    void DeletePathMT(unsigned int pathID) {
        std::lock_guard<std::mutex> lock(pathMapMutex);
        pathMap.erase(pathID);
    }
};
```

## 4. 多线程架构

### 4.1 寻路器实例管理

```cpp
// 每线程独立的寻路器实例
void CPathManager::InitPathFinders() {
    const int numThreads = ThreadPool::GetMaxThreads();
    
    // 分配内存
    size_t totalMemory = CalculateMemoryRequirements(numThreads);
    char* baseAddr = reinterpret_cast<char*>(::operator new(totalMemory));
    
    // PathFinder实例 (每线程3个层级)
    maxResPFs = reinterpret_cast<CPathFinder*>(baseAddr + maxResPFsOffset);
    for (int i = 0; i < numThreads; ++i) {
        new (&maxResPFs[i]) CPathFinder(SQUARE_SIZE);
    }
    
    // PathEstimator实例 (每线程2个分辨率)
    lowResPEs = reinterpret_cast<CPathEstimator*>(baseAddr + lowResPEsOffset);
    medResPEs = reinterpret_cast<CPathEstimator*>(baseAddr + medResPEsOffset);
    
    for (int i = 0; i < numThreads; ++i) {
        new (&lowResPEs[i]) CPathEstimator(&pathingStates[LOWRES_PE], 
                                          LOWRES_PE_BLOCKSIZE, "pe", maxResPFs);
        new (&medResPEs[i]) CPathEstimator(&pathingStates[MEDRES_PE], 
                                          MEDRES_PE_BLOCKSIZE, "pe", maxResPFs);
    }
}
```

### 4.2 异步路径处理

```cpp
void CPathManager::Update() {
    SCOPED_TIMER("PathManager::Update");
    
    // 处理ECS路径请求
    ProcessAsyncPathRequests();
    
    // 更新PathingState
    UpdatePathingStates();
    
    // 清理过期路径
    CleanupExpiredPaths();
}

void CPathManager::ProcessAsyncPathRequests() {
    auto pathSearchView = HAPFS::registry.view<HAPFS::PathSearch>();
    
    if (pathSearchView.empty()) return;
    
    // 多线程批量处理
    for_mt(0, pathSearchView.size(), [&](int idx) {
        auto entity = pathSearchView[idx];
        auto& pathSearch = pathSearchView.get<HAPFS::PathSearch>(entity);
        
        // 执行路径搜索
        MultiPath newPath = ArrangePath(pathSearch);
        
        // 更新路径映射
        UpdateMultiPathMT(pathSearch.pathID, newPath);
        
        // 清理ECS组件
        HAPFS::registry.destroy(entity);
    });
}
```

## 5. 性能优化

### 5.1 工作负载均衡

```cpp
struct WorkloadBalance {
    float lowResRatio = 0.3f;     // 低分辨率更新权重
    float medResRatio = 0.5f;     // 中分辨率更新权重
    bool prioritizeHighRes = false; // 高分辨率优先标志
    
    void AdjustWorkload() {
        int2 queueSizes = GetNumQueuedUpdates();
        
        if (queueSizes.y > 2000) {        // 低分辨率队列过长
            lowResRatio = 0.6f;
            prioritizeHighRes = false;
        } else if (queueSizes.x > 1000) { // 中分辨率队列过长  
            medResRatio = 0.7f;
            prioritizeHighRes = true;
        } else {
            // 恢复默认权重
            lowResRatio = 0.3f;
            medResRatio = 0.5f;
            prioritizeHighRes = false;
        }
    }
};
```

### 5.2 内存管理优化

```cpp
// 统一内存分配避免碎片
void CPathManager::AllocateMemory() {
    // 计算总内存需求
    const size_t pathFinderMem = numThreads * sizeof(CPathFinder) * 3;
    const size_t pathEstimatorMem = numThreads * sizeof(CPathEstimator) * 2;
    const size_t pathingStateMem = sizeof(PathingState) * 2;
    
    const size_t totalMem = pathFinderMem + pathEstimatorMem + pathingStateMem;
    
    // 单次分配大块连续内存
    void* baseMemory = ::operator new(totalMem);
    memset(baseMemory, 0, totalMem);
    
    // 手动布局内存
    char* memPtr = static_cast<char*>(baseMemory);
    
    maxResPFs = reinterpret_cast<CPathFinder*>(memPtr);
    memPtr += pathFinderMem;
    
    lowResPEs = reinterpret_cast<CPathEstimator*>(memPtr);  
    memPtr += numThreads * sizeof(CPathEstimator);
    
    medResPEs = reinterpret_cast<CPathEstimator*>(memPtr);
    memPtr += numThreads * sizeof(CPathEstimator);
    
    // Placement New 就地构造对象
    for (int i = 0; i < numThreads; ++i) {
        new (&maxResPFs[i]) CPathFinder(SQUARE_SIZE);
        new (&lowResPEs[i]) CPathEstimator(/* ... */);
        new (&medResPEs[i]) CPathEstimator(/* ... */);
    }
}
```

### 5.3 性能监控

```cpp
struct PathManagerStats {
    unsigned int totalRequests = 0;
    unsigned int rawPathHits = 0;      // 原始路径命中
    unsigned int maxResRequests = 0;   // 高精度请求数
    unsigned int medResRequests = 0;   // 中精度请求数
    unsigned int lowResRequests = 0;   // 低精度请求数
    
    float avgSearchTime = 0.0f;
    float maxSearchTime = 0.0f;
    
    void LogStatistics() {
        float rawPathRate = float(rawPathHits) / totalRequests;
        
        LOG("PathManager Statistics:");
        LOG("  Total Requests: %u", totalRequests);
        LOG("  Raw Path Hit Rate: %.2f%%", rawPathRate * 100.0f);
        LOG("  Resolution Distribution:");
        LOG("    MaxRes: %u (%.1f%%)", maxResRequests, 
            100.0f * maxResRequests / totalRequests);
        LOG("    MedRes: %u (%.1f%%)", medResRequests,
            100.0f * medResRequests / totalRequests);
        LOG("    LowRes: %u (%.1f%%)", lowResRequests,
            100.0f * lowResRequests / totalRequests);
        LOG("  Average Search Time: %.2fms", avgSearchTime);
        LOG("  Maximum Search Time: %.2fms", maxSearchTime);
    }
};
```

PathManager通过智能的分层调度和高效的资源管理，为HAPFS系统提供了统一而强大的路径管理能力，是整个寻路系统的核心协调者。