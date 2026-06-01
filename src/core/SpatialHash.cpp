#include "SpatialHash.hpp"
#include <algorithm>
#include <cmath>

SpatialHash::SpatialHash(float cellSize, int tableSize)
    : m_cellSize(cellSize), m_tableSize(tableSize) {
    m_cellStart.resize(m_tableSize + 1, 0);
}

void SpatialHash::clear() {
    m_entries.clear();
    std::fill(m_cellStart.begin(), m_cellStart.end(), 0);
}

void SpatialHash::getGridCoords(const Vec2f& pos, int& gx, int& gy) const {
    gx = static_cast<int>(std::floor(pos.x / m_cellSize));
    gy = static_cast<int>(std::floor(pos.y / m_cellSize));
}

int SpatialHash::getHashKey(int gx, int gy) const {
    // 使用大质数对网格坐标进行哈希混合，并转换为无符号数以避免负数模运算
    unsigned int h = (static_cast<unsigned int>(gx) * 73856093) ^ 
                     (static_cast<unsigned int>(gy) * 19349663);
    return static_cast<int>(h % static_cast<unsigned int>(m_tableSize));
}

int SpatialHash::getHashKeyForPosition(const Vec2f& pos) const {
    int gx, gy;
    getGridCoords(pos, gx, gy);
    return getHashKey(gx, gy);
}

void SpatialHash::build(const float* posX, const float* posY, int count) {
    if (count == 0) {
        clear();
        return;
    }

    m_entries.resize(count);
    
    // 1. 计数：统计每个哈希键拥拥有多少个粒子
    std::fill(m_cellStart.begin(), m_cellStart.end(), 0);
    for (int i = 0; i < count; ++i) {
        Vec2f p(posX[i], posY[i]);
        int key = getHashKeyForPosition(p);
        m_cellStart[key]++;
        m_entries[i] = { key, i, p };
    }

    // 2. 前缀和：计算每个键在排序数组中的起始绝对索引位置
    int start = 0;
    for (int i = 0; i < m_tableSize; ++i) {
        int countInCell = m_cellStart[i];
        m_cellStart[i] = start;
        start += countInCell;
    }
    m_cellStart[m_tableSize] = start; // 哨兵节点

    // 3. 排序：利用计数排序，在 O(N) 线性时间内完成位置归档
    std::vector<Entry> sortedEntries(count);
    std::vector<int> currentOffsets = m_cellStart; // 复制当前偏移量

    for (int i = 0; i < count; ++i) {
        int key = m_entries[i].key;
        int destIndex = currentOffsets[key]++;
        sortedEntries[destIndex] = m_entries[i];
    }

    m_entries = std::move(sortedEntries);
}

void SpatialHash::build(const Vec2f* positions, int count) {
    if (count == 0) {
        clear();
        return;
    }

    m_entries.resize(count);
    
    std::fill(m_cellStart.begin(), m_cellStart.end(), 0);
    for (int i = 0; i < count; ++i) {
        int key = getHashKeyForPosition(positions[i]);
        m_cellStart[key]++;
        m_entries[i] = { key, i, positions[i] };
    }

    int start = 0;
    for (int i = 0; i < m_tableSize; ++i) {
        int countInCell = m_cellStart[i];
        m_cellStart[i] = start;
        start += countInCell;
    }
    m_cellStart[m_tableSize] = start;

    std::vector<Entry> sortedEntries(count);
    std::vector<int> currentOffsets = m_cellStart;

    for (int i = 0; i < count; ++i) {
        int key = m_entries[i].key;
        int destIndex = currentOffsets[key]++;
        sortedEntries[destIndex] = m_entries[i];
    }

    m_entries = std::move(sortedEntries);
}

void SpatialHash::build(const std::vector<Vec2f>& positions) {
    build(positions.data(), static_cast<int>(positions.size()));
}

void SpatialHash::query(const Vec2f& pos, float searchRadius, 
                        const float* posX, const float* posY, std::vector<int>& outNeighbors, int maxNeighbors) const {
    outNeighbors.clear();
    if (m_entries.empty()) return;

    // 计算查询范围的外包矩形网格坐标
    int minX, minY, maxX, maxY;
    getGridCoords(pos - Vec2f(searchRadius, searchRadius), minX, minY);
    getGridCoords(pos + Vec2f(searchRadius, searchRadius), maxX, maxY);

    float radiusSq = searchRadius * searchRadius;

    // 遍历外包范围内的网格单元
    for (int gy = minY; gy <= maxY; ++gy) {
        for (int gx = minX; gx <= maxX; ++gx) {
            int key = getHashKey(gx, gy);
            
            int start = m_cellStart[key];
            int end = m_cellStart[key + 1];

            // 遍历单元格对应的桶内元素
            for (int i = start; i < end; ++i) {
                int pIdx = m_entries[i].index;
                // Entry 内部已经缓存了位置（m_entries[i].pos），直接读取以最大化 Cache 命中率
                float distSq = m_entries[i].pos.distanceSquared(pos);
                if (pIdx >= 0 && distSq <= radiusSq) {
                    outNeighbors.push_back(pIdx);
                    if (outNeighbors.size() >= static_cast<size_t>(maxNeighbors)) {
                        return;
                    }
                }
            }
        }
    }
}

void SpatialHash::query(const Vec2f& pos, float searchRadius, 
                        const Vec2f* positions, std::vector<int>& outNeighbors, int maxNeighbors) const {
    query(pos, searchRadius, nullptr, nullptr, outNeighbors, maxNeighbors);
}

void SpatialHash::query(const Vec2f& pos, float searchRadius, 
                        const std::vector<Vec2f>& positions, std::vector<int>& outNeighbors, int maxNeighbors) const {
    query(pos, searchRadius, nullptr, nullptr, outNeighbors, maxNeighbors);
}
