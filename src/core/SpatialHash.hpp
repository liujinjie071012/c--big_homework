#pragma once

#include "Math.hpp"
#include <vector>

class SpatialHash {
public:
    struct Entry {
        int key;        // 空间网格的哈希键值
        int index;      // 粒子/实体的原始索引
    };

    SpatialHash(float cellSize, int tableSize = 100003);

    // 清除并根据粒子位置列表重建哈希网格 (可接受数组指针，便于不同实体共用)
    void build(const Vec2f* positions, int count);
    void build(const std::vector<Vec2f>& positions);

    // 查询给定点周围一定半径内的所有粒子索引
    void query(const Vec2f& pos, float searchRadius, 
               const Vec2f* positions, std::vector<int>& outNeighbors) const;
    void query(const Vec2f& pos, float searchRadius, 
               const std::vector<Vec2f>& positions, std::vector<int>& outNeighbors) const;

    // 清空哈希表
    void clear();

    // 辅助函数：根据坐标获取网格坐标
    void getGridCoords(const Vec2f& pos, int& gx, int& gy) const;
    
    // 辅助函数：根据网格坐标获取哈希键值值
    int getHashKey(int gx, int gy) const;
    int getHashKeyForPosition(const Vec2f& pos) const;

    float getCellSize() const { return m_cellSize; }

private:
    float m_cellSize;                  // 网格单元大小
    int m_tableSize;                   // 哈希表大小 (质数)

    std::vector<Entry> m_entries;      // 存储所有排好序的 (key, index) 对，大小等于粒子总数
    std::vector<int> m_cellStart;      // 指向 m_entries 中该 Key 起始位置的索引，大小为 m_tableSize + 1
};
