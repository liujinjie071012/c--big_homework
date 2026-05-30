#pragma once

#include "../Common.hpp"
#include "../SpatialHash.hpp"
#include "../particle/Particle.hpp"
#include <vector>

class SPHSolver {
public:
    SPHSolver() = default;
    ~SPHSolver() = default;

    // SPH 流体力学主入口：计算所有流体粒子的密度、压力与相互作用力
    void updateSPH(std::vector<Particle>& particles, 
                   const SpatialHash& spatialHash, 
                   const PhysicsConfig& config,
                   const std::vector<Vec2f>& positions);

    // 兼容旧接口（内部构建 positions）
    void updateSPH(std::vector<Particle>& particles, 
                   const SpatialHash& spatialHash, 
                   const PhysicsConfig& config);

private:
    // SPH 核函数计算组件
    float poly6Kernel(float rSq, float h) const;
    Vec2f spikyKernelGradient(const Vec2f& rVec, float r, float h) const;
    float viscosityKernelLaplacian(float r, float h) const;

    std::vector<int> m_neighbors; // 复用的邻居索引缓冲区
};
