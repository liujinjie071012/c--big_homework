#pragma once

#include "../Common.hpp"
#include "../SpatialHash.hpp"
#include "../particle/Particle.hpp"
#include <vector>

// 前置声明
struct FlatNeighborList;

class SPHSolver {
public:
    SPHSolver() = default;
    ~SPHSolver() = default;

    // SPH 流体力学主入口：计算所有流体粒子的密度、压力与相互作用力，支持 SoA 与 CSR 邻域缓存
    void updateSPH(ParticlesSoA& particles, 
                   const FlatNeighborList& neighborList, 
                   const PhysicsConfig& config);

private:
    // SPH 核函数计算组件
    float poly6Kernel(float rSq, float h) const;
    Vec2f spikyKernelGradient(const Vec2f& rVec, float r, float h) const;
    float viscosityKernelLaplacian(float r, float h) const;
};
