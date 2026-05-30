#include "SPHSolver.hpp"
#include <cmath>

const float PI = 3.1415926535f;

float SPHSolver::poly6Kernel(float rSq, float h) const {
    float hSq = h * h;
    if (rSq >= hSq) return 0.0f;
    
    // 315 / (64 * PI * h^9) * (h^2 - r^2)^3
    float coeff = 315.0f / (64.0f * PI * std::pow(h, 9.0f));
    float diff = hSq - rSq;
    return coeff * diff * diff * diff;
}

Vec2f SPHSolver::spikyKernelGradient(const Vec2f& rVec, float r, float h) const {
    if (r >= h || r <= 0.0f) return Vec2f(0.0f, 0.0f);
    
    // -45 / (PI * h^6) * (h - r)^2 * (rVec / r)
    float coeff = -45.0f / (PI * std::pow(h, 6.0f));
    float diff = h - r;
    return coeff * diff * diff * (rVec / r);
}

float SPHSolver::viscosityKernelLaplacian(float r, float h) const {
    if (r >= h) return 0.0f;
    
    // 45 / (PI * h^6) * (h - r)
    float coeff = 45.0f / (PI * std::pow(h, 6.0f));
    return coeff * (h - r);
}

void SPHSolver::updateSPH(std::vector<Particle>& particles, 
                         const SpatialHash& spatialHash, 
                         const PhysicsConfig& config,
                         const std::vector<Vec2f>& positions) {
    int pCount = static_cast<int>(particles.size());
    if (pCount == 0) return;

    float h = config.fluidRadius;
    float hSq = h * h;
    float mass = config.fluidMass;
    float restDensity = config.fluidRestDensity;
    float stiffness = config.fluidStiffness;
    float viscosity = config.fluidViscosity;

    // 预计算核函数系数以提升循环内部性能
    float h3 = h * h * h;
    float h6 = h3 * h3;
    float h9 = h6 * h3;
    float poly6Coeff = 315.0f / (64.0f * PI * h9);
    float spikyCoeff = -45.0f / (PI * h6);
    float viscCoeff = 45.0f / (PI * h6);

    // ==========================================
    // 1. 密度与压力计算阶段
    // ==========================================
    for (int i = 0; i < pCount; ++i) {
        Particle& pA = particles[i];
        if (pA.type != ParticleType::Fluid) continue;

        pA.density = 0.0f;
        
        // 空间哈希网格邻域查询
        spatialHash.query(pA.position, h, positions, m_neighbors);

        for (int jIdx : m_neighbors) {
            const Particle& pB = particles[jIdx];
            if (pB.type != ParticleType::Fluid) continue;

            float distSq = pA.position.distanceSquared(pB.position);
            if (distSq < hSq) {
                // 累计密度项 (核函数叠加)
                float diff = hSq - distSq;
                pA.density += mass * (poly6Coeff * diff * diff * diff);
            }
        }

        // 避免分母为 0 引起除零崩溃
        if (pA.density < restDensity) {
            pA.density = restDensity;
        }

        // 使用 Tait 状态方程计算粒子处的状态压力
        pA.pressure = stiffness * (pA.density - restDensity);
    }

    // ==========================================
    // 2. 压力梯度力与粘性剪切力求解阶段
    // ==========================================
    for (int i = 0; i < pCount; ++i) {
        Particle& pA = particles[i];
        if (pA.type != ParticleType::Fluid) continue;

        Vec2f fPressure(0.0f, 0.0f);
        Vec2f fViscosity(0.0f, 0.0f);

        spatialHash.query(pA.position, h, positions, m_neighbors);

        for (int jIdx : m_neighbors) {
            if (jIdx == i) continue;

            const Particle& pB = particles[jIdx];
            if (pB.type != ParticleType::Fluid) continue;

            Vec2f rVec = pB.position - pA.position;
            float r = rVec.length();

            if (r < h && r > 0.0f) {
                // A. 压力梯度力项 (使用 Spiky 核函数防聚集)
                float diff = h - r;
                Vec2f gradW = (spikyCoeff * diff * diff) * (rVec / r);
                fPressure += -mass * ((pA.pressure + pB.pressure) / (2.0f * pB.density)) * gradW;

                // B. 粘性切应力项 (使用 Viscosity 核函数)
                float lapW = viscCoeff * (h - r);
                fViscosity += viscosity * mass * ((pB.velocity - pA.velocity) / pB.density) * lapW;
            }
        }

        // 累加流体力学合力到粒子
        pA.force += fPressure + fViscosity;
    }
}

void SPHSolver::updateSPH(std::vector<Particle>& particles, 
                         const SpatialHash& spatialHash, 
                         const PhysicsConfig& config) {
    // 兼容旧接口：内部构建 positions 向量
    int pCount = static_cast<int>(particles.size());
    std::vector<Vec2f> positions(pCount);
    for (int i = 0; i < pCount; ++i) {
        positions[i] = particles[i].position;
    }
    updateSPH(particles, spatialHash, config, positions);
}
