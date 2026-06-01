#include "SPHSolver.hpp"
#include "../World.hpp"
#include <immintrin.h>
#include <algorithm>
#include <cmath>

const float PI = 3.1415926535f;

float SPHSolver::poly6Kernel(float rSq, float h) const {
    float hSq = h * h;
    if (rSq >= hSq) return 0.0f;
    
    static float cached_h = -1.0f;
    static float cached_coeff = 0.0f;
    if (h != cached_h) {
        cached_h = h;
        cached_coeff = 4.0f / (PI * std::pow(h, 8.0f));
    }
    
    float diff = hSq - rSq;
    return cached_coeff * diff * diff * diff;
}

Vec2f SPHSolver::spikyKernelGradient(const Vec2f& rVec, float r, float h) const {
    if (r >= h || r <= 1e-5f) return Vec2f(0.0f, 0.0f);
    
    static float cached_h = -1.0f;
    static float cached_coeff = 0.0f;
    if (h != cached_h) {
        cached_h = h;
        cached_coeff = -30.0f / (PI * std::pow(h, 5.0f));
    }
    
    float diff = h - r;
    return cached_coeff * diff * diff * (rVec / r);
}

float SPHSolver::viscosityKernelLaplacian(float r, float h) const {
    if (r >= h) return 0.0f;
    
    static float cached_h = -1.0f;
    static float cached_coeff = 0.0f;
    if (h != cached_h) {
        cached_h = h;
        cached_coeff = 40.0f / (PI * std::pow(h, 5.0f));
    }
    
    return cached_coeff * (h - r);
}

void SPHSolver::updateSPH(ParticlesSoA& particles, 
                           const FlatNeighborList& neighborList, 
                           const PhysicsConfig& config) {
    int pCount = particles.count;
    if (pCount == 0) return;

    float h = config.fluidRadius;
    float hSq = h * h;
    float mass = config.fluidMass;
    float restDensity = config.fluidRestDensity;
    float stiffness = config.fluidStiffness;
    float viscosity = config.fluidViscosity;
    float maxPressure = stiffness * restDensity * 6.0f;
    float maxFluidForce = mass * std::max(4500.0f, std::abs(config.gravity.y) * 12.0f);

    // 预计算 2D 核函数系数以提升循环内部性能
    float h2 = h * h;
    float h4 = h2 * h2;
    float h5 = h4 * h;
    float h8 = h4 * h4;
    float poly6Coeff = 4.0f / (PI * h8);
    float spikyCoeff = -30.0f / (PI * h5);
    float viscCoeff = 40.0f / (PI * h5);

    // ==========================================
    // 1. 密度与压力计算阶段 (SSE2 向量化，每4邻居并行)
    // ==========================================
    #pragma omp parallel for schedule(static, 128)
    for (int i = 0; i < pCount; ++i) {
        if (particles.type[i] != ParticleType::Fluid) continue;

        int start = neighborList.begin[i];
        int end = neighborList.end[i];
        int n = end - start;

        const float* posX = particles.posX.data();
        const float* posY = particles.posY.data();
        const int* nbrs = neighborList.data.data() + start;

        // 【极致优化二】SSE2 向量化：4 邻居并行
        const __m128 px  = _mm_set1_ps(particles.posX[i]);
        const __m128 py  = _mm_set1_ps(particles.posY[i]);
        const __m128 H2  = _mm_set1_ps(hSq);
        const __m128 C   = _mm_set1_ps(poly6Coeff);
        __m128 acc       = _mm_setzero_ps();

        int k = 0;
        for (; k + 3 < n; k += 4) {
            int j0 = nbrs[k], j1 = nbrs[k+1], j2 = nbrs[k+2], j3 = nbrs[k+3];

            // Gather：从连续物理数组中打包邻居位置坐标
            __m128 jx = _mm_set_ps(posX[j3], posX[j2], posX[j1], posX[j0]);
            __m128 jy = _mm_set_ps(posY[j3], posY[j2], posY[j1], posY[j0]);

            __m128 dx = _mm_sub_ps(px, jx);
            __m128 dy = _mm_sub_ps(py, jy);
            __m128 r2 = _mm_add_ps(_mm_mul_ps(dx, dx), _mm_mul_ps(dy, dy));

            __m128 mask = _mm_cmplt_ps(r2, H2);         // r2 < h² 过滤掩码
            __m128 q    = _mm_sub_ps(H2, r2);           // h² - r²
            __m128 q3   = _mm_mul_ps(_mm_mul_ps(q, q), q);
            acc = _mm_add_ps(acc, _mm_and_ps(_mm_mul_ps(C, q3), mask));
        }

        // 水平累加 SIMD 四通道结果
        alignas(16) float temp[4];
        _mm_store_ps(temp, acc);
        float localDensity = temp[0] + temp[1] + temp[2] + temp[3];

        // 尾部标量处理 (不足4个的部分)
        for (; k < n; ++k) {
            int j = nbrs[k];
            float dx = particles.posX[i] - posX[j];
            float dy = particles.posY[i] - posY[j];
            float r2 = dx*dx + dy*dy;
            if (r2 < hSq) {
                float diff = hSq - r2;
                localDensity += poly6Coeff * diff * diff * diff;
            }
        }

        // 自身粒子质量及核函数贡献
        localDensity *= mass;
        localDensity += mass * poly6Coeff * (hSq * hSq * hSq);

        particles.density[i] = localDensity;
        float rhoForPressure = std::max(localDensity, restDensity);
        particles.pressure[i] = std::min(stiffness * (rhoForPressure - restDensity), maxPressure);
    }

    // ==========================================
    // 2. 压力梯度力与粘性剪切力求解阶段 (SoA 并行求解)
    // ==========================================
    #pragma omp parallel for schedule(static, 128)
    for (int i = 0; i < pCount; ++i) {
        if (particles.type[i] != ParticleType::Fluid) continue;

        int start = neighborList.begin[i];
        int end = neighborList.end[i];

        float rhoA = std::max(particles.density[i], 0.001f);
        float pressA = particles.pressure[i];

        Vec2f fPressure(0.0f, 0.0f);
        Vec2f fViscosity(0.0f, 0.0f);

        for (int k = start; k < end; ++k) {
            int j = neighborList.data[k];
            if (j == i) continue;

            Vec2f rVec(particles.posX[j] - particles.posX[i], particles.posY[j] - particles.posY[i]);
            float rSq = rVec.lengthSquared();

            if (rSq < hSq) {
                float r = std::sqrt(rSq);
                if (r <= 1e-5f) {
                    float angle = (i + j) * 0.1f;
                    rVec = Vec2f(std::cos(angle), std::sin(angle)) * 0.01f;
                    r = 0.01f;
                }

                // A. 压力梯度力项 (使用 Spiky 核函数的梯度)
                float diff = h - r;
                Vec2f gradW = (spikyCoeff * diff * diff) * (rVec / r);
                
                float rhoB = std::max(particles.density[j], 0.001f);
                float pressureTerm = (pressA / (rhoA * rhoA)) + (particles.pressure[j] / (rhoB * rhoB));
                fPressure += -mass * mass * pressureTerm * gradW;

                // B. 粘性切应力项 (使用 Viscosity 核函数)
                float lapW = viscCoeff * diff;
                Vec2f vDiff(particles.velX[j] - particles.velX[i], particles.velY[j] - particles.velY[i]);
                fViscosity += viscosity * mass * (vDiff / rhoB) * lapW;
            }
        }

        Vec2f fluidForce = fPressure + fViscosity;
        float forceSq = fluidForce.lengthSquared();
        if (forceSq > maxFluidForce * maxFluidForce) {
            fluidForce = fluidForce.normalized() * maxFluidForce;
        }

        // 累加流体力学合力到粒子合力缓存中 (SoA 写入)
        particles.forceX[i] += fluidForce.x;
        particles.forceY[i] += fluidForce.y;
    }
}
