#pragma once

#include "Math.hpp"

// 物理世界全局配置结构体
struct PhysicsConfig {
    Vec2f gravity = Vec2f(0.0f, 900.0f);     // 重力加速度 (像素/秒^2)
    float timeStep = 0.016f;                  // 物理时间步长
    int substeps = 5;                         // 物理子步数（5步可在维持完美稳定性的前提下获得巨大帧率提升）

    // 屏幕/世界边界 (像素)
    float worldWidth = 800.0f;
    float worldHeight = 600.0f;
    float boundaryDamping = 0.5f;             // 边界碰撞弹性损失

    // 粒子系统配置
    float particleRadius = 3.0f;              // 基础粒子半径
    float particleElasticity = 0.25f;         // 粒子自碰撞弹性
    float particleFriction = 0.1f;            // 粒子摩擦力

    // SPH 流体仿真配置
    float fluidRadius = 12.0f;                // 流体粒子影响半径 (h)
    float fluidRestDensity = 0.032f;          // 静止参考密度 (rho_0) (对应 h=12, m=1.0, 间距 6.0 像素的 2D 紧密排布理论密度)
    float fluidStiffness = 650.0f;            // Tait 状态方程刚度系数 (k) (调低刚度，让流体更柔软，使重力表现更自然)
    float fluidViscosity = 1.2f;              // 粘性系数 (mu) (提高耗能，抑制堆积时的飞溅)
    float fluidMass = 1.0f;                   // 单个流体粒子的质量 (m)

    // 刚体仿真配置
    float rigidMinMass = 0.0001f;             // 刚体最小质量
    float rigidDefaultDensity = 1.0f;        // 默认刚体密度
    float rigidRestitution = 0.5f;            // 刚体碰撞弹性
    float rigidFriction = 0.3f;               // 刚体摩擦力
    float positionCorrectionBeta = 0.2f;      // 碰撞渗透修正率 (0.2~0.8)
    float positionCorrectionSlop = 0.01f;     // 允许穿透阈值
};

// 刚体类型枚举
enum class BodyType {
    Static,   // 静态（质量无限大，不可移动）
    Dynamic,  // 动态（受重力及碰撞冲量影响移动）
    Kinematic // 运动学（不受力，但有速度，可通过代码直接控制移动）
};

// 刚体形状分类
enum class ShapeType {
    Circle,
    Polygon
};
