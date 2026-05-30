#pragma once

#include "../Math.hpp"

enum class ParticleType {
    Normal,
    Fluid
};

// 基础粒子结构体 (整合 SPH 流体属性以实现极简高效的统一管理)
struct Particle {
    Vec2f position;         // 位置
    Vec2f velocity;         // 速度
    Vec2f force;            // 累积力
    float mass = 1.0f;      // 质量
    float radius = 3.0f;    // 半径
    bool isStatic = false;  // 是否静态

    // SPH 专属物理属性
    ParticleType type = ParticleType::Normal;
    float density = 0.0f;
    float pressure = 0.0f;

    Particle() = default;
    Particle(const Vec2f& pos, const Vec2f& vel = Vec2f(0.0f, 0.0f), float m = 1.0f, float r = 3.0f, bool isStatic = false, ParticleType type = ParticleType::Normal)
        : position(pos), velocity(vel), force(0.0f, 0.0f), mass(m), radius(r), isStatic(isStatic), type(type), density(0.0f), pressure(0.0f) {}
};
