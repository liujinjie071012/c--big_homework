#pragma once

#include "../Math.hpp"
#include <vector>

enum class ParticleType {
    Normal,
    Fluid
};

// 基础粒子结构体 (用作轻量数据传输与兼容接口)
struct Particle {
    Vec2f position;         // 位置
    Vec2f oldPosition;      // 上一帧位置 (用于 CCD 连续碰撞检测)
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
        : position(pos), oldPosition(pos), velocity(vel), force(0.0f, 0.0f), mass(m), radius(r), isStatic(isStatic), type(type), density(0.0f), pressure(0.0f) {}
};

// 【极致优化一】Structure of Arrays (SoA) 高效内存布局，消灭 Cache Misses
struct ParticlesSoA {
    std::vector<float> posX;
    std::vector<float> posY;
    std::vector<float> oldPosX;
    std::vector<float> oldPosY;
    std::vector<float> velX;
    std::vector<float> velY;
    std::vector<float> forceX;
    std::vector<float> forceY;
    std::vector<float> mass;
    std::vector<float> radius;
    std::vector<bool> isStatic;
    std::vector<ParticleType> type;
    std::vector<float> density;
    std::vector<float> pressure;
    int count = 0;

    void clear() {
        posX.clear(); posY.clear();
        oldPosX.clear(); oldPosY.clear();
        velX.clear(); velY.clear();
        forceX.clear(); forceY.clear();
        mass.clear(); radius.clear();
        isStatic.clear(); type.clear();
        density.clear(); pressure.clear();
        count = 0;
    }

    void addParticle(const Vec2f& pos, const Vec2f& vel, float m, float r, bool isStat, ParticleType t) {
        posX.push_back(pos.x); posY.push_back(pos.y);
        oldPosX.push_back(pos.x); oldPosY.push_back(pos.y);
        velX.push_back(vel.x); velY.push_back(vel.y);
        forceX.push_back(0.0f); forceY.push_back(0.0f);
        mass.push_back(m); radius.push_back(r);
        isStatic.push_back(isStat); type.push_back(t);
        density.push_back(0.0f); pressure.push_back(0.0f);
        count++;
    }

    void addParticle(const Particle& p) {
        addParticle(p.position, p.velocity, p.mass, p.radius, p.isStatic, p.type);
    }

    void erase(int index) {
        if (index < 0 || index >= count) return;
        posX.erase(posX.begin() + index);
        posY.erase(posY.begin() + index);
        oldPosX.erase(oldPosX.begin() + index);
        oldPosY.erase(oldPosY.begin() + index);
        velX.erase(velX.begin() + index);
        velY.erase(velY.begin() + index);
        forceX.erase(forceX.begin() + index);
        forceY.erase(forceY.begin() + index);
        mass.erase(mass.begin() + index);
        radius.erase(radius.begin() + index);
        isStatic.erase(isStatic.begin() + index);
        type.erase(type.begin() + index);
        density.erase(density.begin() + index);
        pressure.erase(pressure.begin() + index);
        count--;
    }
};
