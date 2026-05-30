#pragma once

#include "../Math.hpp"

enum class EmitterType {
    NormalParticle,
    SPHFluid
};

class Emitter {
public:
    Vec2f position;           // 发射器坐标
    float angle;              // 发发射方向（弧度，0表示向右）
    float spread;             // 发射开角/扩散范围（弧度）
    float speed;              // 粒子初始喷出速度
    float rate;               // 每秒发射的粒子数量 (Rate)
    float particleRadius;     // 喷出粒子的半径
    EmitterType type;         // 粒子类型 (普通粒子/流体)
    bool active;              // 是否处于激活喷射状态

    float accumulator;        // 增量累加器，用于平滑根据时间差 dt 决定发射多少粒子

    Emitter(const Vec2f& pos, float angle = 0.0f, float spread = 0.2f, 
            float speed = 200.0f, float rate = 50.0f, float radius = 3.0f, 
            EmitterType type = EmitterType::NormalParticle)
        : position(pos), angle(angle), spread(spread), speed(speed), 
          rate(rate), particleRadius(radius), type(type), active(true), accumulator(0.0f) {}

    // 计算当前时间步 dt 下应该发射的粒子数量，返回需要发射的数目并在 accumulator 中扣除
    int update(float dt) {
        if (!active || rate <= 0.0f) return 0;
        
        accumulator += dt * rate;
        int count = static_cast<int>(accumulator);
        accumulator -= static_cast<float>(count);
        return count;
    }
};
