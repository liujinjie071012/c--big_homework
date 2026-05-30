#pragma once

#include "RigidBody.hpp"
#include <vector>

// 碰撞解算接触面信息流形
struct ContactManifold {
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;

    Vec2f normal;                   // 碰撞法向 (bodyA 指向 bodyB)
    float penetration = 0.0f;       // 穿透深度 (Penetration)
    std::vector<Vec2f> contacts;    // 接触点列表 (Contact Points, 2D 中通常最多 2 个)

    ContactManifold() = default;
    ContactManifold(RigidBody* a, RigidBody* b) 
        : bodyA(a), bodyB(b), normal(0.0f, -1.0f), penetration(0.0f) {}
};
