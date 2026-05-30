#pragma once

#include "Contact.hpp"
#include "../Common.hpp"

class RigidBodySolver {
public:
    RigidBodySolver() = default;
    ~RigidBodySolver() = default;

    // 碰撞冲量与摩擦力解算
    static void resolveCollision(ContactManifold& m, const PhysicsConfig& config);

    // 位置校正防止重叠穿透沉降
    static void positionCorrection(ContactManifold& m, const PhysicsConfig& config);
};
