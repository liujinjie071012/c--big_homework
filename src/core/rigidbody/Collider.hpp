#pragma once

#include "Contact.hpp"

class Collider {
public:
    Collider() = default;
    ~Collider() = default;

    // 统一检测入口：根据形状分发到对应具体求解器
    static bool detectCollision(ContactManifold& m, RigidBody* a, RigidBody* b);

    // 1. 圆形-圆形碰撞检测
    static bool collideCircleCircle(ContactManifold& m, RigidBody* a, RigidBody* b);

    // 2. 圆形-多边形碰撞检测
    static bool collideCirclePolygon(ContactManifold& m, RigidBody* a, RigidBody* b);

    // 3. 多边形-多边形碰撞检测 (SAT 分离轴定理)
    static bool collidePolygonPolygon(ContactManifold& m, RigidBody* a, RigidBody* b);

    // 辅助函数：点是否在凸多边形内部
    static bool isPointInPolygon(const Vec2f& p, const RigidBody& poly);

private:
    // 辅助函数：求多边形在指定轴上的投影区间
    static void projectPolygon(const RigidBody& poly, const Vec2f& axis, float& minProj, float& maxProj);
};
