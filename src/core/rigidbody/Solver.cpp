#include "Solver.hpp"
#include <cmath>
#include <algorithm>

void RigidBodySolver::resolveCollision(ContactManifold& m, const PhysicsConfig& config) {
    RigidBody* a = m.bodyA;
    RigidBody* b = m.bodyB;

    float invMassSum = a->invMass + b->invMass;
    if (invMassSum == 0.0f) return; // 两个都是静态刚体，直接跳过不作处理

    int contactsCount = static_cast<int>(m.contacts.size());
    if (contactsCount == 0) return;

    // 联合计算恢复系数
    float e = std::min(a->restitution, b->restitution);
    // 联合计算摩擦力系数
    float sf = std::sqrt(a->friction * b->friction);

    for (int i = 0; i < contactsCount; ++i) {
        Vec2f pc = m.contacts[i];
        
        // 质心到接触点的向量 rVec
        Vec2f rA = pc - a->position;
        Vec2f rB = pc - b->position;

        // 计算接触点处的相对速度 (包含角速度贡献 v = v_linear + omega x r)
        // 2D 叉乘: w x r = Vec2f(-w * r.y, w * r.x)
        Vec2f vRelA = a->velocity + Vec2f(-a->angularVelocity * rA.y, a->angularVelocity * rA.x);
        Vec2f vRelB = b->velocity + Vec2f(-b->angularVelocity * rB.y, b->angularVelocity * rB.x);
        Vec2f rv = vRelB - vRelA;

        // 相对速度在法线方向上的投影
        float vNormal = rv.dot(m.normal);

        // 如果它们正在远离，则不需要做任何冲量反弹
        if (vNormal > 0.0f) continue;

        // 计算法向冲量 scalar j
        float rAcrossN = rA.cross(m.normal);
        float rBcrossN = rB.cross(m.normal);
        float rotMass = rAcrossN * rAcrossN * a->invInertia + 
                        rBcrossN * rBcrossN * b->invInertia;

        float j = -(1.0f + e) * vNormal;
        j /= (invMassSum + rotMass);
        j /= static_cast<float>(contactsCount); // 均摊到所有接触点上

        // 施加法向反弹冲量
        Vec2f impulse = m.normal * j;
        a->applyImpulse(-impulse, rA);
        b->applyImpulse(impulse, rB);

        // ==========================================
        // 摩擦冲量计算
        // ==========================================
        // 重新计算相对速度以体现法向冲量带来的影响
        vRelA = a->velocity + Vec2f(-a->angularVelocity * rA.y, a->angularVelocity * rA.x);
        vRelB = b->velocity + Vec2f(-b->angularVelocity * rB.y, b->angularVelocity * rB.x);
        rv = vRelB - vRelA;

        // 沿法线切向的方向向量 t
        Vec2f tangent(-m.normal.y, m.normal.x);
        float vTangent = rv.dot(tangent);

        // 计算切向摩擦冲量 scalar jt
        float rAcrossT = rA.cross(tangent);
        float rBcrossT = rB.cross(tangent);
        float rotMassT = rAcrossT * rAcrossT * a->invInertia + 
                         rBcrossT * rBcrossT * b->invInertia;

        float jt = -vTangent;
        jt /= (invMassSum + rotMassT);
        jt /= static_cast<float>(contactsCount);

        // 库伦滑动/静摩擦力边界限制 (|jt| <= j * friction)
        float maxJt = j * sf;
        jt = std::max(-maxJt, std::min(maxJt, jt));

        // 施加摩擦冲量
        Vec2f frictionImpulse = tangent * jt;
        a->applyImpulse(-frictionImpulse, rA);
        b->applyImpulse(frictionImpulse, rB);
    }
}

void RigidBodySolver::positionCorrection(ContactManifold& m, const PhysicsConfig& config) {
    RigidBody* a = m.bodyA;
    RigidBody* b = m.bodyB;

    float invMassSum = a->invMass + b->invMass;
    if (invMassSum == 0.0f) return;

    // 线投影穿透修正法 (Linear Projection)
    float correctionAmount = std::max(m.penetration - config.positionCorrectionSlop, 0.0f) / invMassSum;
    correctionAmount *= config.positionCorrectionBeta; // 阻尼校正率

    Vec2f correction = m.normal * correctionAmount;
    if (a->type != BodyType::Static) {
        a->position -= correction * a->invMass;
    }
    if (b->type != BodyType::Static) {
        b->position += correction * b->invMass;
    }
}
