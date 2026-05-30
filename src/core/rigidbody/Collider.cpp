#include "Collider.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

bool Collider::detectCollision(ContactManifold& m, RigidBody* a, RigidBody* b) {
    m.bodyA = a;
    m.bodyB = b;
    m.contacts.clear();

    if (a->shape == ShapeType::Circle && b->shape == ShapeType::Circle) {
        return collideCircleCircle(m, a, b);
    }
    else if (a->shape == ShapeType::Polygon && b->shape == ShapeType::Polygon) {
        return collidePolygonPolygon(m, a, b);
    }
    else if (a->shape == ShapeType::Circle && b->shape == ShapeType::Polygon) {
        return collideCirclePolygon(m, a, b);
    }
    else if (a->shape == ShapeType::Polygon && b->shape == ShapeType::Circle) {
        // 多边形与圆形碰撞，调换顺序求解并反转法线
        bool collided = collideCirclePolygon(m, b, a);
        if (collided) {
            m.normal = -m.normal;
            m.bodyA = a;
            m.bodyB = b;
        }
        return collided;
    }
    return false;
}

bool Collider::collideCircleCircle(ContactManifold& m, RigidBody* a, RigidBody* b) {
    Vec2f toB = b->position - a->position;
    float distSq = toB.lengthSquared();
    float radiusSum = a->radius + b->radius;

    if (distSq >= radiusSum * radiusSum) {
        return false;
    }

    float dist = std::sqrt(distSq);
    if (dist == 0.0f) {
        m.penetration = a->radius;
        m.normal = Vec2f(0.0f, -1.0f);
        m.contacts.push_back(a->position);
    } else {
        m.penetration = radiusSum - dist;
        m.normal = toB / dist; // 指向 B 的单位法线
        m.contacts.push_back(a->position + m.normal * a->radius);
    }
    return true;
}

void Collider::projectPolygon(const RigidBody& poly, const Vec2f& axis, float& minProj, float& maxProj) {
    if (poly.globalVertices.empty()) return;
    
    minProj = poly.globalVertices[0].dot(axis);
    maxProj = minProj;
    
    int count = static_cast<int>(poly.globalVertices.size());
    for (int i = 1; i < count; ++i) {
        float proj = poly.globalVertices[i].dot(axis);
        if (proj < minProj) minProj = proj;
        if (proj > maxProj) maxProj = proj;
    }
}

bool Collider::isPointInPolygon(const Vec2f& p, const RigidBody& poly) {
    if (poly.shape != ShapeType::Polygon) return false;
    
    int count = static_cast<int>(poly.globalVertices.size());
    for (int i = 0; i < count; ++i) {
        Vec2f v1 = poly.globalVertices[i];
        Vec2f v2 = poly.globalVertices[(i + 1) % count];
        Vec2f edge = v2 - v1;
        Vec2f toPoint = p - v1;
        
        // 2D 叉乘：若小于 0 则点在边右侧，表明点在凸多边形外部 (假定逆时针定点序)
        if (edge.cross(toPoint) < -0.01f) {
            return false;
        }
    }
    return true;
}

bool Collider::collidePolygonPolygon(ContactManifold& m, RigidBody* a, RigidBody* b) {
    a->updateGlobalVertices();
    b->updateGlobalVertices();

    float minOverlap = std::numeric_limits<float>::max();
    Vec2f bestAxis;

    // 1. 遍历多边形 A 的所有面法线作为投影轴
    int countA = static_cast<int>(a->globalVertices.size());
    for (int i = 0; i < countA; ++i) {
        Vec2f axis = a->globalNormals[i];
        
        float minA, maxA, minB, maxB;
        projectPolygon(*a, axis, minA, maxA);
        projectPolygon(*b, axis, minB, maxB);

        float overlap = std::min(maxA, maxB) - std::max(minA, minB);
        if (overlap <= 0.0f) {
            return false; // 找到分离轴，未发生碰撞
        }

        if (overlap < minOverlap) {
            minOverlap = overlap;
            bestAxis = axis;
        }
    }

    // 2. 遍历多边形 B 的所有面法线作为投影轴
    int countB = static_cast<int>(b->globalVertices.size());
    for (int i = 0; i < countB; ++i) {
        Vec2f axis = b->globalNormals[i];
        
        float minA, maxA, minB, maxB;
        projectPolygon(*a, axis, minA, maxA);
        projectPolygon(*b, axis, minB, maxB);

        float overlap = std::min(maxA, maxB) - std::max(minA, minB);
        if (overlap <= 0.0f) {
            return false; // 找到分离轴，未发生碰撞
        }

        if (overlap < minOverlap) {
            minOverlap = overlap;
            bestAxis = axis;
        }
    }

    // 保证碰撞法线方向始终是由 A 指向 B
    Vec2f toB = b->position - a->position;
    if (toB.dot(bestAxis) < 0.0f) {
        bestAxis = -bestAxis;
    }

    m.normal = bestAxis;
    m.penetration = minOverlap;

    // 3. 接触点检测 (通过顶点包含法检测接触点，在 2D 中极其鲁棒且易于编写)
    for (const auto& v : a->globalVertices) {
        if (isPointInPolygon(v, *b)) {
            m.contacts.push_back(v);
        }
    }
    for (const auto& v : b->globalVertices) {
        if (isPointInPolygon(v, *a)) {
            m.contacts.push_back(v);
        }
    }

    // 防御性保护：若两个顶点平行刚好压在边缘，顶点包含检测可能失效，取重叠区中心点作为接触点
    if (m.contacts.empty()) {
        Vec2f centroid = (a->position + b->position) * 0.5f;
        m.contacts.push_back(centroid);
    }

    return true;
}

bool Collider::collideCirclePolygon(ContactManifold& m, RigidBody* a, RigidBody* b) {
    b->updateGlobalVertices();

    float minOverlap = std::numeric_limits<float>::max();
    Vec2f bestAxis;

    // 1. 遍历多边形 B 的所有边法线投影轴
    int countB = static_cast<int>(b->globalVertices.size());
    for (int i = 0; i < countB; ++i) {
        Vec2f axis = b->globalNormals[i];

        float minB, maxB;
        projectPolygon(*b, axis, minB, maxB);

        // 圆投影在其中心投影左右半径范围内
        float centerProj = a->position.dot(axis);
        float minA = centerProj - a->radius;
        float maxA = centerProj + a->radius;

        float overlap = std::min(maxA, maxB) - std::max(minA, minB);
        if (overlap <= 0.0f) return false;

        if (overlap < minOverlap) {
            minOverlap = overlap;
            bestAxis = axis;
        }
    }

    // 2. 检测从多边形最近顶点指向圆心的特殊投影轴
    float minDistance = std::numeric_limits<float>::max();
    Vec2f closestVertex;
    for (const auto& v : b->globalVertices) {
        float dist = a->position.distanceSquared(v);
        if (dist < minDistance) {
            minDistance = dist;
            closestVertex = v;
        }
    }

    Vec2f axis = a->position - closestVertex;
    if (axis.lengthSquared() > 0.0001f) {
        axis.normalize();

        float minB, maxB;
        projectPolygon(*b, axis, minB, maxB);

        float centerProj = a->position.dot(axis);
        float minA = centerProj - a->radius;
        float maxA = centerProj + a->radius;

        float overlap = std::min(maxA, maxB) - std::max(minA, minB);
        if (overlap <= 0.0f) return false;

        if (overlap < minOverlap) {
            minOverlap = overlap;
            bestAxis = axis;
        }
    }

    // 确保法线方向是由 A (Circle) 指向 B (Polygon)
    Vec2f toB = b->position - a->position;
    if (toB.dot(bestAxis) < 0.0f) {
        bestAxis = -bestAxis;
    }

    m.normal = bestAxis;
    m.penetration = minOverlap;
    
    // 接触点为圆心沿碰撞法线排布的交点
    m.contacts.push_back(a->position + bestAxis * (a->radius - minOverlap * 0.5f));

    return true;
}
