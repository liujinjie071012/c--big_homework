#pragma once

#include "../Math.hpp"
#include "../Common.hpp"
#include <vector>

const float MATH_PI = 3.1415926535f;

class RigidBody {
public:
    BodyType type = BodyType::Dynamic;
    ShapeType shape = ShapeType::Circle;

    // 线运动学状态
    Vec2f position;
    Vec2f velocity;
    Vec2f force;

    // 角运动学状态
    float angle = 0.0f;          // 旋转弧度
    float angularVelocity = 0.0f;
    float torque = 0.0f;

    // 物理力学质量参数
    float mass = 1.0f;
    float invMass = 1.0f;
    float inertia = 1.0f;        // 转动惯量
    float invInertia = 1.0f;

    // 材质系数
    float restitution = 0.5f;     // 弹性系数
    float friction = 0.3f;        // 摩擦系数

    // 圆形属性
    float radius = 10.0f;

    // 多边形专用顶点（逆时针排列，局部坐标系）
    std::vector<Vec2f> vertices;
    std::vector<Vec2f> normals;

    // 缓存的全局状态顶点（避免每一帧多次重复计算旋转和位移）
    std::vector<Vec2f> globalVertices;
    std::vector<Vec2f> globalNormals;

    RigidBody() = default;
    ~RigidBody() = default;

    // 更新多边形的世界坐标系顶点和法线
    void updateGlobalVertices() {
        if (shape == ShapeType::Polygon) {
            int count = static_cast<int>(vertices.size());
            globalVertices.resize(count);
            globalNormals.resize(count);
            
            Mat2f R(angle);
            for (int i = 0; i < count; ++i) {
                globalVertices[i] = position + R * vertices[i];
                globalNormals[i] = R * normals[i];
            }
        }
    }

    // 重置外力与扭矩
    void clearForces() {
        force = Vec2f(0.0f, 0.0f);
        torque = 0.0f;
    }

    // 施加冲量到特定作用点 (点相对质心的向量 rVec)
    void applyImpulse(const Vec2f& impulse, const Vec2f& rVec) {
        if (type == BodyType::Static) return;
        velocity += impulse * invMass;
        angularVelocity += rVec.cross(impulse) * invInertia; // 2D 叉乘产生旋转阻尼力矩
    }

    // 静态工厂函数：快速构造矩形多边形刚体
    static RigidBody createBox(const Vec2f& pos, float width, float height, BodyType type = BodyType::Dynamic) {
        RigidBody b;
        b.position = pos;
        b.type = type;
        b.shape = ShapeType::Polygon;
        
        float hx = width * 0.5f;
        float hy = height * 0.5f;
        
        // 逆时针定点
        b.vertices = {
            Vec2f(-hx, -hy), // 左上
            Vec2f(-hx, hy),  // 左下
            Vec2f(hx, hy),   // 右下
            Vec2f(hx, -hy)   // 右上
        };
        
        // 四条边的外法线
        b.normals = {
            Vec2f(-1.0f, 0.0f),
            Vec2f(0.0f, 1.0f),
            Vec2f(1.0f, 0.0f),
            Vec2f(0.0f, -1.0f)
        };

        if (type == BodyType::Static) {
            b.mass = 0.0f;
            b.invMass = 0.0f;
            b.inertia = 0.0f;
            b.invInertia = 0.0f;
        } else {
            b.mass = width * height * 0.01f; // 缩放质量密度
            b.invMass = 1.0f / b.mass;
            b.inertia = b.mass * (width * width + height * height) / 12.0f;
            b.invInertia = 1.0f / b.inertia;
        }
        
        b.updateGlobalVertices();
        return b;
    }

    // 静态工厂函数：快速构造圆形刚体
    static RigidBody createCircle(const Vec2f& pos, float radius, BodyType type = BodyType::Dynamic) {
        RigidBody b;
        b.position = pos;
        b.type = type;
        b.shape = ShapeType::Circle;
        b.radius = radius;

        if (type == BodyType::Static) {
            b.mass = 0.0f;
            b.invMass = 0.0f;
            b.inertia = 0.0f;
            b.invInertia = 0.0f;
        } else {
            b.mass = MATH_PI * radius * radius * 0.01f;
            b.invMass = 1.0f / b.mass;
            b.inertia = 0.5f * b.mass * radius * radius;
            b.invInertia = 1.0f / b.inertia;
        }
        
        return b;
    }
};
