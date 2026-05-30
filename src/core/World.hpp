#pragma once

#include "Common.hpp"
#include "SpatialHash.hpp"
#include "particle/Particle.hpp"
#include "particle/Emitter.hpp"
#include "fluid/SPHSolver.hpp"
#include "rigidbody/RigidBody.hpp"
#include <vector>
#include <memory>

struct Wall {
    Vec2f start;
    Vec2f end;
    Wall(const Vec2f& s, const Vec2f& e) : start(s), end(e) {}
};

class PhysicsWorld {
public:
    PhysicsConfig config;

    PhysicsWorld();
    ~PhysicsWorld() = default;

    // 实体管理
    void addParticle(const Particle& p);
    void addEmitter(const Emitter& e);
    void addWall(const Vec2f& start, const Vec2f& end);
    void addRigidBody(const RigidBody& b);

    // 核心仿真流程
    void update(float dt);
    void step(); // 执行单步调试
    void reset();
    void clear();

    // 数据获取器
    const std::vector<Particle>& getParticles() const { return m_particles; }
    std::vector<Particle>& getParticles() { return m_particles; }
    
    const std::vector<Emitter>& getEmitters() const { return m_emitters; }
    std::vector<Emitter>& getEmitters() { return m_emitters; }
    
    const std::vector<Wall>& getWalls() const { return m_walls; }
    
    const std::vector<RigidBody>& getRigidBodies() const { return m_rigidBodies; }
    std::vector<RigidBody>& getRigidBodies() { return m_rigidBodies; }

    const SpatialHash& getSpatialHash() const { return m_spatialHash; }

private:
    std::vector<Particle> m_particles;
    std::vector<Emitter> m_emitters;
    std::vector<Wall> m_walls;
    std::vector<RigidBody> m_rigidBodies;

    SpatialHash m_spatialHash;
    SPHSolver m_sphSolver;
    std::vector<Vec2f> m_positionsCache; // 缓存粒子位置，避免每帧重复堆分配

    // 子步积分与碰撞解算
    void substep(float sdt);
    void applyForces(float sdt);
    void emitParticles(float sdt);
    void resolveCollisions();
    void resolveBoundaryCollisions(Particle& p);
    void resolveWallCollisions(Particle& p);
    void resolveRigidBoundaryCollisions(RigidBody& b);
    void resolveRigidWallCollisions(RigidBody& b);
    void resolveParticleRigidCollisions();
    void integrate(float sdt);
};
