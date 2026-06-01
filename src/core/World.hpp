#pragma once

#include "Common.hpp"
#include "SpatialHash.hpp"
#include "particle/Particle.hpp"
#include "particle/Emitter.hpp"
#include "fluid/SPHSolver.hpp"
#include "rigidbody/RigidBody.hpp"
#include <vector>

struct Wall {
    Vec2f start;
    Vec2f end;
    Vec2f minBound;
    Vec2f maxBound;
    Wall(const Vec2f& s, const Vec2f& e) : start(s), end(e) {
        minBound.x = std::min(s.x, e.x);
        minBound.y = std::min(s.y, e.y);
        maxBound.x = std::max(s.x, e.x);
        maxBound.y = std::max(s.y, e.y);
    }
};

// 【极致优化三】平铺的一维 CSR 邻居列表
struct FlatNeighborList {
    std::vector<int> data;   // 所有粒子的邻居索引连续平铺存储
    std::vector<int> begin;  // begin[i]：粒子 i 的邻居在 data 中的起始索引
    std::vector<int> end;    // end[i]：粒子 i 的邻居在 data 中的结束索引
    
    void resize(int pCount) {
        begin.assign(pCount, 0);
        end.assign(pCount, 0);
        data.clear();
    }
};

// 【极致优化三】跨子步邻居缓存复用器
struct NeighborCache {
    FlatNeighborList list;
    int staleCounter = 0;
    static constexpr int REBUILD_INTERVAL = 2; // 每 2 子步重建一次邻居表

    std::vector<float> prevPosX; // 重建时的粒子位置，用于最大位移校对
    std::vector<float> prevPosY;

    bool needsRebuild() const { return staleCounter >= REBUILD_INTERVAL; }
    void markRebuilt() { staleCounter = 0; }
    void age() { staleCounter++; }
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
    const ParticlesSoA& getParticles() const { return m_particles; }
    ParticlesSoA& getParticles() { return m_particles; }
    
    const std::vector<Emitter>& getEmitters() const { return m_emitters; }
    std::vector<Emitter>& getEmitters() { return m_emitters; }
    
    const std::vector<Wall>& getWalls() const { return m_walls; }
    
    const std::vector<RigidBody>& getRigidBodies() const { return m_rigidBodies; }
    std::vector<RigidBody>& getRigidBodies() { return m_rigidBodies; }

    const SpatialHash& getSpatialHash() const { return m_spatialHash; }
    const FlatNeighborList& getNeighborList() const { return m_neighborCache.list; }

private:
    ParticlesSoA m_particles; // 【极致优化一】SoA 内存布局替换 AoS 列表
    std::vector<Emitter> m_emitters;
    std::vector<Wall> m_walls;
    std::vector<RigidBody> m_rigidBodies;

    SpatialHash m_spatialHash;
    SPHSolver m_sphSolver;
    NeighborCache m_neighborCache; // 【极致优化三】邻域跨子步缓存复用

    // 子步积分与碰撞解算
    void substep(float sdt);
    void applyForces(float sdt);
    void emitParticles(float sdt);
    void buildNeighborLists();     // 构建 CSR 邻居列表的一维平铺高效表示
    void resolveCollisions();
    void resolveBoundaryCollisions(int i);
    void resolveWallCollisions(int i);
    void resolveRigidBoundaryCollisions(RigidBody& b);
    void resolveRigidWallCollisions(RigidBody& b);
    void resolveParticleRigidCollisions();
    void integrateStep1(float sdt);
    void integrateStep2(float sdt);
};
