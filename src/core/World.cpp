#include "World.hpp"
#include "rigidbody/Collider.hpp"
#include "rigidbody/Solver.hpp"
#include <random>
#include <cmath>
#include <algorithm>

// 辅助随机数生成器
static float randomFloat(float min, float max) {
    static std::mt19937 gen(42); // 固定种子保证物理复现性
    std::uniform_real_distribution<float> dis(min, max);
    return dis(gen);
}

PhysicsWorld::PhysicsWorld()
    : m_spatialHash(config.fluidRadius, 100003) {
}

void PhysicsWorld::addParticle(const Particle& p) {
    m_particles.addParticle(p);
}

void PhysicsWorld::addEmitter(const Emitter& e) {
    m_emitters.push_back(e);
}

void PhysicsWorld::addWall(const Vec2f& start, const Vec2f& end) {
    m_walls.emplace_back(start, end);
}

void PhysicsWorld::addRigidBody(const RigidBody& b) {
    m_rigidBodies.push_back(b);
}

void PhysicsWorld::clear() {
    m_particles.clear();
    m_emitters.clear();
    m_walls.clear();
    m_rigidBodies.clear();
    m_spatialHash.clear();
    m_neighborCache.list.resize(0);
    m_neighborCache.prevPosX.clear();
    m_neighborCache.prevPosY.clear();
}

void PhysicsWorld::reset() {
    m_particles.clear();
    m_spatialHash.clear();
    for (auto& e : m_emitters) {
        e.accumulator = 0.0f;
    }
    m_neighborCache.list.resize(0);
    m_neighborCache.prevPosX.clear();
    m_neighborCache.prevPosY.clear();
}

void PhysicsWorld::step() {
    update(config.timeStep);
}

void PhysicsWorld::update(float dt) {
    // 【极致优化四】CFL 条件自适应子步步长调节 (从 5 ➔ 2~5 自动变档)
    int pCount = m_particles.count;
    float maxSpeedSq = 0.0f;
    for (int i = 0; i < pCount; ++i) {
        float vSq = m_particles.velX[i] * m_particles.velX[i] + m_particles.velY[i] * m_particles.velY[i];
        if (vSq > maxSpeedSq) {
            maxSpeedSq = vSq;
        }
    }
    float soundSpeed = 300.0f; // 声速项
    float vmax = std::sqrt(maxSpeedSq) + soundSpeed;

    const float CFL_COEFF = 0.4f;
    float h = config.fluidRadius;
    float dtMax = CFL_COEFF * h / (vmax + 1e-6f);
    
    int substeps = static_cast<int>(std::ceil(dt / dtMax));
    substeps = std::clamp(substeps, 2, 5); // 限制在 2 步到 5 步之间以保底稳定度
    
    float sdt = dt / static_cast<float>(substeps);
    for (int step = 0; step < substeps; ++step) {
        substep(sdt);
    }
}

void PhysicsWorld::substep(float sdt) {
    // 1. 发射器喷出新粒子
    emitParticles(sdt);

    // 2. Velocity Verlet 阶段1
    integrateStep1(sdt);

    // 3. 累加外力（如重力、阻力）
    applyForces(sdt);

    // 【极致优化三】邻域跨子步缓存复用校验 (判断位移量是否超出 0.3h 重建阈值)
    int pCount = m_particles.count;
    bool forceRebuild = false;
    if (m_neighborCache.prevPosX.size() != static_cast<size_t>(pCount)) {
        forceRebuild = true;
    } else {
        float maxDispSq = 0.0f;
        float h = config.fluidRadius;
        float threshold = 0.09f * h * h; // 位移平方 > 0.09h² (即位移 > 0.3h)
        
        #pragma omp parallel for reduction(max:maxDispSq) schedule(static, 128)
        for (int i = 0; i < pCount; ++i) {
            float dx = m_particles.posX[i] - m_neighborCache.prevPosX[i];
            float dy = m_particles.posY[i] - m_neighborCache.prevPosY[i];
            float d2 = dx*dx + dy*dy;
            if (d2 > maxDispSq) {
                maxDispSq = d2;
            }
        }
        if (maxDispSq > threshold) {
            forceRebuild = true;
        }
    }

    if (forceRebuild || m_neighborCache.needsRebuild()) {
        m_spatialHash.build(m_particles.posX.data(), m_particles.posY.data(), pCount);
        buildNeighborLists();
        m_neighborCache.markRebuilt();
    } else {
        m_neighborCache.age();
    }

    // 4. 计算 SPH 流体压力与合力
    m_sphSolver.updateSPH(m_particles, m_neighborCache.list, config);

    // 4.5. Velocity Verlet 阶段2
    integrateStep2(sdt);

    // 5. 碰撞检测与求解响应
    resolveCollisions();
}

void PhysicsWorld::buildNeighborLists() {
    int pCount = m_particles.count;
    m_neighborCache.list.resize(pCount);
    
    // 【极致优化三】多线程无锁 CSR 快速平铺构建：
    // 通过前缀和计算每个粒子的邻居数量，消除线程竞争与 Dynamic 堆分配
    std::vector<std::vector<int>> tempNbrs(pCount);
    float h = config.fluidRadius;
    
    #pragma omp parallel
    {
        std::vector<int> localNbrs;
        localNbrs.reserve(64);
        
        #pragma omp for schedule(static, 128)
        for (int i = 0; i < pCount; ++i) {
            if (m_particles.type[i] != ParticleType::Fluid) continue;
            
            Vec2f pos(m_particles.posX[i], m_particles.posY[i]);
            // 过滤：邻域内只收集 Fluid 粒子作为其邻居，让 SIMD 内部循环不需要任何分支过滤
            m_spatialHash.query(pos, h, m_particles.posX.data(), m_particles.posY.data(), localNbrs);
            
            std::vector<int> fluidNbrs;
            fluidNbrs.reserve(localNbrs.size());
            for (int idx : localNbrs) {
                if (m_particles.type[idx] == ParticleType::Fluid) {
                    fluidNbrs.push_back(idx);
                }
            }
            tempNbrs[i] = std::move(fluidNbrs);
        }
    }
    
    // 计算前缀和前驱索引偏移
    int totalNbrs = 0;
    for (int i = 0; i < pCount; ++i) {
        m_neighborCache.list.begin[i] = totalNbrs;
        totalNbrs += tempNbrs[i].size();
        m_neighborCache.list.end[i] = totalNbrs;
    }
    
    m_neighborCache.list.data.resize(totalNbrs);
    
    // 多线程无冲突填充平铺一维数组
    #pragma omp parallel for schedule(static, 128)
    for (int i = 0; i < pCount; ++i) {
        int start = m_neighborCache.list.begin[i];
        int size = tempNbrs[i].size();
        for (int j = 0; j < size; ++j) {
            m_neighborCache.list.data[start + j] = tempNbrs[i][j];
        }
    }
    
    // 同步记录此时的位置
    m_neighborCache.prevPosX = m_particles.posX;
    m_neighborCache.prevPosY = m_particles.posY;
}

void PhysicsWorld::emitParticles(float sdt) {
    for (auto& emitter : m_emitters) {
        int count = emitter.update(sdt);
        for (int i = 0; i < count; ++i) {
            float spawnAngle = emitter.angle + randomFloat(-emitter.spread * 0.5f, emitter.spread * 0.5f);
            Vec2f dir(std::cos(spawnAngle), std::sin(spawnAngle));
            
            Vec2f vel = dir * emitter.speed;
            Vec2f posOffset = dir * emitter.particleRadius * 1.5f + Vec2f(randomFloat(-1.0f, 1.0f), randomFloat(-1.0f, 1.0f));
            Vec2f spawnPos = emitter.position + posOffset;

            // 限制粒子总数，防止爆发式溢出 (粒子上限设为 8000)
            if (m_particles.count >= 8000) {
                m_particles.erase(0);
            }

            if (emitter.type == EmitterType::NormalParticle) {
                m_particles.addParticle(spawnPos, vel, 1.0f, emitter.particleRadius, false, ParticleType::Normal);
            } else if (emitter.type == EmitterType::SPHFluid) {
                m_particles.addParticle(spawnPos, vel, config.fluidMass, emitter.particleRadius, false, ParticleType::Fluid);
            }
        }
    }
}

void PhysicsWorld::applyForces(float sdt) {
    // A. 施加于粒子系统
    int pCount = m_particles.count;
    #pragma omp parallel for schedule(static, 128)
    for (int i = 0; i < pCount; ++i) {
        if (m_particles.isStatic[i]) continue;
        m_particles.forceX[i] += config.gravity.x * m_particles.mass[i];
        m_particles.forceY[i] += config.gravity.y * m_particles.mass[i];
        
        // 微弱空气阻尼
        m_particles.forceX[i] -= m_particles.velX[i] * 0.28f * m_particles.mass[i];
        m_particles.forceY[i] -= m_particles.velY[i] * 0.28f * m_particles.mass[i];
    }

    // B. 施加于刚体系统
    for (auto& b : m_rigidBodies) {
        if (b.type == BodyType::Static) continue;
        b.force += config.gravity * b.mass;
        b.force -= b.velocity * 0.1f * b.mass;
        b.torque -= b.angularVelocity * 0.1f * b.inertia;
    }
}

void PhysicsWorld::integrateStep1(float sdt) {
    // A. 粒子 Velocity Verlet Step 1 (SoA 适配)
    int pCount = m_particles.count;
    #pragma omp parallel for schedule(static, 128)
    for (int i = 0; i < pCount; ++i) {
        if (m_particles.isStatic[i]) continue;
        
        m_particles.oldPosX[i] = m_particles.posX[i];
        m_particles.oldPosY[i] = m_particles.posY[i];
        
        // 安全速度钳制
        float maxSpeed = 900.0f;
        float vSq = m_particles.velX[i]*m_particles.velX[i] + m_particles.velY[i]*m_particles.velY[i];
        if (vSq > maxSpeed * maxSpeed) {
            float len = std::sqrt(vSq);
            m_particles.velX[i] = (m_particles.velX[i] / len) * maxSpeed;
            m_particles.velY[i] = (m_particles.velY[i] / len) * maxSpeed;
        }
        
        float accX = m_particles.forceX[i] / m_particles.mass[i];
        float accY = m_particles.forceY[i] / m_particles.mass[i];
        
        m_particles.posX[i] += m_particles.velX[i] * sdt + accX * (0.5f * sdt * sdt);
        m_particles.posY[i] += m_particles.velY[i] * sdt + accY * (0.5f * sdt * sdt);
        
        m_particles.velX[i] += accX * (0.5f * sdt);
        m_particles.velY[i] += accY * (0.5f * sdt);

        // 位移安全阀
        float dispX = m_particles.posX[i] - m_particles.oldPosX[i];
        float dispY = m_particles.posY[i] - m_particles.oldPosY[i];
        float dispSq = dispX*dispX + dispY*dispY;
        float maxDisplacement = m_particles.radius[i] * 2.0f;
        if (dispSq > maxDisplacement * maxDisplacement) {
            float len = std::sqrt(dispSq);
            m_particles.posX[i] = m_particles.oldPosX[i] + (dispX / len) * maxDisplacement;
            m_particles.posY[i] = m_particles.oldPosY[i] + (dispY / len) * maxDisplacement;
        }
        
        m_particles.forceX[i] = 0.0f;
        m_particles.forceY[i] = 0.0f;
    }

    // B. 刚体 Velocity Verlet Step 1 (未变)
    for (auto& b : m_rigidBodies) {
        if (b.type == BodyType::Static) continue;
        
        Vec2f acc = b.force * b.invMass;
        float angAcc = b.torque * b.invInertia;

        b.position += b.velocity * sdt + acc * (0.5f * sdt * sdt);
        b.velocity += acc * (0.5f * sdt);

        b.angle += b.angularVelocity * sdt + angAcc * (0.5f * sdt * sdt);
        b.angularVelocity += angAcc * (0.5f * sdt);

        b.updateGlobalVertices();
        b.clearForces();
    }
}

void PhysicsWorld::integrateStep2(float sdt) {
    // A. 粒子 Velocity Verlet Step 2
    int pCount = m_particles.count;
    #pragma omp parallel for schedule(static, 128)
    for (int i = 0; i < pCount; ++i) {
        if (m_particles.isStatic[i]) continue;
        
        float accX = m_particles.forceX[i] / m_particles.mass[i];
        float accY = m_particles.forceY[i] / m_particles.mass[i];
        m_particles.velX[i] += accX * (0.5f * sdt);
        m_particles.velY[i] += accY * (0.5f * sdt);

        float maxSpeed = 900.0f;
        float vSq = m_particles.velX[i]*m_particles.velX[i] + m_particles.velY[i]*m_particles.velY[i];
        if (vSq > maxSpeed * maxSpeed) {
            float len = std::sqrt(vSq);
            m_particles.velX[i] = (m_particles.velX[i] / len) * maxSpeed;
            m_particles.velY[i] = (m_particles.velY[i] / len) * maxSpeed;
        }
    }

    // B. 刚体 Velocity Verlet Step 2 (未变)
    for (auto& b : m_rigidBodies) {
        if (b.type == BodyType::Static) continue;
        
        Vec2f acc = b.force * b.invMass;
        float angAcc = b.torque * b.invInertia;

        b.velocity += acc * (0.5f * sdt);
        b.angularVelocity += angAcc * (0.5f * sdt);

        float maxSpeed = 2000.0f; 
        if (b.velocity.lengthSquared() > maxSpeed * maxSpeed) {
            b.velocity = b.velocity.normalized() * maxSpeed;
        }

        float maxAngular = 50.0f;
        if (std::abs(b.angularVelocity) > maxAngular) {
            b.angularVelocity = (b.angularVelocity > 0.0f) ? maxAngular : -maxAngular;
        }
    }
}

void PhysicsWorld::resolveBoundaryCollisions(int i) {
    float r = m_particles.radius[i];
    float e = config.boundaryDamping;

    // 左边界
    if (m_particles.posX[i] - r < 0.0f) {
        m_particles.posX[i] = r;
        m_particles.velX[i] = -m_particles.velX[i] * e;
    }
    // 右边界
    else if (m_particles.posX[i] + r > config.worldWidth) {
        m_particles.posX[i] = config.worldWidth - r;
        m_particles.velX[i] = -m_particles.velX[i] * e;
    }

    // 上边界
    if (m_particles.posY[i] - r < 0.0f) {
        m_particles.posY[i] = r;
        m_particles.velY[i] = -m_particles.velY[i] * e;
    }
    // 下边界
    else if (m_particles.posY[i] + r > config.worldHeight) {
        m_particles.posY[i] = config.worldHeight - r;
        m_particles.velY[i] = -m_particles.velY[i] * e;
        
        float frictionFactor = (m_particles.type[i] == ParticleType::Fluid) ? 0.998f : 0.95f;
        m_particles.velX[i] *= frictionFactor; 
    }
}

void PhysicsWorld::resolveWallCollisions(int i) {
    if (m_particles.isStatic[i]) return;

    float px = m_particles.posX[i];
    float py = m_particles.posY[i];
    float r = m_particles.radius[i];

    for (const auto& wall : m_walls) {
        // AABB Culling
        if (px < wall.minBound.x - r || px > wall.maxBound.x + r ||
            py < wall.minBound.y - r || py > wall.maxBound.y + r) {
            continue;
        }

        Vec2f ab = wall.end - wall.start;
        
        // 1. CCD (仅对非流体粒子进行，流体粒子有速度限值且子步极短，免去叉乘几何开销)
        if (m_particles.type[i] != ParticleType::Fluid) {
            float trajX = px - m_particles.oldPosX[i];
            float trajY = py - m_particles.oldPosY[i];
            float cross_traj_ab = trajX * ab.y - trajY * ab.x;
            
            if (std::abs(cross_traj_ab) > 1e-5f) {
                float diffX = wall.start.x - m_particles.oldPosX[i];
                float diffY = wall.start.y - m_particles.oldPosY[i];
                
                float t = (diffX * ab.y - diffY * ab.x) / cross_traj_ab;
                float u = (diffX * trajY - diffY * trajX) / cross_traj_ab;
                
                if (t > 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f) {
                    float interX = m_particles.oldPosX[i] + trajX * t;
                    float interY = m_particles.oldPosY[i] + trajY * t;
                    
                    Vec2f wallNormal(-ab.y, ab.x);
                    wallNormal.normalize();
                    
                    if (trajX * wallNormal.x + trajY * wallNormal.y > 0.0f) {
                        wallNormal = -wallNormal;
                    }
                    
                    m_particles.posX[i] = interX + wallNormal.x * (r + 0.1f);
                    m_particles.posY[i] = interY + wallNormal.y * (r + 0.1f);
                    
                    float vNormal = m_particles.velX[i] * wallNormal.x + m_particles.velY[i] * wallNormal.y;
                    float elasticity = config.particleElasticity;
                    Vec2f tangent(-wallNormal.y, wallNormal.x);
                    float vTangent = m_particles.velX[i] * tangent.x + m_particles.velY[i] * tangent.y;
                    float frictionFactor = (m_particles.type[i] == ParticleType::Fluid) ? 0.998f : 0.95f;
                    
                    m_particles.velX[i] = tangent.x * vTangent * frictionFactor - wallNormal.x * vNormal * elasticity;
                    m_particles.velY[i] = tangent.y * vTangent * frictionFactor - wallNormal.y * vNormal * elasticity;
                    continue;
                }
            }
        }
        
        // 2. DCD
        float apX = px - wall.start.x;
        float apY = py - wall.start.y;
        float abLenSq = ab.lengthSquared();
        if (abLenSq == 0.0f) continue;

        float t = std::max(0.0f, std::min(1.0f, (apX * ab.x + apY * ab.y) / abLenSq));
        float closestX = wall.start.x + ab.x * t;
        float closestY = wall.start.y + ab.y * t;
        
        float diffX = px - closestX;
        float diffY = py - closestY;
        float distSq = diffX*diffX + diffY*diffY;

        if (distSq > 0.0001f && distSq < r * r) {
            float dist = std::sqrt(distSq);
            float normalX = diffX / dist;
            float normalY = diffY / dist;
            float penetration = r - dist;

            m_particles.posX[i] += normalX * penetration;
            m_particles.posY[i] += normalY * penetration;

            float vNormal = m_particles.velX[i] * normalX + m_particles.velY[i] * normalY;
            if (vNormal < 0.0f) {
                float elasticity = config.particleElasticity;
                Vec2f tangent(-normalY, normalX);
                float vTangent = m_particles.velX[i] * tangent.x + m_particles.velY[i] * tangent.y;
                float frictionFactor = (m_particles.type[i] == ParticleType::Fluid) ? 0.998f : 0.95f;
                m_particles.velX[i] = tangent.x * vTangent * frictionFactor - normalX * vNormal * elasticity;
                m_particles.velY[i] = tangent.y * vTangent * frictionFactor - normalY * vNormal * elasticity;
            }
        }
    }
}

void PhysicsWorld::resolveCollisions() {
    int pCount = m_particles.count;
    int rbCount = static_cast<int>(m_rigidBodies.size());

    // 1. 粒子-粒子自碰撞 (仅针对非双流体对，利用空间网格加速)
    if (pCount > 0) {
        std::vector<int> neighbors;
        neighbors.reserve(64);
        
        const float* posX = m_particles.posX.data();
        const float* posY = m_particles.posY.data();
        
        for (int i = 0; i < pCount; ++i) {
            float px = m_particles.posX[i];
            float py = m_particles.posY[i];
            float pr = m_particles.radius[i];
            ParticleType typeA = m_particles.type[i];
            
            float searchRadius = pr * 2.5f;
            Vec2f pPos(px, py);
            
            m_spatialHash.query(pPos, searchRadius, posX, posY, neighbors);

            for (int jIdx : neighbors) {
                if (jIdx <= i) continue;

                // 【核心修复】流体粒子之间纯靠 SPH 压力排斥，严禁引入硬碰撞叠加修正！
                if (typeA == ParticleType::Fluid && m_particles.type[jIdx] == ParticleType::Fluid) {
                    continue;
                }
                
                float dx = m_particles.posX[jIdx] - px;
                float dy = m_particles.posY[jIdx] - py;
                float distSq = dx*dx + dy*dy;
                float minDist = pr + m_particles.radius[jIdx];

                if (distSq < minDist * minDist) {
                    float dist = std::sqrt(distSq);
                    Vec2f normal(0.0f, -1.0f);
                    if (dist > 0.0f) {
                        normal = Vec2f(dx / dist, dy / dist);
                    }
                    float penetration = minDist - dist;

                    float massRatioA = m_particles.isStatic[i] ? 0.0f : (m_particles.isStatic[jIdx] ? 1.0f : 0.5f);
                    float massRatioB = m_particles.isStatic[jIdx] ? 0.0f : (m_particles.isStatic[i] ? 1.0f : 0.5f);

                    m_particles.posX[i] -= normal.x * penetration * massRatioA;
                    m_particles.posY[i] -= normal.y * penetration * massRatioA;
                    m_particles.posX[jIdx] += normal.x * penetration * massRatioB;
                    m_particles.posY[jIdx] += normal.y * penetration * massRatioB;

                    float relativeVelX = m_particles.velX[jIdx] - m_particles.velX[i];
                    float relativeVelY = m_particles.velY[jIdx] - m_particles.velY[i];
                    float vNormal = relativeVelX * normal.x + relativeVelY * normal.y;

                    if (vNormal < 0.0f) {
                        float elasticity = config.particleElasticity;
                        float invMassSum = (m_particles.isStatic[i] ? 0.0f : 1.0f / m_particles.mass[i]) + 
                                           (m_particles.isStatic[jIdx] ? 0.0f : 1.0f / m_particles.mass[jIdx]);
                        if (invMassSum > 0.0f) {
                            float impulseScalar = -(1.0f + elasticity) * vNormal / invMassSum;
                            
                            if (!m_particles.isStatic[i]) {
                                m_particles.velX[i] -= normal.x * impulseScalar / m_particles.mass[i];
                                m_particles.velY[i] -= normal.y * impulseScalar / m_particles.mass[i];
                            }
                            if (!m_particles.isStatic[jIdx]) {
                                m_particles.velX[jIdx] += normal.x * impulseScalar / m_particles.mass[jIdx];
                                m_particles.velY[jIdx] += normal.y * impulseScalar / m_particles.mass[jIdx];
                            }
                        }
                    }
                }
            }

            resolveBoundaryCollisions(i);
            resolveWallCollisions(i);
        }
    }

    // 2. 粒子-刚体碰撞解算 (水流冲刷/阻挡刚体)
    if (pCount > 0 && rbCount > 0) {
        resolveParticleRigidCollisions();
    }

    // 2.5 粒子二次加固校验 (防止穿透)
    if (pCount > 0) {
        for (int pass = 0; pass < 2; ++pass) {
            #pragma omp parallel for schedule(static, 128)
            for (int i = 0; i < pCount; ++i) {
                if (m_particles.isStatic[i]) continue;
                resolveBoundaryCollisions(i);
                resolveWallCollisions(i);
            }
        }
        
        #pragma omp parallel for schedule(static, 128)
        for (int i = 0; i < pCount; ++i) {
            if (m_particles.isStatic[i]) continue;
            float maxSpeed = 900.0f;
            float vSq = m_particles.velX[i]*m_particles.velX[i] + m_particles.velY[i]*m_particles.velY[i];
            if (vSq > maxSpeed * maxSpeed) {
                float len = std::sqrt(vSq);
                m_particles.velX[i] = (m_particles.velX[i] / len) * maxSpeed;
                m_particles.velY[i] = (m_particles.velY[i] / len) * maxSpeed;
            }
        }
    }

    // 3. 刚体-刚体碰撞检测与求解 (SAT 分离轴定理与冲量求解器)
    for (int i = 0; i < rbCount; ++i) {
        for (int j = i + 1; j < rbCount; ++j) {
            ContactManifold manifold;
            if (Collider::detectCollision(manifold, &m_rigidBodies[i], &m_rigidBodies[j])) {
                RigidBodySolver::resolveCollision(manifold, config);
                RigidBodySolver::positionCorrection(manifold, config);
            }
        }
    }

    // 4. 刚体-边界碰撞解算
    for (int i = 0; i < rbCount; ++i) {
        resolveRigidBoundaryCollisions(m_rigidBodies[i]);
    }

    // 5. 刚体-墙体碰撞解算
    for (int i = 0; i < rbCount; ++i) {
        resolveRigidWallCollisions(m_rigidBodies[i]);
    }
}

void PhysicsWorld::resolveRigidBoundaryCollisions(RigidBody& b) {
    if (b.type == BodyType::Static) return;

    if (b.shape == ShapeType::Circle) {
        float r = b.radius;
        float e = b.restitution;

        // 左右边界
        if (b.position.x - r < 0.0f) {
            b.position.x = r;
            b.velocity.x = -b.velocity.x * e;
        } else if (b.position.x + r > config.worldWidth) {
            b.position.x = config.worldWidth - r;
            b.velocity.x = -b.velocity.x * e;
        }

        // 上下边界
        if (b.position.y - r < 0.0f) {
            b.position.y = r;
            b.velocity.y = -b.velocity.y * e;
        } else if (b.position.y + r > config.worldHeight) {
            b.position.y = config.worldHeight - r;
            b.velocity.y = -b.velocity.y * e;
            b.velocity.x *= 0.9f;            // 地面粗糙摩擦力
            b.angularVelocity *= 0.9f;
        }
    }
    else if (b.shape == ShapeType::Polygon) {
        b.updateGlobalVertices();
        Vec2f correction(0.0f, 0.0f);
        int collisionCount = 0;

        float e = b.restitution;

        for (const auto& v : b.globalVertices) {
            // 左边界
            if (v.x < 0.0f) {
                correction.x += -v.x;
                b.velocity.x = std::abs(b.velocity.x) * e;
                collisionCount++;
            }
            // 右边界
            else if (v.x > config.worldWidth) {
                correction.x += config.worldWidth - v.x;
                b.velocity.x = -std::abs(b.velocity.x) * e;
                collisionCount++;
            }

            // 上边界
            if (v.y < 0.0f) {
                correction.y += -v.y;
                b.velocity.y = std::abs(b.velocity.y) * e;
                collisionCount++;
            }
            // 下边界
            else if (v.y > config.worldHeight) {
                correction.y += config.worldHeight - v.y;
                b.velocity.y = -std::abs(b.velocity.y) * e;
                b.velocity.x *= 0.92f;        // 地面摩擦阻力
                b.angularVelocity *= 0.92f;
                collisionCount++;
            }
        }

        if (collisionCount > 0) {
            b.position += correction / static_cast<float>(collisionCount);
            b.updateGlobalVertices();
        }
    }
}

void PhysicsWorld::resolveParticleRigidCollisions() {
    int pCount = m_particles.count;

    // 【极致优化五】并行化碰撞受力收集，使用 atomic 写入刚体，实现完美的线程无锁
    #pragma omp parallel for schedule(dynamic, 64)
    for (int i = 0; i < pCount; ++i) {
        if (m_particles.isStatic[i]) continue;
        
        float pX = m_particles.posX[i];
        float pY = m_particles.posY[i];
        float pR = m_particles.radius[i];
        float pM = m_particles.mass[i];

        for (auto& b : m_rigidBodies) {
            if (b.shape == ShapeType::Circle) {
                float diffX = pX - b.position.x;
                float diffY = pY - b.position.y;
                float distSq = diffX*diffX + diffY*diffY;
                float minDist = pR + b.radius;

                if (distSq < minDist * minDist && distSq > 0.0f) {
                    float dist = std::sqrt(distSq);
                    float normalX = diffX / dist;
                    float normalY = diffY / dist;
                    float penetration = minDist - dist;

                    float invMassSum = 1.0f / pM + b.invMass;
                    m_particles.posX[i] += normalX * penetration * (1.0f / pM / invMassSum);
                    m_particles.posY[i] += normalY * penetration * (1.0f / pM / invMassSum);
                    
                    if (b.type != BodyType::Static) {
                        float corrX = -normalX * penetration * (b.invMass / invMassSum);
                        float corrY = -normalY * penetration * (b.invMass / invMassSum);
                        #pragma omp atomic
                        b.position.x += corrX;
                        #pragma omp atomic
                        b.position.y += corrY;
                    }

                    float relativeVelX = m_particles.velX[i] - b.velocity.x;
                    float relativeVelY = m_particles.velY[i] - b.velocity.y;
                    float vNormal = relativeVelX * normalX + relativeVelY * normalY;

                    if (vNormal < 0.0f) {
                        float elasticity = std::min(config.particleElasticity, b.restitution);
                        float j = -(1.0f + elasticity) * vNormal / invMassSum;
                        
                        m_particles.velX[i] += normalX * j / pM;
                        m_particles.velY[i] += normalY * j / pM;
                        
                        if (b.type != BodyType::Static) {
                            float dvx = -normalX * j * b.invMass;
                            float dvy = -normalY * j * b.invMass;
                            #pragma omp atomic
                            b.velocity.x += dvx;
                            #pragma omp atomic
                            b.velocity.y += dvy;
                        }
                    }
                }
            }
            else if (b.shape == ShapeType::Polygon) {
                int count = static_cast<int>(b.globalVertices.size());
                float minDistance = std::numeric_limits<float>::max();
                Vec2f closestPoint;
                Vec2f bestNormal;

                for (int j = 0; j < count; ++j) {
                    Vec2f v1 = b.globalVertices[j];
                    Vec2f v2 = b.globalVertices[(j + 1) % count];
                    Vec2f edge = v2 - v1;
                    float apX = pX - v1.x;
                    float apY = pY - v1.y;

                    float edgeLenSq = edge.lengthSquared();
                    if (edgeLenSq == 0.0f) continue;

                    float t = (apX * edge.x + apY * edge.y) / edgeLenSq;
                    t = std::max(0.0f, std::min(1.0f, t));

                    Vec2f pt = v1 + edge * t;
                    float dx = pX - pt.x;
                    float dy = pY - pt.y;
                    float dist = std::sqrt(dx*dx + dy*dy);

                    if (dist < minDistance) {
                        minDistance = dist;
                        closestPoint = pt;
                        bestNormal = b.globalNormals[j];
                    }
                }

                Vec2f pPos(pX, pY);
                bool inside = Collider::isPointInPolygon(pPos, b);

                if (minDistance < pR || inside) {
                    Vec2f normal = bestNormal;
                    if (inside) {
                        minDistance = -minDistance; 
                    }
                    float penetration = pR - minDistance;

                    float invMassSum = 1.0f / pM + b.invMass;
                    m_particles.posX[i] += normal.x * penetration * (1.0f / pM / invMassSum);
                    m_particles.posY[i] += normal.y * penetration * (1.0f / pM / invMassSum);
                    
                    if (b.type != BodyType::Static) {
                        float corrX = -normal.x * penetration * (b.invMass / invMassSum);
                        float corrY = -normal.y * penetration * (b.invMass / invMassSum);
                        #pragma omp atomic
                        b.position.x += corrX;
                        #pragma omp atomic
                        b.position.y += corrY;
                    }

                    float relativeVelX = m_particles.velX[i] - b.velocity.x;
                    float relativeVelY = m_particles.velY[i] - b.velocity.y;
                    float vNormal = relativeVelX * normal.x + relativeVelY * normal.y;

                    if (vNormal < 0.0f) {
                        float elasticity = std::min(config.particleElasticity, b.restitution);
                        float j = -(1.0f + elasticity) * vNormal / invMassSum;

                        m_particles.velX[i] += normal.x * j / pM;
                        m_particles.velY[i] += normal.y * j / pM;
                        
                        if (b.type != BodyType::Static) {
                            float dvx = -normal.x * j * b.invMass;
                            float dvy = -normal.y * j * b.invMass;
                            #pragma omp atomic
                            b.velocity.x += dvx;
                            #pragma omp atomic
                            b.velocity.y += dvy;
                        }
                    }
                }
            }
        }
    }

    for (auto& b : m_rigidBodies) {
        if (b.type != BodyType::Static) {
            b.updateGlobalVertices();
        }
    }
}

void PhysicsWorld::resolveRigidWallCollisions(RigidBody& b) {
    if (b.type == BodyType::Static) return;
    if (m_walls.empty()) return;

    float e = b.restitution;

    if (b.shape == ShapeType::Circle) {
        for (const auto& wall : m_walls) {
            if (b.position.x < wall.minBound.x - b.radius || b.position.x > wall.maxBound.x + b.radius ||
                b.position.y < wall.minBound.y - b.radius || b.position.y > wall.maxBound.y + b.radius) {
                continue;
            }

            Vec2f ab = wall.end - wall.start;
            float abLenSq = ab.lengthSquared();
            if (abLenSq == 0.0f) continue;

            Vec2f wallNormal(-ab.y, ab.x);
            float wallLen = wallNormal.length();
            wallNormal = wallNormal / wallLen;

            if ((b.position - wall.start).dot(wallNormal) < 0.0f) {
                wallNormal *= -1.0f;
            }

            Vec2f ap = b.position - wall.start;
            float t = ap.dot(ab) / abLenSq;
            float tClamped = std::max(0.0f, std::min(1.0f, t));
            Vec2f closest = wall.start + ab * tClamped;

            Vec2f toCenter = b.position - closest;
            float signedDist = toCenter.dot(wallNormal);
            float dist = toCenter.length();

            float tangentDistSq = std::max(0.0f, dist * dist - signedDist * signedDist);

            if (signedDist < b.radius && tangentDistSq < b.radius * b.radius) {
                Vec2f normal = wallNormal;
                float penetration = b.radius - signedDist;

                if (signedDist > 0.0f && dist > 0.0f && (t < 0.0f || t > 1.0f)) {
                    normal = toCenter / dist;
                    penetration = b.radius - dist;
                    if (dist > b.radius) continue; 
                }

                b.position += normal * penetration;

                Vec2f r = closest - b.position;
                Vec2f vRel = b.velocity + Vec2f(-b.angularVelocity * r.y, b.angularVelocity * r.x);
                float vNormal = vRel.dot(normal);

                if (vNormal < 0.0f) {
                    float rCrossN = r.cross(normal);
                    float rotMass = rCrossN * rCrossN * b.invInertia;
                    float j = -(1.0f + e) * vNormal / (b.invMass + rotMass);
                    
                    Vec2f impulse = normal * j;
                    b.applyImpulse(impulse, r);

                    vRel = b.velocity + Vec2f(-b.angularVelocity * r.y, b.angularVelocity * r.x);
                    Vec2f tangent(-normal.y, normal.x);
                    float vTangent = vRel.dot(tangent);
                    float rCrossT = r.cross(tangent);
                    float rotMassT = rCrossT * rCrossT * b.invInertia;
                    float jt = -vTangent / (b.invMass + rotMassT);
                    
                    float maxJt = j * b.friction;
                    jt = std::max(-maxJt, std::min(maxJt, jt));
                    
                    b.applyImpulse(tangent * jt, r);
                }
            }
        }
    }
    else if (b.shape == ShapeType::Polygon) {
        b.updateGlobalVertices();

        float polyMinX = b.globalVertices[0].x, polyMaxX = b.globalVertices[0].x;
        float polyMinY = b.globalVertices[0].y, polyMaxY = b.globalVertices[0].y;
        for (const auto& v : b.globalVertices) {
            if (v.x < polyMinX) polyMinX = v.x;
            if (v.x > polyMaxX) polyMaxX = v.x;
            if (v.y < polyMinY) polyMinY = v.y;
            if (v.y > polyMaxY) polyMaxY = v.y;
        }

        for (const auto& wall : m_walls) {
            if (polyMaxX < wall.minBound.x || polyMinX > wall.maxBound.x ||
                polyMaxY < wall.minBound.y || polyMinY > wall.maxBound.y) {
                continue;
            }

            Vec2f ab = wall.end - wall.start;
            float abLenSq = ab.lengthSquared();
            if (abLenSq == 0.0f) continue;

            Vec2f wallNormal(-ab.y, ab.x);
            float wallLen = wallNormal.length();
            if (wallLen == 0.0f) continue;
            wallNormal = wallNormal / wallLen;

            Vec2f toBody = b.position - wall.start;
            if (toBody.dot(wallNormal) < 0.0f) {
                wallNormal = wallNormal * (-1.0f);
            }

            Vec2f totalCorrection(0.0f, 0.0f);
            int collisionCount = 0;
            float maxPenetration = 0.0f;
            Vec2f bestContact = b.position; 

            for (const auto& v : b.globalVertices) {
                Vec2f vp = v - wall.start;
                float tProj = vp.dot(ab) / abLenSq;
                float tClamped = std::max(0.0f, std::min(1.0f, tProj));

                Vec2f closest = wall.start + ab * tClamped;
                Vec2f toVertex = v - closest;
                float dist = toVertex.length();
                float signedDist = toVertex.dot(wallNormal);

                float tangentDistSq = std::max(0.0f, dist * dist - signedDist * signedDist);

                float threshold = 2.0f; 
                if (signedDist < threshold && tangentDistSq < 400.0f) {
                    float penetration = threshold - signedDist;
                    totalCorrection += wallNormal * penetration;
                    collisionCount++;
                    if (penetration > maxPenetration) {
                        maxPenetration = penetration;
                        bestContact = v; 
                    }
                }
            }

            if (collisionCount > 0) {
                b.position += totalCorrection / static_cast<float>(collisionCount);
                b.updateGlobalVertices();

                Vec2f r = bestContact - b.position;
                Vec2f vRel = b.velocity + Vec2f(-b.angularVelocity * r.y, b.angularVelocity * r.x);
                float vNormal = vRel.dot(wallNormal);

                if (vNormal < 0.0f) {
                    float rCrossN = r.cross(wallNormal);
                    float rotMass = rCrossN * rCrossN * b.invInertia;
                    float j = -(1.0f + e) * vNormal / (b.invMass + rotMass);
                    
                    b.applyImpulse(wallNormal * j, r);

                    vRel = b.velocity + Vec2f(-b.angularVelocity * r.y, b.angularVelocity * r.x);
                    Vec2f tangent(-wallNormal.y, wallNormal.x);
                    float vTangent = vRel.dot(tangent);
                    float rCrossT = r.cross(tangent);
                    float rotMassT = rCrossT * rCrossT * b.invInertia;
                    float jt = -vTangent / (b.invMass + rotMassT);
                    
                    float maxJt = j * b.friction;
                    jt = std::max(-maxJt, std::min(maxJt, jt));
                    
                    b.applyImpulse(tangent * jt, r);
                }
            }
        }
    }
}
