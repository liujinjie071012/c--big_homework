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
    m_particles.push_back(p);
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
}

void PhysicsWorld::reset() {
    m_particles.clear();
    m_spatialHash.clear();
    // 重置发射器的累加器
    for (auto& e : m_emitters) {
        e.accumulator = 0.0f;
    }
}

void PhysicsWorld::step() {
    // 强制执行一个时间步
    update(config.timeStep);
}

void PhysicsWorld::update(float dt) {
    // 使用子步积分提升碰撞与约束求解的物理稳定性
    float sdt = dt / static_cast<float>(config.substeps);
    for (int step = 0; step < config.substeps; ++step) {
        substep(sdt);
    }
}

void PhysicsWorld::substep(float sdt) {
    // 1. 发发射器喷出新粒子
    emitParticles(sdt);

    // 2. 累加外力（如重力、阻力）
    applyForces(sdt);

    // 3. 构建空间哈希网格，并更新 SPH 流体压力与受力
    int pCount = static_cast<int>(m_particles.size());
    if (pCount > 0) {
        m_positionsCache.resize(pCount);
        for (int i = 0; i < pCount; ++i) {
            m_positionsCache[i] = m_particles[i].position;
        }
        m_spatialHash.build(m_positionsCache);
        m_sphSolver.updateSPH(m_particles, m_spatialHash, config, m_positionsCache);
    }

    // 4. 数值积分（先更新速度，便于计算碰撞速度）
    integrate(sdt);

    // 5. 碰撞检测与求解响应
    resolveCollisions();
}

void PhysicsWorld::emitParticles(float sdt) {
    for (auto& emitter : m_emitters) {
        int count = emitter.update(sdt);
        for (int i = 0; i < count; ++i) {
            // 在发射方向上加入随机扩散角
            float spawnAngle = emitter.angle + randomFloat(-emitter.spread * 0.5f, emitter.spread * 0.5f);
            Vec2f dir(std::cos(spawnAngle), std::sin(spawnAngle));
            
            // 喷射初始速度与微小位置抖动防止重叠
            Vec2f vel = dir * emitter.speed;
            Vec2f posOffset = dir * emitter.particleRadius * 1.5f + Vec2f(randomFloat(-1.0f, 1.0f), randomFloat(-1.0f, 1.0f));
            Vec2f spawnPos = emitter.position + posOffset;

            // 限制粒子总数，防止爆发式溢出 (粒子上限设为 8000)
            if (m_particles.size() >= 8000) {
                // 移除最早产生的粒子以形成循环流体/粒子效果
                m_particles.erase(m_particles.begin());
            }

            if (emitter.type == EmitterType::NormalParticle) {
                m_particles.emplace_back(spawnPos, vel, 1.0f, emitter.particleRadius, false, ParticleType::Normal);
            } else if (emitter.type == EmitterType::SPHFluid) {
                m_particles.emplace_back(spawnPos, vel, config.fluidMass, emitter.particleRadius, false, ParticleType::Fluid);
            }
        }
    }
}

void PhysicsWorld::applyForces(float sdt) {
    // A. 施加于粒子系统
    for (auto& p : m_particles) {
        if (p.isStatic) continue;
        p.force += config.gravity * p.mass;
        p.force -= p.velocity * 0.15f * p.mass; // 微弱阻力
    }

    // B. 施加于刚体系统
    for (auto& b : m_rigidBodies) {
        if (b.type == BodyType::Static) continue;
        b.force += config.gravity * b.mass;
        // 施加平动与转动微弱空气阻尼
        b.force -= b.velocity * 0.1f * b.mass;
        b.torque -= b.angularVelocity * 0.1f * b.inertia;
    }
}

void PhysicsWorld::integrate(float sdt) {
    // A. 积分粒子系统
    for (auto& p : m_particles) {
        if (p.isStatic) continue;
        p.velocity += (p.force / p.mass) * sdt;

        // 限制粒子最大速度，防止高压下速度爆炸导致穿墙
        float maxSpeed = 1500.0f;
        if (p.velocity.lengthSquared() > maxSpeed * maxSpeed) {
            p.velocity = p.velocity.normalized() * maxSpeed;
        }

        p.position += p.velocity * sdt;
        p.force = Vec2f(0.0f, 0.0f);
    }

    // B. 积分刚体系统
    for (auto& b : m_rigidBodies) {
        if (b.type == BodyType::Static) continue;
        
        // 辛欧拉平动积分
        b.velocity += (b.force * b.invMass) * sdt;
        
        // 限制最大平动速度，防止在高压挤压下速度爆炸导致穿模
        float maxSpeed = 2000.0f; 
        if (b.velocity.lengthSquared() > maxSpeed * maxSpeed) {
            b.velocity = b.velocity.normalized() * maxSpeed;
        }

        b.position += b.velocity * sdt;

        // 辛欧拉转动积分
        b.angularVelocity += (b.torque * b.invInertia) * sdt;
        
        // 限制最大角速度
        float maxAngular = 50.0f;
        if (std::abs(b.angularVelocity) > maxAngular) {
            b.angularVelocity = (b.angularVelocity > 0.0f) ? maxAngular : -maxAngular;
        }

        b.angle += b.angularVelocity * sdt;

        // 重新缓存刚体的世界顶点坐标数据
        b.updateGlobalVertices();
        b.clearForces();
    }
}

void PhysicsWorld::resolveBoundaryCollisions(Particle& p) {
    float r = p.radius;
    float e = config.boundaryDamping;

    // 左边界
    if (p.position.x - r < 0.0f) {
        p.position.x = r;
        p.velocity.x = -p.velocity.x * e;
    }
    // 右边界
    else if (p.position.x + r > config.worldWidth) {
        p.position.x = config.worldWidth - r;
        p.velocity.x = -p.velocity.x * e;
    }

    // 上边界
    if (p.position.y - r < 0.0f) {
        p.position.y = r;
        p.velocity.y = -p.velocity.y * e;
    }
    // 下边界 (底座)
    else if (p.position.y + r > config.worldHeight) {
        p.position.y = config.worldHeight - r;
        p.velocity.y = -p.velocity.y * e;
        // 增加摩擦力使球能在地上滚停下来
        p.velocity.x *= 0.95f; 
    }
}

void PhysicsWorld::resolveWallCollisions(Particle& p) {
    if (p.isStatic) return;

    for (const auto& wall : m_walls) {
        Vec2f ab = wall.end - wall.start;
        Vec2f ap = p.position - wall.start;

        float abLenSq = ab.lengthSquared();
        if (abLenSq == 0.0f) continue;

        // 投影求最近点系数 t
        float t = std::max(0.0f, std::min(1.0f, ap.dot(ab) / abLenSq));

        Vec2f closest = wall.start + ab * t;
        Vec2f diff = p.position - closest;
        float distSq = diff.lengthSquared();

        // 放弃单面的 SDF 检测，恢复为双面纯距离检测。
        // 因为我们已经加入了 Velocity Clamping（最大单步位移被限制在 6 像素），
        // 且半径为 3 像素，所以粒子不可能在单步内完全跨越墙体，
        // 纯距离检测不仅完全安全，而且能完美支持双面碰撞，绝不会产生“黑洞吸附”现象。
        if (distSq > 0.0001f && distSq < p.radius * p.radius) {
            float dist = std::sqrt(distSq);
            Vec2f normal = diff / dist;
            float penetration = p.radius - dist;

            // 1. 位置纠正（强制推出）
            p.position += normal * penetration;

            // 2. 速度反射
            float vNormal = p.velocity.dot(normal);
            if (vNormal < 0.0f) {
                float elasticity = config.particleElasticity;
                Vec2f tangent(-normal.y, normal.x);
                float vTangent = p.velocity.dot(tangent);
                p.velocity = tangent * vTangent * 0.95f - normal * vNormal * elasticity;
            }
        }
    }
}

void PhysicsWorld::resolveCollisions() {
    int pCount = static_cast<int>(m_particles.size());
    int rbCount = static_cast<int>(m_rigidBodies.size());

    // ==========================================
    // 1. 粒子-粒子自碰撞解算 (利用空间哈希网格加速)
    // ==========================================
    if (pCount > 0) {
        // 重用缓存的位置向量，用积分后的新位置重建空间哈希
        m_positionsCache.resize(pCount);
        for (int i = 0; i < pCount; ++i) {
            m_positionsCache[i] = m_particles[i].position;
        }
        m_spatialHash.build(m_positionsCache);

        std::vector<int> neighbors;
        for (int i = 0; i < pCount; ++i) {
            Particle& pA = m_particles[i];
            float searchRadius = pA.radius * 2.5f;
            m_spatialHash.query(pA.position, searchRadius, m_positionsCache, neighbors);

            for (int jIdx : neighbors) {
                if (jIdx <= i) continue;

                Particle& pB = m_particles[jIdx];
                
                float dist = pA.position.distance(pB.position);
                float minDist = pA.radius + pB.radius;

                if (dist < minDist) {
                    Vec2f normal(0.0f, -1.0f);
                    if (dist > 0.0f) {
                        normal = (pB.position - pA.position) / dist;
                    }
                    float penetration = minDist - dist;

                    // 位置修正防止粘连沉降
                    float massRatioA = pA.isStatic ? 0.0f : (pB.isStatic ? 1.0f : 0.5f);
                    float massRatioB = pB.isStatic ? 0.0f : (pA.isStatic ? 1.0f : 0.5f);

                    pA.position -= normal * penetration * massRatioA;
                    pB.position += normal * penetration * massRatioB;

                    // 冲量弹性反弹
                    Vec2f relativeVel = pB.velocity - pA.velocity;
                    float vNormal = relativeVel.dot(normal);

                    if (vNormal < 0.0f) {
                        float elasticity = std::min(config.particleElasticity, config.particleElasticity);
                        float invMassSum = (pA.isStatic ? 0.0f : 1.0f / pA.mass) + 
                                           (pB.isStatic ? 0.0f : 1.0f / pB.mass);
                        if (invMassSum > 0.0f) {
                            float impulseScalar = -(1.0f + elasticity) * vNormal / invMassSum;
                            Vec2f impulse = normal * impulseScalar;

                            if (!pA.isStatic) pA.velocity -= impulse / pA.mass;
                            if (!pB.isStatic) pB.velocity += impulse / pB.mass;
                        }
                    }
                }
            }

            // 粒子与边界及静态墙体的碰撞解算
            resolveBoundaryCollisions(pA);
            resolveWallCollisions(pA);
        }
    }

    // ==========================================
    // 2. 粒子-刚体碰撞解算 (水流冲刷/阻挡刚体)
    // ==========================================
    if (pCount > 0 && rbCount > 0) {
        resolveParticleRigidCollisions();
    }

    // ==========================================
    // 2.5 粒子-墙体/边界 二次加固校验 (Post-Pass)
    //     粒子-粒子 和 粒子-刚体 碰撞解算可能把粒子推穿墙壁，
    //     这里做两轮强制位置夹紧，防止高密度下穿透。
    // ==========================================
    if (pCount > 0) {
        for (int pass = 0; pass < 2; ++pass) {
            for (int i = 0; i < pCount; ++i) {
                Particle& p = m_particles[i];
                if (p.isStatic) continue;
                resolveBoundaryCollisions(p);
                resolveWallCollisions(p);
            }
        }
    }

    // ==========================================
    // 3. 刚体-刚体碰撞检测与求解 (SAT 分离轴定理与冲量求解器)
    // ==========================================
    for (int i = 0; i < rbCount; ++i) {
        for (int j = i + 1; j < rbCount; ++j) {
            ContactManifold manifold;
            if (Collider::detectCollision(manifold, &m_rigidBodies[i], &m_rigidBodies[j])) {
                // 求解反冲冲量与摩擦力
                RigidBodySolver::resolveCollision(manifold, config);
                // 位置投影修正，彻底规避刚体穿透
                RigidBodySolver::positionCorrection(manifold, config);
            }
        }
    }

    // ==========================================
    // 4. 刚体-边界碰撞解算
    // ==========================================
    for (int i = 0; i < rbCount; ++i) {
        resolveRigidBoundaryCollisions(m_rigidBodies[i]);
    }

    // ==========================================
    // 5. 刚体-墙体碰撞解算 (用户绘制的墙壁线段)
    // ==========================================
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
    for (auto& p : m_particles) {
        if (p.isStatic) continue;

        for (auto& b : m_rigidBodies) {
            if (b.shape == ShapeType::Circle) {
                float dist = p.position.distance(b.position);
                float minDist = p.radius + b.radius;

                if (dist < minDist && dist > 0.0f) {
                    Vec2f normal = (p.position - b.position) / dist;
                    float penetration = minDist - dist;

                    // 1. 位置纠正
                    float invMassSum = 1.0f / p.mass + b.invMass;
                    p.position += normal * penetration * (1.0f / p.mass / invMassSum);
                    if (b.type != BodyType::Static) {
                        b.position -= normal * penetration * (b.invMass / invMassSum);
                        b.updateGlobalVertices();
                    }

                    // 2. 冲量碰撞响应
                    Vec2f relativeVel = p.velocity - b.velocity;
                    float vNormal = relativeVel.dot(normal);

                    if (vNormal < 0.0f) {
                        float elasticity = std::min(config.particleElasticity, b.restitution);
                        float j = -(1.0f + elasticity) * vNormal / invMassSum;
                        
                        p.velocity += normal * j / p.mass;
                        if (b.type != BodyType::Static) {
                            b.velocity -= normal * j * b.invMass;
                        }
                    }
                }
            }
            else if (b.shape == ShapeType::Polygon) {
                // 将粒子视为小圆，投影到多边形的各条边，找距离最近的接触线段
                int count = static_cast<int>(b.globalVertices.size());
                float minDistance = std::numeric_limits<float>::max();
                Vec2f closestPoint;
                Vec2f bestNormal;

                for (int i = 0; i < count; ++i) {
                    Vec2f v1 = b.globalVertices[i];
                    Vec2f v2 = b.globalVertices[(i + 1) % count];
                    Vec2f edge = v2 - v1;
                    Vec2f ap = p.position - v1;

                    float edgeLenSq = edge.lengthSquared();
                    if (edgeLenSq == 0.0f) continue;

                    float t = ap.dot(edge) / edgeLenSq;
                    t = std::max(0.0f, std::min(1.0f, t));

                    Vec2f pt = v1 + edge * t;
                    float dist = p.position.distance(pt);

                    if (dist < minDistance) {
                        minDistance = dist;
                        closestPoint = pt;
                        bestNormal = b.globalNormals[i]; // 多边形边法线
                    }
                }

                // 判断粒子中心是否处于多边形内部，如果在内部，强制用法线推出来
                bool inside = Collider::isPointInPolygon(p.position, b);

                if (minDistance < p.radius || inside) {
                    Vec2f normal = bestNormal;
                    if (inside) {
                        // 在内部，穿透距离为多边形边缘到点距离加上粒子半径
                        minDistance = -minDistance; 
                    }
                    float penetration = p.radius - minDistance;

                    // 1. 位置位置校正
                    float invMassSum = 1.0f / p.mass + b.invMass;
                    p.position += normal * penetration * (1.0f / p.mass / invMassSum);
                    if (b.type != BodyType::Static) {
                        b.position -= normal * penetration * (b.invMass / invMassSum);
                        b.updateGlobalVertices();
                    }

                    // 2. 冲量碰撞反射
                    Vec2f relativeVel = p.velocity - b.velocity;
                    float vNormal = relativeVel.dot(normal);

                    if (vNormal < 0.0f) {
                        float elasticity = std::min(config.particleElasticity, b.restitution);
                        float j = -(1.0f + elasticity) * vNormal / invMassSum;

                        p.velocity += normal * j / p.mass;
                        if (b.type != BodyType::Static) {
                            b.velocity -= normal * j * b.invMass;
                        }
                    }
                }
            }
        }
    }
}

void PhysicsWorld::resolveRigidWallCollisions(RigidBody& b) {
    if (b.type == BodyType::Static) return;
    if (m_walls.empty()) return;

    float e = b.restitution;

    if (b.shape == ShapeType::Circle) {
        // 圆形刚体 vs 墙壁线段 (支持深穿透检测)
        for (const auto& wall : m_walls) {
            Vec2f ab = wall.end - wall.start;
            float abLenSq = ab.lengthSquared();
            if (abLenSq == 0.0f) continue;

            Vec2f wallNormal(-ab.y, ab.x);
            float wallLen = wallNormal.length();
            wallNormal = wallNormal / wallLen;

            // 确保法线指向刚体所在方向
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

            // 切向偏离（用于避免把平行于墙壁且很远的对象判定为碰撞）
            float tangentDistSq = std::max(0.0f, dist * dist - signedDist * signedDist);

            // 发生浅穿透 或 深穿透 (无深度限制，只要在切向范围内即可)
            if (signedDist < b.radius && tangentDistSq < b.radius * b.radius) {
                // 默认使用墙壁法线（特别是在深穿透时极其重要，避免法线翻转）
                Vec2f normal = wallNormal;
                float penetration = b.radius - signedDist;

                // 如果是端点的浅穿透，使用真实的圆角法线
                if (signedDist > 0.0f && dist > 0.0f && (t < 0.0f || t > 1.0f)) {
                    normal = toCenter / dist;
                    penetration = b.radius - dist;
                    // 如果真正的端点距离大于半径，说明只是擦过延伸线，并未真正撞到端点
                    if (dist > b.radius) continue; 
                }

                // 位置纠正
                b.position += normal * penetration;

                // 速度反射与摩擦转动
                Vec2f r = closest - b.position;
                Vec2f vRel = b.velocity + Vec2f(-b.angularVelocity * r.y, b.angularVelocity * r.x);
                float vNormal = vRel.dot(normal);

                if (vNormal < 0.0f) {
                    float rCrossN = r.cross(normal);
                    float rotMass = rCrossN * rCrossN * b.invInertia;
                    float j = -(1.0f + e) * vNormal / (b.invMass + rotMass);
                    
                    Vec2f impulse = normal * j;
                    b.applyImpulse(impulse, r);

                    // 摩擦力
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
        // 多边形刚体 vs 墙壁线段：逐顶点深穿透检测
        b.updateGlobalVertices();

        for (const auto& wall : m_walls) {
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

                // 渗透阈值：移除深穿透限制，允许高压下的无限深穿透纠正，只要切向偏离不离谱（<20像素）即可
                float threshold = 2.0f; 
                if (signedDist < threshold && tangentDistSq < 400.0f) {
                    float penetration = threshold - signedDist;
                    totalCorrection += wallNormal * penetration;
                    collisionCount++;
                    if (penetration > maxPenetration) {
                        maxPenetration = penetration;
                        bestContact = v; // 用该穿透最深的顶点作为施加力的接触点
                    }
                }
            }

            if (collisionCount > 0) {
                // 位置纠正
                b.position += totalCorrection / static_cast<float>(collisionCount);
                b.updateGlobalVertices();

                // 速度反射与摩擦转动
                Vec2f r = bestContact - b.position;
                Vec2f vRel = b.velocity + Vec2f(-b.angularVelocity * r.y, b.angularVelocity * r.x);
                float vNormal = vRel.dot(wallNormal);

                if (vNormal < 0.0f) {
                    float rCrossN = r.cross(wallNormal);
                    float rotMass = rCrossN * rCrossN * b.invInertia;
                    float j = -(1.0f + e) * vNormal / (b.invMass + rotMass);
                    
                    b.applyImpulse(wallNormal * j, r);

                    // 摩擦力
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
