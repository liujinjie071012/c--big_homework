#include "RenderCanvas.hpp"
#include <QPainter>
#include <QTime>
#include <QColor>
#include <QFont>
#include <cmath>

RenderCanvas::RenderCanvas(PhysicsWorld* world, QWidget* parent)
    : QWidget(parent), m_world(world), m_paused(false), 
      m_singleStepRequested(false), m_isDrawingWall(false),
      m_frameCount(0), m_fps(60.0f), m_physicsTimeMs(0.0f), m_renderTimeMs(0.0f) {
    
    // 启用双缓冲防闪烁
    setAttribute(Qt::WA_OpaquePaintEvent);
    setFixedSize(static_cast<int>(m_world->config.worldWidth), 
                 static_cast<int>(m_world->config.worldHeight));
    
    setMouseTracking(true);
    m_fpsTimer.start();
}

void RenderCanvas::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);

    QElapsedTimer localTimer;
    
    // ==========================================
    // 1. 物理计算步长更新
    // ==========================================
    localTimer.start();
    if (!m_paused) {
        m_world->update(m_world->config.timeStep);
    } else if (m_singleStepRequested) {
        m_world->step();
        m_singleStepRequested = false;
    }
    m_physicsTimeMs = static_cast<float>(localTimer.nsecsElapsed()) / 1000000.0f; // 换算为毫秒

    // ==========================================
    // 2. QPainter 界面绘制
    // ==========================================
    localTimer.start();
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    // 深黑色极简高级背景
    painter.fillRect(rect(), QColor(25, 27, 34));

    // A. 绘制静态线段墙体
    painter.setPen(QPen(QColor(180, 185, 200), 4, Qt::SolidLine, Qt::RoundCap));
    for (const auto& wall : m_world->getWalls()) {
        painter.drawLine(QPointF(wall.start.x, wall.start.y), 
                         QPointF(wall.end.x, wall.end.y));
    }

    // B. 绘制喷射发射器 (Emitters)
    for (const auto& e : m_world->getEmitters()) {
        painter.setPen(QPen(QColor(255, 140, 50), 2));
        painter.setBrush(QColor(255, 140, 50, 60));
        painter.drawEllipse(QPointF(e.position.x, e.position.y), 8.0, 8.0);
        
        // 绘制发射朝向线
        Vec2f dirEnd = e.position + Vec2f(std::cos(e.angle), std::sin(e.angle)) * 22.0f;
        painter.setPen(QPen(QColor(255, 140, 50), 2, Qt::DashLine));
        painter.drawLine(QPointF(e.position.x, e.position.y), 
                         QPointF(dirEnd.x, dirEnd.y));
    }

    // C. 绘制刚体列表 (圆形/多边形刚体物理绘制)
    for (const auto& b : m_world->getRigidBodies()) {
        if (b.type == BodyType::Static) {
            // 静态刚体使用稳重的暗青灰色
            painter.setPen(QPen(QColor(100, 130, 150), 2));
            painter.setBrush(QColor(43, 52, 65, 200));
        } else {
            // 动态刚体使用亮丽的深蓝色/青色
            painter.setPen(QPen(QColor(120, 200, 255), 2));
            painter.setBrush(QColor(46, 88, 148, 180));
        }

        if (b.shape == ShapeType::Circle) {
            painter.drawEllipse(QPointF(b.position.x, b.position.y), b.radius, b.radius);
            
            // 绘制一条从中心到圆周的旋转方位线，表现角速度旋转状态
            Vec2f lineEnd = b.position + Vec2f(std::cos(b.angle), std::sin(b.angle)) * b.radius;
            painter.setPen(QPen(QColor(255, 255, 255, 180), 1.5f));
            painter.drawLine(QPointF(b.position.x, b.position.y), QPointF(lineEnd.x, lineEnd.y));
        }
        else if (b.shape == ShapeType::Polygon) {
            QPolygonF qPoly;
            for (const auto& v : b.globalVertices) {
                qPoly << QPointF(v.x, v.y);
            }
            painter.drawPolygon(qPoly);
        }
    }

    // D. 绘制粒子列表 (高性能批量渲染优化)
    //    关闭粒子的抗锯齿：半径3px的圆，抗锯齿几乎无视觉差异，但开销巨大
    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.setPen(Qt::NoPen);

    // 按类型分批绘制，最大限度减少 setBrush 状态切换次数
    // (QPainter 每次 setBrush 都会触发内部状态重建，是最大的性能杀手)
    const auto& particles = m_world->getParticles();
    
    // 第一批：普通粒子
    painter.setBrush(QColor(60, 150, 250, 200));
    for (const auto& p : particles) {
        if (p.type != ParticleType::Normal) continue;
        if (p.radius <= 2.5f) {
            // 小粒子用矩形替代椭圆，绘制速度提升约3倍
            painter.drawRect(QRectF(p.position.x - p.radius, p.position.y - p.radius,
                                    p.radius * 2.0f, p.radius * 2.0f));
        } else {
            painter.drawEllipse(QPointF(p.position.x, p.position.y), p.radius, p.radius);
        }
    }

    // 第二批：流体粒子
    painter.setBrush(QColor(0, 220, 220, 210));
    for (const auto& p : particles) {
        if (p.type != ParticleType::Fluid) continue;
        if (p.radius <= 2.5f) {
            painter.drawRect(QRectF(p.position.x - p.radius, p.position.y - p.radius,
                                    p.radius * 2.0f, p.radius * 2.0f));
        } else {
            painter.drawEllipse(QPointF(p.position.x, p.position.y), p.radius, p.radius);
        }
    }

    // 恢复抗锯齿给后续UI元素使用
    painter.setRenderHint(QPainter::Antialiasing, true);

    // D. 绘制鼠标拖拽画墙的实时虚线预览
    if (m_isDrawingWall) {
        painter.setPen(QPen(QColor(80, 220, 100, 180), 2, Qt::DashLine, Qt::RoundCap));
        painter.drawLine(QPointF(m_drawStart.x, m_drawStart.y), 
                         QPointF(m_drawCurrent.x, m_drawCurrent.y));
        painter.setBrush(QColor(80, 220, 100, 100));
        painter.drawEllipse(QPointF(m_drawStart.x, m_drawStart.y), 4.0, 4.0);
    }

    // E. 绘制性能状态面板
    drawPerformanceOverlay(painter);

    m_renderTimeMs = static_cast<float>(localTimer.nsecsElapsed()) / 1000000.0f;

    // ==========================================
    // 3. FPS 帧率计数统计
    // ==========================================
    m_frameCount++;
    if (m_fpsTimer.elapsed() >= 1000) {
        m_fps = static_cast<float>(m_frameCount * 1000.0) / static_cast<float>(m_fpsTimer.restart());
        m_frameCount = 0;
    }
}

void RenderCanvas::drawPerformanceOverlay(QPainter& painter) {
    int overlayW = 220;
    int overlayH = 110;
    int overlayX = width() - overlayW - 15;
    int overlayY = 15;

    // 绘制半透明黑色玻璃板底座
    painter.setPen(QPen(QColor(255, 255, 255, 30), 1));
    painter.setBrush(QColor(10, 12, 18, 200));
    painter.drawRoundedRect(QRect(overlayX, overlayY, overlayW, overlayH), 8.0, 8.0);

    // 文字参数配置
    painter.setPen(QColor(240, 240, 245));
    QFont font = painter.font();
    font.setPointSize(9);
    font.setFamily("Consolas");
    painter.setFont(font);

    int startTextY = overlayY + 20;
    int lineSpacing = 18;

    painter.drawText(overlayX + 15, startTextY, QString("FPS:          %1").arg(m_fps, 0, 'f', 1));
    painter.drawText(overlayX + 15, startTextY + lineSpacing, 
                     QString("Particles:    %1").arg(m_world->getParticles().size()));
    painter.drawText(overlayX + 15, startTextY + lineSpacing * 2, 
                     QString("Physics Time: %1 ms").arg(m_physicsTimeMs, 0, 'f', 2));
    painter.drawText(overlayX + 15, startTextY + lineSpacing * 3, 
                     QString("Render Time:  %1 ms").arg(m_renderTimeMs, 0, 'f', 2));
    
    // 单击运行/暂停标志指示灯
    painter.setPen(Qt::NoPen);
    if (m_paused) {
        painter.setBrush(QColor(240, 60, 60)); // 红色圆点表示暂停
    } else {
        painter.setBrush(QColor(60, 220, 80)); // 绿色圆点表示运行中
    }
    painter.drawEllipse(QPointF(overlayX + overlayW - 20, overlayY + 20), 5.0, 5.0);
}

void RenderCanvas::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        m_isDrawingWall = true;
        m_drawStart = Vec2f(static_cast<float>(event->position().x()), 
                            static_cast<float>(event->position().y()));
        m_drawCurrent = m_drawStart;
        update();
    }
}

void RenderCanvas::mouseMoveEvent(QMouseEvent* event) {
    m_drawCurrent = Vec2f(static_cast<float>(event->position().x()), 
                          static_cast<float>(event->position().y()));
    
    // 触发鼠标状态坐标更新信号
    emit mousePositionChanged(QString("X: %1, Y: %2")
                              .arg(static_cast<int>(m_drawCurrent.x))
                              .arg(static_cast<int>(m_drawCurrent.y)));

    if (m_isDrawingWall) {
        update();
    }
}

void RenderCanvas::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton && m_isDrawingWall) {
        m_isDrawingWall = false;
        
        // 只有线段长度大于 5 像素才判定为有效墙体，防止鼠标误触误点
        if (m_drawStart.distance(m_drawCurrent) > 5.0f) {
            m_world->addWall(m_drawStart, m_drawCurrent);
        }
        update();
    }
}
