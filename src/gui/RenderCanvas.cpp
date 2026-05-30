#include "RenderCanvas.hpp"
#include <QColor>
#include <QFont>
#include <QPainter>
#include <QTime>

// 粒子顶点数据结构
struct ParticleVertex {
  float x, y;
  float r, g, b, a;
  float size;
};

// Vertex Shader: 支持传入坐标、颜色、尺寸，应用正交投影
static const char *vsrc = R"(
#version 330 core
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec4 aColor;
layout(location = 2) in float aSize;

out vec4 vColor;

uniform mat4 projection;

void main() {
    gl_Position = projection * vec4(aPos, 0.0, 1.0);
    // 适配不同高分屏可能有稍微缩放问题，这里用基础的 size 放大两倍（因为它是半径）
    gl_PointSize = aSize * 2.0; 
    vColor = aColor;
}
)";

// Fragment Shader: 裁剪点精灵(Point Sprite)形成圆形，支持柔和边缘
static const char *fsrc = R"(
#version 330 core
in vec4 vColor;
out vec4 FragColor;

void main() {
    // 将坐标从 [0,1] 映射到 [-1, 1]
    vec2 coord = gl_PointCoord * 2.0 - vec2(1.0);
    float distSq = dot(coord, coord);
    
    // 如果到中心距离超过 1.0，丢弃像素（变圆）
    if (distSq > 1.0) {
        discard;
    }
    
    // 简单的抗锯齿效果 (柔和边缘)
    float alpha = smoothstep(1.0, 0.8, distSq);
    FragColor = vec4(vColor.rgb, vColor.a * alpha);
}
)";

RenderCanvas::RenderCanvas(PhysicsWorld *world, QWidget *parent)
    : QOpenGLWidget(parent), m_world(world), m_paused(false),
      m_singleStepRequested(false), m_isDrawingWall(false), m_frameCount(0),
      m_fps(60.0f), m_physicsTimeMs(0.0f), m_renderTimeMs(0.0f),
      m_shader(nullptr) {

  setFixedSize(static_cast<int>(m_world->config.worldWidth),
               static_cast<int>(m_world->config.worldHeight));

  setMouseTracking(true);
  m_fpsTimer.start();
}

RenderCanvas::~RenderCanvas() {
  makeCurrent();
  m_vbo.destroy();
  m_vao.destroy();
  if (m_shader) {
    delete m_shader;
  }
  doneCurrent();
}

void RenderCanvas::initializeGL() {
  initializeOpenGLFunctions();

  // 输出 OpenGL 版本信息，用于诊断
  qDebug() << "OpenGL Version:" << reinterpret_cast<const char *>(glGetString(GL_VERSION));
  qDebug() << "GLSL Version:" << reinterpret_cast<const char *>(glGetString(GL_SHADING_LANGUAGE_VERSION));
  qDebug() << "Renderer:" << reinterpret_cast<const char *>(glGetString(GL_RENDERER));

  // 允许开启点大小修改，以及混合
  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  m_shader = new QOpenGLShaderProgram(this);
  if (!m_shader->addShaderFromSourceCode(QOpenGLShader::Vertex, vsrc)) {
    qDebug() << "Vertex Shader ERROR:" << m_shader->log();
  }
  if (!m_shader->addShaderFromSourceCode(QOpenGLShader::Fragment, fsrc)) {
    qDebug() << "Fragment Shader ERROR:" << m_shader->log();
  }
  if (!m_shader->link()) {
    qDebug() << "Shader Link ERROR:" << m_shader->log();
  }

  m_vao.create();
  m_vao.bind();

  m_vbo.create();
  m_vbo.bind();
  m_vbo.setUsagePattern(QOpenGLBuffer::StreamDraw);

  // aPos
  m_shader->enableAttributeArray(0);
  m_shader->setAttributeBuffer(0, GL_FLOAT, offsetof(ParticleVertex, x), 2,
                               sizeof(ParticleVertex));
  // aColor
  m_shader->enableAttributeArray(1);
  m_shader->setAttributeBuffer(1, GL_FLOAT, offsetof(ParticleVertex, r), 4,
                               sizeof(ParticleVertex));
  // aSize
  m_shader->enableAttributeArray(2);
  m_shader->setAttributeBuffer(2, GL_FLOAT, offsetof(ParticleVertex, size), 1,
                               sizeof(ParticleVertex));

  m_vao.release();
}

void RenderCanvas::resizeGL(int w, int h) {
  m_projectionMatrix.setToIdentity();
  m_projectionMatrix.ortho(0.0f, w, h, 0.0f, -1.0f, 1.0f); // 正交投影，Y轴向下
}

void RenderCanvas::updatePhysics() {
  QElapsedTimer localTimer;
  localTimer.start();
  if (!m_paused) {
    m_world->update(m_world->config.timeStep);
  } else if (m_singleStepRequested) {
    m_world->step();
    m_singleStepRequested = false;
  }
  m_physicsTimeMs = static_cast<float>(localTimer.nsecsElapsed()) / 1000000.0f;
}

void RenderCanvas::paintGL() {
  QElapsedTimer localTimer;

  // ==========================================
  // 1. 物理计算步长更新
  // ==========================================
  updatePhysics();

  // ==========================================
  // 2 & 3. 混合渲染管线：QPainter 与 原生 OpenGL
  // ==========================================
  QPainter painter(this);
  
  // 必须在 QPainter 开启原生 OpenGL 混合，防止 QPainter 初始化时清空我们的粒子
  painter.beginNativePainting();

  // 强制恢复被 QPainter 改变的底层 OpenGL 状态
  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  localTimer.start();

  // 深黑色极简高级背景
  glClearColor(25.0f / 255.0f, 27.0f / 255.0f, 34.0f / 255.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  const auto &particles = m_world->getParticles();
  if (!particles.empty()) {
    std::vector<ParticleVertex> vertexData;
    vertexData.reserve(particles.size());

    for (const auto &p : particles) {
      ParticleVertex v;
      v.x = p.position.x;
      v.y = p.position.y;

      if (p.type == ParticleType::Fluid) {
        // 流体粒子视觉半径 = SPH 影响半径的一半，让水流连成一片
        v.size = m_world->config.fluidRadius * 0.5f;
        v.r = 0.0f;
        v.g = 220.0f / 255.0f;
        v.b = 220.0f / 255.0f;
        v.a = 210.0f / 255.0f;
      } else {
        // 普通粒子保持原始半径
        v.size = p.radius;
        v.r = 60.0f / 255.0f;
        v.g = 150.0f / 255.0f;
        v.b = 250.0f / 255.0f;
        v.a = 200.0f / 255.0f;
      }
      vertexData.push_back(v);
    }

    m_shader->bind();
    m_shader->setUniformValue("projection", m_projectionMatrix);

    m_vao.bind();
    m_vbo.bind();
    // 传输数据到显存
    m_vbo.allocate(vertexData.data(), static_cast<int>(vertexData.size() *
                                                       sizeof(ParticleVertex)));

    // 点精灵批量绘制，一次 Draw Call 解决几万个粒子
    glDrawArrays(GL_POINTS, 0, static_cast<int>(vertexData.size()));

    m_vao.release();
    m_shader->release();
  }

  // 结束原生 OpenGL 绘制，交还控制权给 QPainter
  painter.endNativePainting();

  // 继续用 QPainter 绘制覆盖在粒子上方的线段、刚体和 UI
  painter.setRenderHint(QPainter::Antialiasing, true);

  // A. 绘制静态线段墙体
  painter.setPen(QPen(QColor(180, 185, 200), 4, Qt::SolidLine, Qt::RoundCap));
  for (const auto &wall : m_world->getWalls()) {
    painter.drawLine(QPointF(wall.start.x, wall.start.y),
                     QPointF(wall.end.x, wall.end.y));
  }

  // B. 绘制喷射发射器 (Emitters)
  for (const auto &e : m_world->getEmitters()) {
    painter.setPen(QPen(QColor(255, 140, 50), 2));
    painter.setBrush(QColor(255, 140, 50, 60));
    painter.drawEllipse(QPointF(e.position.x, e.position.y), 8.0, 8.0);

    Vec2f dirEnd =
        e.position + Vec2f(std::cos(e.angle), std::sin(e.angle)) * 22.0f;
    painter.setPen(QPen(QColor(255, 140, 50), 2, Qt::DashLine));
    painter.drawLine(QPointF(e.position.x, e.position.y),
                     QPointF(dirEnd.x, dirEnd.y));
  }

  // C. 绘制刚体
  for (const auto &b : m_world->getRigidBodies()) {
    if (b.type == BodyType::Static) {
      painter.setPen(QPen(QColor(100, 130, 150), 2));
      painter.setBrush(QColor(43, 52, 65, 200));
    } else {
      painter.setPen(QPen(QColor(120, 200, 255), 2));
      painter.setBrush(QColor(46, 88, 148, 180));
    }

    if (b.shape == ShapeType::Circle) {
      painter.drawEllipse(QPointF(b.position.x, b.position.y), b.radius,
                          b.radius);

      Vec2f lineEnd =
          b.position + Vec2f(std::cos(b.angle), std::sin(b.angle)) * b.radius;
      painter.setPen(QPen(QColor(255, 255, 255, 180), 1.5f));
      painter.drawLine(QPointF(b.position.x, b.position.y),
                       QPointF(lineEnd.x, lineEnd.y));
    } else if (b.shape == ShapeType::Polygon) {
      QPolygonF qPoly;
      for (const auto &v : b.globalVertices) {
        qPoly << QPointF(v.x, v.y);
      }
      painter.drawPolygon(qPoly);
    }
  }

  // D. 绘制鼠标拖拽画墙的实时虚线预览
  if (m_isDrawingWall) {
    painter.setPen(
        QPen(QColor(80, 220, 100, 180), 2, Qt::DashLine, Qt::RoundCap));
    painter.drawLine(QPointF(m_drawStart.x, m_drawStart.y),
                     QPointF(m_drawCurrent.x, m_drawCurrent.y));
    painter.setBrush(QColor(80, 220, 100, 100));
    painter.drawEllipse(QPointF(m_drawStart.x, m_drawStart.y), 4.0, 4.0);
  }

  // E. 绘制性能状态面板
  drawPerformanceOverlay(painter);

  // 结束 QPainter，保证 OpenGL 状态正确恢复
  painter.end();

  m_renderTimeMs = static_cast<float>(localTimer.nsecsElapsed()) / 1000000.0f;

  // ==========================================
  // 4. FPS 帧率计数统计
  // ==========================================
  m_frameCount++;
  if (m_fpsTimer.elapsed() >= 1000) {
    m_fps = static_cast<float>(m_frameCount * 1000.0) /
            static_cast<float>(m_fpsTimer.restart());
    m_frameCount = 0;
  }
}

void RenderCanvas::drawPerformanceOverlay(QPainter &painter) {
  int overlayW = 220;
  int overlayH = 110;
  int overlayX = width() - overlayW - 15;
  int overlayY = 15;

  // 绘制半透明黑色玻璃板底座
  painter.setPen(QPen(QColor(255, 255, 255, 30), 1));
  painter.setBrush(QColor(10, 12, 18, 200));
  painter.drawRoundedRect(QRect(overlayX, overlayY, overlayW, overlayH), 8.0,
                          8.0);

  // 文字参数配置
  painter.setPen(QColor(240, 240, 245));
  QFont font = painter.font();
  font.setPointSize(9);
  font.setFamily("Consolas");
  painter.setFont(font);

  int startTextY = overlayY + 20;
  int lineSpacing = 18;

  painter.drawText(overlayX + 15, startTextY,
                   QString("FPS:          %1").arg(m_fps, 0, 'f', 1));
  painter.drawText(
      overlayX + 15, startTextY + lineSpacing,
      QString("Particles:    %1").arg(m_world->getParticles().size()));
  painter.drawText(
      overlayX + 15, startTextY + lineSpacing * 2,
      QString("Physics Time: %1 ms").arg(m_physicsTimeMs, 0, 'f', 2));
  painter.drawText(
      overlayX + 15, startTextY + lineSpacing * 3,
      QString("Render Time:  %1 ms").arg(m_renderTimeMs, 0, 'f', 2));

  // 单击运行/暂停标志指示灯
  painter.setPen(Qt::NoPen);
  if (m_paused) {
    painter.setBrush(QColor(240, 60, 60)); // 红色圆点表示暂停
  } else {
    painter.setBrush(QColor(60, 220, 80)); // 绿色圆点表示运行中
  }
  painter.drawEllipse(QPointF(overlayX + overlayW - 20, overlayY + 20), 5.0,
                      5.0);
}

void RenderCanvas::mousePressEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    m_isDrawingWall = true;
    m_drawStart = Vec2f(static_cast<float>(event->position().x()),
                        static_cast<float>(event->position().y()));
    m_drawCurrent = m_drawStart;
    update();
  }
}

void RenderCanvas::mouseMoveEvent(QMouseEvent *event) {
  m_drawCurrent = Vec2f(static_cast<float>(event->position().x()),
                        static_cast<float>(event->position().y()));

  emit mousePositionChanged(QString("X: %1, Y: %2")
                                .arg(static_cast<int>(m_drawCurrent.x))
                                .arg(static_cast<int>(m_drawCurrent.y)));

  if (m_isDrawingWall) {
    update();
  }
}

void RenderCanvas::mouseReleaseEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton && m_isDrawingWall) {
    m_isDrawingWall = false;

    if (m_drawStart.distance(m_drawCurrent) > 5.0f) {
      m_world->addWall(m_drawStart, m_drawCurrent);
    }
    update();
  }
}
