#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QMouseEvent>
#include <QElapsedTimer>
#include "../core/World.hpp"

class RenderCanvas : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    RenderCanvas(PhysicsWorld* world, QWidget* parent = nullptr);
    ~RenderCanvas();

    void setPaused(bool paused) { m_paused = paused; }
    bool isPaused() const { return m_paused; }

    void triggerSingleStep() { m_singleStepRequested = true; }

    // 性能数据接口
    float getFPS() const { return m_fps; }
    float getPhysicsTimeMs() const { return m_physicsTimeMs; }
    float getRenderTimeMs() const { return m_renderTimeMs; }

signals:
    void mousePositionChanged(const QString& posStr);

protected:
    // QOpenGLWidget 核心生命周期函数
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    // 鼠标交互事件
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    PhysicsWorld* m_world;
    bool m_paused;
    bool m_singleStepRequested;

    // 鼠标画墙交互变量
    bool m_isDrawingWall;
    Vec2f m_drawStart;
    Vec2f m_drawCurrent;

    // 性能监测与 FPS 统计
    QElapsedTimer m_fpsTimer;
    int m_frameCount;
    float m_fps;
    float m_physicsTimeMs;
    float m_renderTimeMs;

    // OpenGL 核心对象
    QOpenGLShaderProgram* m_shader;
    QOpenGLBuffer m_vbo;
    QOpenGLVertexArrayObject m_vao;

    // 缓存矩阵
    QMatrix4x4 m_projectionMatrix;

    void drawPerformanceOverlay(QPainter& painter);
    void updatePhysics();
};
