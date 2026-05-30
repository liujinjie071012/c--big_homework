#include "MainWindow.hpp"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QStatusBar>
#include <QApplication>
#include <QStyle>
#include <random>

static float randomFloat(float min, float max) {
    static std::mt19937 gen(1337);
    std::uniform_real_distribution<float> dis(min, max);
    return dis(gen);
}

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent) {
    
    m_world = std::make_unique<PhysicsWorld>();
    
    // 初始化 UI 构建
    setupUI();
    
    // 信号槽连接
    createConnections();

    // 初始化默认场景
    initializeDefaultScene();

    // 同步初始的暂停 UI 状态
    onPlayPauseClicked();

    // 启动重绘定时器 (60 FPS)，驱动画布每 16 毫秒（大约 60 帧）自动重绘
    m_repaintTimer = new QTimer(this);
    connect(m_repaintTimer, &QTimer::timeout, this, &MainWindow::triggerRepaint);
    m_repaintTimer->start(16);
}

void MainWindow::setupUI() {
    setWindowTitle("2D 物理沙盒编辑器 - Qt6");
    
    // 设置深色极客风主题 QSS 样式表
    setStyleSheet(R"(
        QMainWindow {
            background-color: #1a1b22;
        }
        QWidget {
            background-color: #1a1b22;
            color: #d8dee9;
            font-family: 'Segoe UI', Arial, sans-serif;
        }
        QGroupBox {
            border: 1px solid #3b4252;
            border-radius: 6px;
            margin-top: 12px;
            padding-top: 14px;
            font-weight: bold;
            color: #88c0d0;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            subcontrol-position: top left;
            left: 10px;
            padding: 0 5px;
        }
        QPushButton {
            background-color: #2e3440;
            border: 1px solid #4c566a;
            border-radius: 4px;
            padding: 6px 12px;
            font-weight: bold;
            color: #eceff4;
        }
        QPushButton:hover {
            background-color: #3b4252;
            border-color: #88c0d0;
        }
        QPushButton:pressed {
            background-color: #434c5e;
        }
        QPushButton:checked {
            background-color: #4c566a;
            border-color: #a3be8c;
        }
        QSlider::groove:horizontal {
            border: 1px solid #434c5e;
            height: 6px;
            background: #2e3440;
            border-radius: 3px;
        }
        QSlider::handle:horizontal {
            background: #88c0d0;
            width: 14px;
            margin: -4px 0;
            border-radius: 7px;
        }
        QSlider::handle:horizontal:hover {
            background: #8fbcbb;
        }
        QLabel {
            font-weight: 500;
        }
    )");

    // 创建主窗口布局
    QWidget* centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    QHBoxLayout* mainLayout = new QHBoxLayout(centralWidget);
    mainLayout->setContentsMargins(10, 10, 10, 10);
    mainLayout->setSpacing(12);

    // 1. 左侧物理画布
    m_canvas = new RenderCanvas(m_world.get(), this);
    mainLayout->addWidget(m_canvas);

    // 2. 右侧控制面板
    QVBoxLayout* panelLayout = new QVBoxLayout();
    panelLayout->setSpacing(10);

    // Group 1: 仿真状态控制
    QGroupBox* grpState = new QGroupBox("仿真状态控制", this);
    QVBoxLayout* stateLayout = new QVBoxLayout(grpState);
    stateLayout->setSpacing(8);

    m_btnPlayPause = new QPushButton("暂停 ⏸", this);
    m_btnPlayPause->setCheckable(true);
    m_btnPlayPause->setChecked(true); // 默认初始处于暂停状态
    m_btnStep = new QPushButton("单步微调 ⏭", this);
    m_btnReset = new QPushButton("清空粒子 🔄", this);
    m_btnClearAll = new QPushButton("重置整个沙盒 💣", this);

    stateLayout->addWidget(m_btnPlayPause);
    stateLayout->addWidget(m_btnStep);
    stateLayout->addWidget(m_btnReset);
    stateLayout->addWidget(m_btnClearAll);

    // Group 2: 全局物理参数调节
    QGroupBox* grpParams = new QGroupBox("全局物理参数", this);
    QFormLayout* paramsLayout = new QFormLayout(grpParams);
    paramsLayout->setSpacing(10);

    // 重力滑动条 (0 - 1500)
    m_sliderGravity = new QSlider(Qt::Horizontal, this);
    m_sliderGravity->setRange(0, 1500);
    m_sliderGravity->setValue(static_cast<int>(m_world->config.gravity.y));
    m_lblGravityVal = new QLabel(QString::number(m_world->config.gravity.y), this);
    paramsLayout->addRow("垂直重力:", m_sliderGravity);
    paramsLayout->addRow("", m_lblGravityVal);

    // 步长滑动条 (5ms - 40ms)
    m_sliderTimeStep = new QSlider(Qt::Horizontal, this);
    m_sliderTimeStep->setRange(5, 40);
    m_sliderTimeStep->setValue(static_cast<int>(m_world->config.timeStep * 1000.0f));
    m_lblTimeStepVal = new QLabel(QString("%1 ms").arg(m_world->config.timeStep * 1000.0f, 0, 'f', 1), this);
    paramsLayout->addRow("物理步长:", m_sliderTimeStep);
    paramsLayout->addRow("", m_lblTimeStepVal);

    // Group 3: 快速编辑区
    QGroupBox* grpEdit = new QGroupBox("沙盒编辑器", this);
    QVBoxLayout* editLayout = new QVBoxLayout(grpEdit);
    
    m_btnAddEmitter = new QPushButton("放置水射流发射器 💧", this);
    m_btnAddNormalEmitter = new QPushButton("放置普通粒子发射器 ☄️", this);
    m_btnAddCircle = new QPushButton("添加圆形刚体 🔴", this);
    m_btnAddBox = new QPushButton("添加方形箱体 📦", this);
    editLayout->addWidget(m_btnAddEmitter);
    editLayout->addWidget(m_btnAddNormalEmitter);
    editLayout->addWidget(m_btnAddCircle);
    editLayout->addWidget(m_btnAddBox);

    QLabel* lblGuide = new QLabel(
        "💡 <b>画图指南</b>:<br>"
        "1. 在画布上<b>左键拖拽</b>以画出阻挡线段墙体。<br>"
        "2. 刚体及流体力学参数将在后续阶段完整加入。<br>"
        "3. 粒子达到 8000 阈值后将自动循环剔除。", this);
    lblGuide->setWordWrap(true);
    lblGuide->setStyleSheet("color: #a3be8c; font-size: 11px; line-height: 1.4;");
    editLayout->addWidget(lblGuide);

    // 组装右侧面板
    panelLayout->addWidget(grpState);
    panelLayout->addWidget(grpParams);
    panelLayout->addWidget(grpEdit);
    panelLayout->addStretch(); // 弹簧置底

    mainLayout->addLayout(panelLayout);

    // 3. 状态栏坐标显示
    QStatusBar* statBar = statusBar();
    statBar->setStyleSheet("background-color: #2e3440; color: #d8dee9;");
    m_lblCoords = new QLabel("X: 0, Y: 0", this);
    statBar->addPermanentWidget(m_lblCoords);
}

void MainWindow::createConnections() {
    // 仿真控制
    connect(m_btnPlayPause, &QPushButton::clicked, this, &MainWindow::onPlayPauseClicked);
    connect(m_btnStep, &QPushButton::clicked, this, &MainWindow::onStepClicked);
    connect(m_btnReset, &QPushButton::clicked, this, &MainWindow::onResetClicked);
    connect(m_btnClearAll, &QPushButton::clicked, this, &MainWindow::onClearAllClicked);

    // 滑动条参数变更
    connect(m_sliderGravity, &QSlider::valueChanged, this, &MainWindow::onGravitySliderChanged);
    connect(m_sliderTimeStep, &QSlider::valueChanged, this, &MainWindow::onTimeStepSliderChanged);

    // 发射器与刚体创建
    connect(m_btnAddEmitter, &QPushButton::clicked, this, &MainWindow::onAddEmitterClicked);
    connect(m_btnAddNormalEmitter, &QPushButton::clicked, this, &MainWindow::onAddNormalEmitterClicked);
    connect(m_btnAddCircle, &QPushButton::clicked, this, &MainWindow::onAddCircleClicked);
    connect(m_btnAddBox, &QPushButton::clicked, this, &MainWindow::onAddBoxClicked);

    // 鼠标坐标变化联动
    connect(m_canvas, &RenderCanvas::mousePositionChanged, this, &MainWindow::updateStatusBar);
}

void MainWindow::initializeDefaultScene() {
    // 1. 添加默认斜面墙体
    m_world->addWall(Vec2f(100.0f, 220.0f), Vec2f(420.0f, 320.0f));
    m_world->addWall(Vec2f(380.0f, 450.0f), Vec2f(720.0f, 350.0f));
    m_world->addWall(Vec2f(80.0f, 520.0f), Vec2f(300.0f, 550.0f));

    // 2. 添加默认水射流发射器 (喷口朝右下方倾斜)
    m_world->addEmitter(Emitter(Vec2f(120.0f, 100.0f), 0.5f, 0.25f, 280.0f, 150.0f, 3.5f, EmitterType::SPHFluid));

    // 3. 添加刚体：一个中央静态挡板 + 两两个高空落下的动态物理刚体 (圆形和多边形木箱)
    m_world->addRigidBody(RigidBody::createBox(Vec2f(520.0f, 250.0f), 180.0f, 24.0f, BodyType::Static));
    m_world->addRigidBody(RigidBody::createCircle(Vec2f(480.0f, 80.0f), 22.0f, BodyType::Dynamic));
    m_world->addRigidBody(RigidBody::createBox(Vec2f(560.0f, 70.0f), 42.0f, 42.0f, BodyType::Dynamic));
}

void MainWindow::triggerRepaint() {
    m_canvas->update();
}

void MainWindow::onPlayPauseClicked() {
    bool isPaused = m_btnPlayPause->isChecked();
    m_canvas->setPaused(isPaused);
    if (isPaused) {
        m_btnPlayPause->setText("继续运行 ▶");
    } else {
        m_btnPlayPause->setText("暂停 ⏸");
    }
}

void MainWindow::onStepClicked() {
    m_canvas->triggerSingleStep();
}

void MainWindow::onResetClicked() {
    m_world->reset();
    m_canvas->update();
}

void MainWindow::onClearAllClicked() {
    m_world->clear();
    initializeDefaultScene();
    m_canvas->update();
}

void MainWindow::onGravitySliderChanged(int value) {
    m_world->config.gravity.y = static_cast<float>(value);
    m_lblGravityVal->setText(QString::number(value));
}

void MainWindow::onTimeStepSliderChanged(int value) {
    m_world->config.timeStep = static_cast<float>(value) / 1000.0f;
    m_lblTimeStepVal->setText(QString("%1 ms").arg(value));
}

void MainWindow::onAddEmitterClicked() {
    // 随机一个位置和角度喷射
    float rx = randomFloat(100.0f, 700.0f);
    float ry = randomFloat(50.0f, 150.0f);
    float rAngle = randomFloat(0.0f, 6.28f);
    m_world->addEmitter(Emitter(Vec2f(rx, ry), rAngle, 0.3f, 250.0f, 80.0f, 3.0f, EmitterType::SPHFluid));
    m_canvas->update();
}

void MainWindow::onAddNormalEmitterClicked() {
    // 随机一个位置和角度喷射
    float rx = randomFloat(100.0f, 700.0f);
    float ry = randomFloat(50.0f, 150.0f);
    float rAngle = randomFloat(0.0f, 6.28f);
    m_world->addEmitter(Emitter(Vec2f(rx, ry), rAngle, 0.3f, 250.0f, 80.0f, 3.0f, EmitterType::NormalParticle));
    m_canvas->update();
}

void MainWindow::onAddCircleClicked() {
    float rx = randomFloat(200.0f, 600.0f);
    float ry = randomFloat(50.0f, 150.0f);
    float radius = randomFloat(15.0f, 30.0f);
    m_world->addRigidBody(RigidBody::createCircle(Vec2f(rx, ry), radius, BodyType::Dynamic));
    m_canvas->update();
}

void MainWindow::onAddBoxClicked() {
    float rx = randomFloat(200.0f, 600.0f);
    float ry = randomFloat(50.0f, 150.0f);
    float rw = randomFloat(30.0f, 60.0f);
    float rh = randomFloat(30.0f, 60.0f);
    m_world->addRigidBody(RigidBody::createBox(Vec2f(rx, ry), rw, rh, BodyType::Dynamic));
    m_canvas->update();
}

void MainWindow::updateStatusBar(const QString& posStr) {
    m_lblCoords->setText(posStr);
}
