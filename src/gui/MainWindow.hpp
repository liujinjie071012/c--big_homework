#pragma once

#include <QMainWindow>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QTimer>
#include <QGroupBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <memory>
#include "../core/World.hpp"
#include "RenderCanvas.hpp"

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow() = default;

private slots:
    void onPlayPauseClicked();
    void onStepClicked();
    void onResetClicked();
    void onClearAllClicked();
    void onGravitySliderChanged(int value);
    void onTimeStepSliderChanged(int value);
    void onAddEmitterClicked();
    void onAddNormalEmitterClicked();
    void onAddCircleClicked();
    void onAddBoxClicked();
    void updateStatusBar(const QString& posStr);
    void triggerRepaint(); // 驱动 Qt 刷新重绘定时器回调

private:
    std::unique_ptr<PhysicsWorld> m_world;
    RenderCanvas* m_canvas;
    QTimer* m_repaintTimer;  // 重绘驱动定时器

    // UI 控件
    QPushButton* m_btnPlayPause;
    QPushButton* m_btnStep;
    QPushButton* m_btnReset;
    QPushButton* m_btnClearAll;
    QPushButton* m_btnAddEmitter;
    QPushButton* m_btnAddNormalEmitter;
    QPushButton* m_btnAddCircle;
    QPushButton* m_btnAddBox;
    
    QSlider* m_sliderGravity;
    QLabel* m_lblGravityVal;

    QSlider* m_sliderTimeStep;
    QLabel* m_lblTimeStepVal;

    QLabel* m_lblCoords;

    void setupUI();
    void createConnections();
    void initializeDefaultScene();
};
