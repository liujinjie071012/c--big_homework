#include <QApplication>
#include <QSurfaceFormat>
#include "src/gui/MainWindow.hpp"

int main(int argc, char *argv[]) {
    // 强制使用桌面 OpenGL (而不是 ANGLE/DirectX 翻译层)，
    // 否则 ANGLE 不支持 #version 330 core 着色器
    QApplication::setAttribute(Qt::AA_UseDesktopOpenGL);

    // 设置全局 OpenGL 版本为 3.3 Core Profile
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);
    
    MainWindow window;
    window.show();
    
    return app.exec();
}