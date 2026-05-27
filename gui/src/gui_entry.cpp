#include "gui_entry.hpp"
#include "main_window.hpp"

#include <QApplication>
#include <QSurfaceFormat>

int runGui(int argc, char *argv[]) {
    // Request a core 3.3 context for the QOpenGLWidget viewer.
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setDepthBufferSize(24);
    fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);
    MainWindow w;
    w.show();
    return app.exec();
}
