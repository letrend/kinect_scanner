#include "gui_entry.hpp"
#include "app_options.hpp"
#include "main_window.hpp"

#include <QApplication>
#include <QCommandLineParser>
#include <QScreen>
#include <QSurfaceFormat>

namespace {

bool parseHostPort(const QString &text, QString &host, int &port) {
    QString value = text.trimmed();
    if (value.isEmpty()) return false;
    int colon = value.lastIndexOf(':');
    bool ok = false;
    if (colon > 0) {
        int p = value.mid(colon + 1).toInt(&ok);
        if (!ok || p <= 0 || p > 65535) return false;
        host = value.left(colon);
        port = p;
        return true;
    }
    int p = value.toInt(&ok);
    if (!ok || p <= 0 || p > 65535) return false;
    port = p;
    return true;
}

} // namespace

int runGui(int argc, char *argv[]) {
    // Request a core 3.3 context for the QOpenGLWidget viewer.
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setDepthBufferSize(24);
    fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);

    QCommandLineParser parser;
    parser.setApplicationDescription("Kinect 3D Scanner");
    parser.addHelpOption();
    QCommandLineOption simulateOpt("simulate", "Run with the simulated frame source.");
    QCommandLineOption actuatedOpt("actuated", "Run with actuator TCP pose source.");
    QCommandLineOption simStlOpt("sim-stl", "STL file used by simulation.", "path");
    QCommandLineOption controlOpt("control-tcp",
                                  "Enable UI-control TCP server on [host:]port.",
                                  "endpoint", "127.0.0.1:5056");
    QCommandLineOption actuatorOpt("actuator-tcp",
                                   "Set actuator TCP server [host:]port.",
                                   "endpoint", "0.0.0.0:5055");
    parser.addOption(simulateOpt);
    parser.addOption(actuatedOpt);
    parser.addOption(simStlOpt);
    parser.addOption(controlOpt);
    parser.addOption(actuatorOpt);
    parser.process(app);

    AppOptions options;
    options.simulate = parser.isSet(simulateOpt);
    options.actuated = parser.isSet(actuatedOpt);
    options.simStlPath = parser.value(simStlOpt);
    parseHostPort(parser.value(controlOpt), options.controlTcpHost, options.controlTcpPort);
    parseHostPort(parser.value(actuatorOpt), options.actuatorTcpHost, options.actuatorTcpPort);

    MainWindow w(options);
    if (QScreen *screen = app.primaryScreen()) {
        const QRect available = screen->availableGeometry();
        QSize size(qMax(900, int(available.width() * 0.95)),
                   qMax(650, int(available.height() * 0.95)));
        size.setWidth(qMin(size.width(), available.width()));
        size.setHeight(qMin(size.height(), available.height()));
        w.resize(size);
        w.move(available.center() - QPoint(size.width() / 2, size.height() / 2));
    }
    w.show();
    return app.exec();
}
