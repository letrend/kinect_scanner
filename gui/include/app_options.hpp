#pragma once

#include <QString>

struct AppOptions {
    bool simulate = false;
    bool actuated = false;
    bool benchmark = false;
    QString simStlPath;
    QString simScenario;
    QString simMotionPreset;
    QString simMotionPath;
    QString simReportPath;
    QString controlTcpHost = "127.0.0.1";
    int controlTcpPort = 5056;
    QString actuatorTcpHost = "0.0.0.0";
    int actuatorTcpPort = 5055;
};
