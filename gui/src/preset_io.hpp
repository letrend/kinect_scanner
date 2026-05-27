#pragma once

#include <QString>
#include "scan_parameters.hpp"

/// JSON preset persistence for ScanParameters.
namespace preset_io {
    bool saveJson(const QString &path, const ScanParameters &p, QString *err = nullptr);
    bool loadJson(const QString &path, ScanParameters &p, QString *err = nullptr);
}
