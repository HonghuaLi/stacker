#pragma once

#include <QMap>
#include <QFileInfo>
#include "QSurfaceMesh.h"

extern QMap<QString, QSurfaceMesh> all_objects;

// Functions
void addNewObject(QString fileName);
QSurfaceMesh * getObject(QString objectId);
