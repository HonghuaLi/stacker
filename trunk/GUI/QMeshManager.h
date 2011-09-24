#pragma once

#include <QMap>
#include <QFileInfo>
#include "QSurfaceMesh.h"

extern QMap<QString, QSurfaceMesh> all_objects;
extern uint global_id;

// Functions
QString addNewObject(QString fileName);
QSurfaceMesh * getObject(QString objectId);
