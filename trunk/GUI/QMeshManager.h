#pragma once

#include <QMap>
#include <QFileInfo>
#include "QSegMesh.h"

extern QMap<QString, QSegMesh> all_objects;
extern uint global_id;

// Functions
QString addNewObject(QString fileName);
QSegMesh * getObject( QString objectId );