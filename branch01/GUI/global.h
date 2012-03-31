#pragma once

extern bool NORMALIZE_MESH;
extern bool MOVE_CENTER_TO_ORIGIN;

#include <QString>
extern QString DEFAULT_FILE_PATH;

#include "GUI/QMeshDoc.h"
extern QMeshDoc * mDoc;

extern QSurfaceMesh * morphObj;

// Stacker globals
extern int NUM_EXPECTED_SOLUTION;
extern double BB_TOLERANCE;
extern double TARGET_STACKABILITY;
extern double HOT_RANGE;
