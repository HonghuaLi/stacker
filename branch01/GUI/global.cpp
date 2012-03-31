#include "global.h"

bool NORMALIZE_MESH = true;
bool MOVE_CENTER_TO_ORIGIN = true;

QString DEFAULT_FILE_PATH;

QMeshDoc * mDoc;

QSurfaceMesh * morphObj = NULL;

// Stacker globals
int NUM_EXPECTED_SOLUTION = 10;
double BB_TOLERANCE = 1.1;
double TARGET_STACKABILITY = 0.7;
double HOT_RANGE = 0.95;
