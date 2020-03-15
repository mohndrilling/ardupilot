#pragma once

#include "vector3.h"

// return angles for 5th order polynomial trajectory with duration T at time t
void polynomial_trajectory(Vector3f &current, Vector3f start, Vector3f target, float T, float t);


