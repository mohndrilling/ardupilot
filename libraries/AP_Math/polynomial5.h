#pragma once

#include "vector3.h"

// calculate function value for 5th order polynomial trajectory with duration T at time t
void polynomial_trajectory(float &current, float start, float target, float T, float t);

// 5th order polynomial trajectory for 3D Vectors
void polynomial_trajectory_3d(Vector3f &current, Vector3f start, Vector3f target, float T, float t);


