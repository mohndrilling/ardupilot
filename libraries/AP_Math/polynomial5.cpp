#include "AP_Math.h"

void polynomial_trajectory(float &current, float start, float target, float T, float t)
{
    float deltap = target - start;

    // polynomial coefficients (a1 = a2 = 0)
    float a0 = start;
    float a3 = 10 * deltap / (T*T*T);
    float a4 = -15 * deltap / (T*T*T*T);
    float a5 = 6 * deltap / (T*T*T*T*T);

    current = a0 + a3 * t*t*t + a4 * t*t*t*t + a5 * t*t*t*t*t;
}

void polynomial_trajectory_3d(Vector3f &current, Vector3f start, Vector3f target, float T, float t)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        polynomial_trajectory(current[i], start[i], target[i], T, t);
    }
}
