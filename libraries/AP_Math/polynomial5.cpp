#include "AP_Math.h"

void polynomial_trajectory(Vector3f &current, Vector3f start, Vector3f target, float T, float t)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        float deltap = target[i] - start[i];

        // polynomial coefficients (a1 = a2 = 0)
        float a0 = start[i];
        float a3 = 10 * deltap / (T*T*T);
        float a4 = -15 * deltap / (T*T*T*T);
        float a5 = 6 * deltap / (T*T*T*T*T);

        current[i] = a0 + a3 * t*t*t + a4 * t*t*t*t + a5 * t*t*t*t*t;
    }
}
