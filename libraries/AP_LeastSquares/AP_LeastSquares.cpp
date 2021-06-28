#include "AP_LeastSquares.h"
#include <GCS_MAVLink/GCS.h>


AP_LeastSquares::AP_LeastSquares()
    : _max_num_samples(AP_LEASTSQUARE_MAX_SAMPLES)
{
    reset();
}

void AP_LeastSquares::reset()
{
    // reset to default values
    _cur_num_samples = 0;
    _head = _max_num_samples - 1;
    _sum_x = _sum_y = _sum_xx = _sum_xy = 0.0f;
    _m = _b = 0;
}

void AP_LeastSquares::set_num_samples(int num_samples)
{
    if (num_samples == _max_num_samples) return;

    _max_num_samples = constrain_value(num_samples, 0, AP_LEASTSQUARE_MAX_SAMPLES);
    reset();
}

void AP_LeastSquares::add_sample(float x, float y)
{
    // increment head
    _head = (_head+1) % _max_num_samples;

    // retrieve old values at current head
    float old_x = 0.0f;
    float old_y = 0.0f;
    if (_cur_num_samples == _max_num_samples)
    {
        old_x = _samples[_head].x;
        old_y = _samples[_head].y;
    }

    // replace values at current head
    _samples[_head] = Vector2<float>(x, y);


    // update sums
    _sum_x += x - old_x;
    _sum_y += y - old_y;
    _sum_xx += x*x - old_x*old_x;
    _sum_xy += x*y - old_x*old_y;

    // increment sample counter
    _cur_num_samples = constrain_value(_cur_num_samples+1, 0, _max_num_samples);

    // perform linear regression
    update();
}

void AP_LeastSquares::update()
{
    // https://www.mathworks.com/help/curvefit/least-squares-fitting.html
    float N = float(_cur_num_samples);
    float denom = (N * _sum_xx - _sum_x * _sum_x);

    if (is_zero(N) || is_zero(denom)) return;

    // devation
    _m = (N * _sum_xy - _sum_x * _sum_y) / denom;

    // y-intercept
    _b = (_sum_y - _m * _sum_x) / N;

}
