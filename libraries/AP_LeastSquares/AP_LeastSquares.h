#pragma once

/// @file    AP_LeastSquares.h
/// @brief   Least Square Regression Library

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/vectorN.h>

#define AP_LEASTSQUARE_MAX_SAMPLES 100

class AP_LeastSquares {
public:
    AP_LeastSquares();

    // Empty destructor to suppress compiler warning
    virtual ~AP_LeastSquares() {}

    // set sample size
    void set_num_samples(int num_samples);

    // get number of current samples
    int num_samples() { return _cur_num_samples; }

    // get maximum number of samples
    int max_num_samples() { return _max_num_samples; }

    // enables removal of outliers
    void enable_outlier_removal(bool enable) { _filter_outliers = enable; }

    // set outlier threshold
    void set_outlier_threshold(float outlier_threshold) { _outlier_threshold = outlier_threshold; }

    // add a new xy sample
    void add_sample(float x, float y);

    // get y extrapolated via linear regression model
    float get_y(float x) { return _m*x+_b; }

    // get most recently added sample
    Vector2<float> latest_sample() { return _samples[_head]; }

    // clear all samples
    void reset();

private:

    // perform the linear regression
    void update();

    // max number of samples to consider
    int _max_num_samples;

    // current number of added samples
    int _cur_num_samples;

    // pointer to the most recently added sample
    int _head;

    // whether to remove outliers
    bool _filter_outliers;

    // threshold between sample and regression line beyond which to consider a sample as outlier
    float _outlier_threshold;

    // list of xy samples where linear regression is performed on
    Vector2<float> _samples[AP_LEASTSQUARE_MAX_SAMPLES];

    // sums, used for linear regression formula
    float _sum_x;
    float _sum_y;
    float _sum_xx;
    float _sum_xy;

    // deviation m and y-intercept b of the linear regression function
    float _m;
    float _b;

};
