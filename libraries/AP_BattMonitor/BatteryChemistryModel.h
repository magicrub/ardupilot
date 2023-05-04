#pragma once

#include "LinearInterpolator.h"

class BatteryChemistryModel {
public:
    virtual float OCV_from_SOC(float SOC, float temp_C) const = 0;
    virtual float OCV_from_SOC_diff(float SOC, float temp_C) const = 0;
    virtual float OCV_from_SOC_integral(float SOC, float temp_C) const = 0;
    virtual float SOC_from_OCV(float OCV, float temp_C) const = 0;
    virtual float SOC_from_OCV_diff(float OCV, float temp_C) const = 0;
    virtual float min_valid_OCV(float temp_C) const = 0;
    virtual float max_valid_OCV(float temp_C) const = 0;
};

class BatteryChemistryModelLinearInterpolated : public BatteryChemistryModel {
public:
    BatteryChemistryModelLinearInterpolated(const float* x_points, const float* y_points, size_t num_points) :
    _interpolator_OCV_from_SOC(x_points, y_points, num_points),
    _interpolator_SOC_from_OCV(y_points, x_points, num_points),
    _minv(y_points[0]),
    _maxv(y_points[num_points-1])
    {}

    virtual float OCV_from_SOC(float SOC, float temp_C) const override {
        (void)temp_C;
        return _interpolator_OCV_from_SOC.eval(SOC);
    }
    virtual float OCV_from_SOC_diff(float SOC, float temp_C) const override {
        (void)temp_C;
        return _interpolator_OCV_from_SOC.slope(SOC);
    }
    virtual float OCV_from_SOC_integral(float SOC, float temp_C) const override  {
        (void)temp_C;
        return _interpolator_OCV_from_SOC.definite_integral(0, SOC);
    }
    virtual float SOC_from_OCV(float OCV, float temp_C) const override {
        (void)temp_C;
        return _interpolator_SOC_from_OCV.eval(OCV);
    }
    virtual float SOC_from_OCV_diff(float OCV, float temp_C) const override {
        (void)temp_C;
        return _interpolator_SOC_from_OCV.slope(OCV);
    }
    virtual float min_valid_OCV(float temp_C) const override {
        (void)temp_C;
        return _minv;
    }
    virtual float max_valid_OCV(float temp_C) const override {
        (void)temp_C;
        return _maxv;
    }


private:
    LinearInterpolator<float> _interpolator_OCV_from_SOC;
    LinearInterpolator<float> _interpolator_SOC_from_OCV;
    float _minv, _maxv;
};
