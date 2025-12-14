#pragma once

#include "vehicle/tire/tire.h"

class TireSimple : public Tire
{
    float calculateForce(float verticalLoad, bool isLateral);
public:
    TireSimple() = default;
    TireSimple(float scalingFactor, float quadFac, float linFac, bool isDriven);
    float getLatrealForce(float verticalLoad) override;
    float getLongitudinalForce(float verticalLoad) override;
};

