#pragma once

#include "vehicle/vehicle.h"
#include "config/config.h"

class Simulation
{
protected:
    SimulationConstants simulationConstants;
    SimConfig simConfig;
    Vehicle &vehicle;

public:
    Simulation(Vehicle &vehicle, SimConfig simConfig, SimulationConstants simulationConstants);
    virtual float run() = 0;
    float calculatePoints(float time, float tMax, float pMax, const std::array<float, 3> &coef) const;
};
