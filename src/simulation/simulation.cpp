#include "simulation/simulation.h"

#include <cmath>

Simulation::Simulation(Vehicle &vehicle, SimConfig simConfig, SimulationConstants simulationConstants) : vehicle(vehicle), simulationConstants(simulationConstants), simConfig(simConfig) {}

float Simulation::calculatePoints(float time, float tMax, float pMax, const std::array<float, 3> &coef) const
{
    float a = coef[0];
    float b = coef[1];
    float c = coef[2];
    return a * pMax * (std::pow(tMax / time, 2) - 1) / b + c * pMax;
}