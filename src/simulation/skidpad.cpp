#include "simulation/skidpad.h"

#include <cmath>
#include <cstdio>
#include <numbers>

SkidPad::SkidPad(Vehicle &vehicle, SimConfig simConfig, SimulationConstants simulationConstants, SkidPadConfig skidPadConfig) : trackConfig(skidPadConfig), Simulation(vehicle, simConfig, simulationConstants)
{
    diameter = trackConfig.minDiameter + trackConfig.diameterOffset;
    radius = diameter / 2;
    trackLength = 2 * std::numbers::pi_v<float> * radius;
}

float SkidPad::run()
{
    float lapTime = 0;
    float acceleration = std::pow(simConfig.startSpeed, 2) / radius;
    float maxIterations = simulationConstants.trackMaxIterations;
    float minIterations = simulationConstants.trackMinIterations;

    for (int i = 0; i < maxIterations; i++)
    {
        float forces = vehicle.getTireForces(simConfig.startSpeed, acceleration, simConfig, true);
        float newAcc = forces / vehicle.getMass();
        float newVelocity = std::sqrt(newAcc * radius);
        float newLapTime = trackLength / newVelocity;

        if (i >= minIterations and abs(newLapTime - lapTime) <= simConfig.errDelta)
        {
            break;
        }

        lapTime = newLapTime;
        acceleration = newAcc;
    }

    return lapTime;
}

float SkidPad::calculatePoints(float time, const PointsConfig &pointsConfig) const
{
    return Simulation::calculatePoints(time, pointsConfig.historicalBestTime * 1.25, pointsConfig.historicalPMax, pointsConfig.pointsCoefficients);
}