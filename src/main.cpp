#include "vehicle/vehicle.h"
#include "simulation/skidpad.h"
#include "config/config.h"

#include <cstdio>
#include <iostream>

int main()
{
    VehicleConfig vc;
    SimConfig sc;
    SkidPadConfig skidPadConfig;
    SimulationConstants simulationConstants;
    PointsConfig pointsConfig;

    Vehicle v(vc);
    SkidPad s(v, sc, simulationConstants, skidPadConfig);
    float lapTime = s.run();

    float points = s.calculatePoints(lapTime, pointsConfig);
    printf("lap time: %f\npoints: %f\n", lapTime, points);
}