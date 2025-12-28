#pragma once

#include "config/config.h"
#include "vehicle/tire/tire.h"
#include "vehicle/vehicleHelper.h"

class Vehicle {
    VehicleConfig config;

    CarWheelBase<std::unique_ptr<Tire>> tires;
    CarWheelBase<float> distributeForces(float totalForce, float frontDist, float leftDist);
    CarWheelBase<float> totalTireLoads(float velocity, vec2<float> acceleration,
                                       const SimConfig& simConfig);
    CarWheelBase<float> staticLoad(float earthAcc);
    CarWheelBase<float> aeroLoad(float velocity, float airDensity);
    CarWheelBase<float> loadTransfer(vec2<float> acceleration);
    float getEngineTorque(float rpm);
    float getWheelTorque(float engine_torque, int gear);
    const float totalMass;

   public:
    Vehicle(VehicleConfig config);
    float getTireForces(float startSpeed, vec2<float> acceleration, const SimConfig& simConfig);
    float speedToRpm(float speed_ms, int gear);
    float getPowerThrust(float speed_ms, int gear);

    float getTotalMass();
    float getCRR();
    float getCDA();
    float getMaxTorqueRpm();
    unsigned int getGearCount();
    float getShiftTime();
};
