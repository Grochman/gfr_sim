#include "vehicle/vehicle.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numbers>

#include "config/config.h"
#include "vehicle/tire/tire.h"
#include "vehicle/tire/tireSimple.h"
#include "vehicle/vehicleHelper.h"

Vehicle::Vehicle(VehicleConfig config)
    : config(config), totalMass(config.nonSuspendedMass + config.suspendedMass) {
    CarWheelBase<bool> isWheelDriven;
    for (size_t i = 0; i < CarAcronyms::WHEEL_COUNT; i++) {
        isWheelDriven[i] = true;
    }

    switch (config.driveType) {
        case CarAcronyms::RWD:
            isWheelDriven[CarAcronyms::FL] = false;
            isWheelDriven[CarAcronyms::FR] = false;
            break;
        case CarAcronyms::FWD:
            isWheelDriven[CarAcronyms::RL] = false;
            isWheelDriven[CarAcronyms::RR] = false;
            break;
    }

    for (size_t i = 0; i < CarAcronyms::WHEEL_COUNT; i++) {
        tires[i] = std::make_unique<TireSimple>(config.tireScalingFactor, config.quadFac,
                                                config.linFac, isWheelDriven[i]);
    }
}

float Vehicle::getTireForces(float velocity, vec2<float> acceleration, const SimConfig &simConfig) {
    auto loads = totalTireLoads(velocity, acceleration, simConfig);
    float force = 0;

    for (size_t i = 0; i < CarAcronyms::WHEEL_COUNT; i++) {
        if (acceleration.y > acceleration.x) {  // for now
            force += tires[i]->getLateralForce(loads[i]);
        } else {
            force += tires[i]->getLongitudinalForce(loads[i]);
        }
    }

    return force;
}

CarWheelBase<float> Vehicle::totalTireLoads(float velocity, vec2<float> acceleration,
                                            const SimConfig &simConfig) {
    auto static_load = staticLoad(simConfig.earthAcc);
    auto aero = aeroLoad(velocity, simConfig.airDensity);
    auto transfer = loadTransfer(acceleration);
    CarWheelBase<float> ret;
    for (size_t i = 0; i < CarAcronyms::WHEEL_COUNT; i++) {
        ret[i] = std::max(0.f, static_load[i] + aero[i] + transfer[i]);
    }
    return ret;
}

CarWheelBase<float> Vehicle::staticLoad(float earthAcc) {
    // assume the mass center is constant
    return distributeForces(totalMass * earthAcc, config.frontWeightDist, config.leftWeightDist);
}

CarWheelBase<float> Vehicle::distributeForces(float totalForce, float frontDist, float leftDist) {
    CarWheelBase<float> forces;
    forces[CarAcronyms::FL] = totalForce * frontDist * leftDist;
    forces[CarAcronyms::FR] = totalForce * frontDist * (1 - leftDist);
    forces[CarAcronyms::RL] = totalForce * (1 - frontDist) * leftDist;
    forces[CarAcronyms::RR] = totalForce * (1 - frontDist) * (1 - leftDist);
    return forces;
}

CarWheelBase<float> Vehicle::aeroLoad(float velocity, float airDensity) {
    // assume air density is constant
    // assume cla is const -> it should not be / cla for skidpad
    float totalForce = 0.5 * config.cla * airDensity * std::pow(velocity, 2);
    return distributeForces(totalForce, config.frontAeroDist, config.leftAeroDist);
}

// for now same function for transient and steady state
CarWheelBase<float> Vehicle::loadTransfer(vec2<float> acceleration) {
    CarWheelBase<float> loads;

    // pseudo transient
    float momentX = acceleration.x * totalMass * config.suspendedCenterOfGravityHeight;
    float transferX = momentX / config.wheelbase;

    float left = config.leftWeightDist;

    loads[CarAcronyms::FL] = -transferX * left;
    loads[CarAcronyms::FR] = -transferX * (1 - left);
    loads[CarAcronyms::RL] = transferX * left;
    loads[CarAcronyms::RR] = transferX * (1 - left);

    // steady state
    float nonSuspendedMomentY =
        config.nonSuspendedMass * acceleration.y * config.nonSuspendedCenterOfGravityHeight;

    float geometricMomentY = config.suspendedMass * acceleration.y * config.rollCenterHeight;

    float elasticMomentY = config.suspendedMass * acceleration.y *
                           (config.suspendedCenterOfGravityHeight - config.rollCenterHeight);

    float antiRollStiffnessTotal = config.antiRollStiffnessFront + config.antiRollStiffnessRear;

    float frontTransfer =
        (nonSuspendedMomentY + geometricMomentY +
         elasticMomentY * config.antiRollStiffnessFront / antiRollStiffnessTotal) *
        config.frontWeightDist / config.frontTrackWidth;
    float rearTransfer = (nonSuspendedMomentY + geometricMomentY +
                          elasticMomentY * config.antiRollStiffnessRear / antiRollStiffnessTotal) *
                         (1 - config.frontWeightDist) / config.rearTrackWidth;

    loads[CarAcronyms::FL] += -frontTransfer;
    loads[CarAcronyms::FR] += frontTransfer;
    loads[CarAcronyms::RL] += -rearTransfer;
    loads[CarAcronyms::RR] += rearTransfer;

    return loads;
}

float Vehicle::getTotalMass() { return totalMass; }
float Vehicle::getCRR() { return config.crr; }
float Vehicle::getCDA() { return config.cda; }
float Vehicle::getMaxTorqueRpm() { return config.maxTorqueRpm; }
unsigned int Vehicle::getGearCount() { return config.gearRatios.size(); }
float Vehicle::getShiftTime() { return config.shiftTime; }

float Vehicle::speedToRpm(float speed_ms, int gear) {
    if (gear < 0 || gear >= config.gearRatios.size()) {
        return config.idleRpm;
    }
    float ratio = config.gearRatios[gear] * config.finalDriveRatio;
    float rpm = (speed_ms / config.wheelRadius) * (30 / std::numbers::pi_v<float>)*ratio;
    return std::max(static_cast<float>(config.idleRpm),
                    std::min(rpm, static_cast<float>(config.redlineRpm)));
}

float Vehicle::getPowerThrust(float speed_ms, int gear) {
    float rpm = speedToRpm(speed_ms, gear);
    float torque = getEngineTorque(rpm);
    float wheelTorque = getWheelTorque(torque, gear);
    float thrust = wheelTorque / config.wheelRadius;
    return std::max(thrust, 0.f);
}

float Vehicle::getWheelTorque(float engine_torque, int gear) {
    if (gear < 0 || gear >= config.gearRatios.size()) {
        return 0.0;
    }
    return engine_torque * config.gearRatios[gear] * config.finalDriveRatio;
}

float Vehicle::getEngineTorque(float rpm) {
    auto it = std::lower_bound(config.torqueCurve.begin(), config.torqueCurve.end(), rpm,
                               [](auto &p, float v) { return p.first < v; });
    if (it == config.torqueCurve.begin()) return it->second;
    if (it == config.torqueCurve.end()) return (it - 1)->second;

    auto &[x0, y0] = *(it - 1);
    auto &[x1, y1] = *it;
    return y0 + (rpm - x0) * (y1 - y0) / (x1 - x0);
}
