/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-edu.
 *
 * aruw-edu is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-edu is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-edu.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "standard.hpp"
#include "drivers_singleton.hpp"

#include "tap/util_macros.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"

#include "control/chassis/chassis_subsystem.hpp"
#include "control/chassis/chassis_tank_drive_command.hpp"

#include "agitator/agitator_rotate_command.hpp"
#include "agitator/agitator_reverse_command.hpp"
#include "agitator/agitator_subsystem.hpp"

#include "flywheel/flywheel_subsystem.hpp"
#include "flywheel/flywheel_on_command.hpp"

#include "drivers.hpp"
#include "drivers_singleton.hpp"


using tap::can::CanBus;
using tap::communication::serial::Remote;
using tap::control::RemoteMapState;
using tap::motor::MotorId;

using namespace tap::control;
using namespace control;
using namespace tap::communication::serial;

driversFunc drivers = DoNotUse_getDrivers;
control::flywheel::FlywheelSubsystem theFlywheel(drivers());
control::agitator::AgitatorSubsystem theAgitator(drivers());

control::flywheel::FlywheelOnCommand flywheelCommand(&theFlywheel);
control::agitator::AgitatorRotateCommand rotateCommand(&theAgitator);

namespace control
{

HoldRepeatCommandMapping leftSwitchUp(
    drivers(),
    {&rotateCommand, &flywheelCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP), true, -1);

Robot::Robot(Drivers &drivers) : drivers(drivers),
// STEP 3 (Tank Drive): construct ChassisSubsystem and ChassisTankDriveCommand
chassis(drivers, chassis::ChassisConfig{
    .leftFrontId = MotorId::MOTOR2,
    .leftBackId = MotorId::MOTOR3,
    .rightBackId = MotorId::MOTOR4,
    .rightFrontId = MotorId::MOTOR1,
    .canBus = CanBus::CAN_BUS1,
    .wheelVelocityPidConfig = modm::Pid<float>::Parameter(10,0,0,0,1000),
    
}),
chassisTankDrive(chassis, drivers.controlOperatorInterface),
agitator(&drivers, MotorId::MOTOR7, CanBus::CAN_BUS1, false, "e"),
eduPidConfig{
    .kp = 1000,
    .ki = 0,
    .kd = 0,
    .maxICumulative = 0,
    .maxOutput = 16000
},
moveIntegralConfig{
    .targetIntegralChange = M_TWOPI / 10.0f,
    .desiredSetpoint = M_TWOPI,
    .integralSetpointTolerance = 0
},
// STEP 3 (Agitator Control): construct VelocityAgitatorSubsystem and MoveIntegralCommand
velocityAgitatorSubsystem(drivers, eduPidConfig, agitator), // FIX LATER
moveIntegralCommand(velocityAgitatorSubsystem, moveIntegralConfig),
// STEP 8 (Agitator Control): construct HoldRepeatCommandMapping and HoldCommandMapping
rightSwitchUp(&drivers, {&moveIntegralCommand}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP), false),
HCM(&drivers, {&moveIntegralCommand}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP))
{
}

void Robot::initSubsystemCommands()
{
    initializeSubsystems();
    registerSoldierSubsystems();
    setDefaultSoldierCommands();
    startSoldierCommands();
    registerSoldierIoMappings();
}

void Robot::initializeSubsystems()
{
    // STEP 4 (Tank Drive): initialize declared ChassisSubsystem
    chassis.initialize();
    // STEP 4 (Agitator Control): initialize declared VelocityAgitatorSubsystem
    velocityAgitatorSubsystem.initialize();
    theFlywheel.initialize();

}

void Robot::registerSoldierSubsystems()
{
    // STEP 5 (Tank Drive): register declared ChassisSubsystem
    drivers.commandScheduler.registerSubsystem(&chassis);
    // STEP 5 (Agitator Control): register declared VelocityAgitatorSubsystem
    drivers.commandScheduler.registerSubsystem(&velocityAgitatorSubsystem);
    drivers.commandScheduler.registerSubsystem(&theFlywheel);

}

void Robot::setDefaultSoldierCommands()
{
    // STEP 6 (Tank Drive): set ChassisTanKDriveCommand as default command for ChassisSubsystem
   chassis.setDefaultCommand(&chassisTankDrive); 
}

void Robot::startSoldierCommands() {}

void Robot::registerSoldierIoMappings()
{   
    // STEP 9 (Agitator Control): register HoldRepeatCommandMapping and HoldCommandMapping
    drivers.commandMapper.addMap(&rightSwitchUp);
    drivers.commandMapper.addMap(&HCM);
    drivers.commandMapper.addMap(&leftSwitchUp);
    
}
}  // namespace control
