/*
 * Copyright (c) 2020-2021 Queen's Knights Robotics Team
 *
 * This file is part of qkrt-mcb.
 *
 * qkrt-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * qkrt-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with qkrt-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "standard.hpp"

#include "tap/util_macros.hpp"

// #include "turret/turret_subsystem.hpp"
// #include "turret/turret_gimbal_command.hpp"

#include "control/chassis/chassis_subsystem.hpp"
#include "control/chassis/chassis_omni_drive_command.hpp"

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

namespace control
{
/* define subsystems --------------------------------------------------------*/
control::agitator::AgitatorSubsystem theAgitator(drivers());
control::flywheel::FlywheelSubsystem theFlywheel(drivers());

/* define commands ----------------------------------------------------------*/
control::agitator::AgitatorRotateCommand rotateCommand(&theAgitator);
control::agitator::AgitatorReverseCommand reverseCommand(&theAgitator);
control::flywheel::FlywheelOnCommand flywheelCommand(&theFlywheel);

/* define command mappings --------------------------------------------------*/
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&rotateCommand, &flywheelCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH,Remote::SwitchState::UP), true, -1);

HoldRepeatCommandMapping rightSwitchDown(
    drivers(),
    {&reverseCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN), true, -1);

HoldRepeatCommandMapping leftSwitchUp(
    drivers(),
    {&flywheelCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP), true, -1);


    
Robot::Robot(Drivers &drivers) 
    : drivers(drivers),
        // construct ChassisSubsystem and ChassisTankDriveCommand
        chassis(drivers, chassis::ChassisConfig{
            .leftFrontId = MotorId::MOTOR2,
            .leftBackId = MotorId::MOTOR3,
            .rightBackId = MotorId::MOTOR4,
            .rightFrontId = MotorId::MOTOR1,
            .canBus = CanBus::CAN_BUS1,
            .wheelVelocityPidConfig = modm::Pid<float>::Parameter(10,0,0,0,1000),
            
        }),
        chassisOmniDrive(chassis, drivers.controlOperatorInterface),
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
        }
        // construct VelocityAgitatorSubsystem and MoveIntegralCommand
        // velocityAgitatorSubsystem(drivers, eduPidConfig, agitator), // FIX LATER
        // moveIntegralCommand(velocityAgitatorSubsystem, moveIntegralConfig),
        // construct HoldRepeatCommandMapping and HoldCommandMapping
        // // rightSwitchUp(&drivers, {&moveIntegralCommand}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP), false),
        // HCM(&drivers, {&moveIntegralCommand}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP)),
{

}

// void Robot::initSubsystemCommands()
// {
//     initializeSubsystems();
//     registerSoldierSubsystems();
//     setDefaultSoldierCommands();
//     startSoldierCommands();
//     registerSoldierIoMappings();
// }

void Robot::initializeSubsystems()
{
    // STEP 4 (Tank Drive): initialize declared ChassisSubsystem
    chassis.initialize();
    // STEP 4 (Agitator Control): initialize declared VelocityAgitatorSubsystem
    theFlywheel.initialize();
    theAgitator.initialize();
}

// /* define command mappings --------------------------------------------------*/
// HoldRepeatCommandMapping rightSwitchUp(
//     drivers(),
//     {&rotateCommand},
//     RemoteMapState(Remote::Switch::RIGHT_SWITCH,Remote::SwitchState::UP), true, -1);

// HoldRepeatCommandMapping rightSwitchDown(
//     drivers(),
//     {&reverseCommand},
//     RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN), true, -1);

// HoldRepeatCommandMapping leftSwitchUp(
//     drivers(),atCommandMapping rightSwitchUp(
//     drivers(),
//     {&rotateCommand},
//     RemoteMapState(Remote::Switch::RIGHT_SWITCH,Remote::SwitchState::UP), true, -1);

// HoldRepeatCommandMapping rightSwitchDown(
//     drivers(),
//     {&reverseCommand},
//     RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN), true, -1);

// HoldRepeatCommandMapping leftSwitchUp(
//     drivers(),
//     {&flywheelCommand},
//     RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP), true, -1);

void Robot::registerSoldierSubsystems()
{
    // STEP 5 (Tank Drive): register declared ChassisSubsystem
    drivers.commandScheduler.registerSubsystem(&chassis);

}
/* register subsystems here -------------------------------------------------*/
void registerStandardSubsystems(tap::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&theAgitator);
    drivers->commandScheduler.registerSubsystem(&theFlywheel);
}


void Robot::setDefaultSoldierCommands()
{
    // STEP 6 (Tank Drive): set ChassisTanKDriveCommand as default command for ChassisSubsystem
   chassis.setDefaultCommand(&chassisOmniDrive); 
}

/* add any starting commands to the scheduler here --------------------------*/
void startStandardCommands(tap::Drivers *) {}

/* register io mappings here ------------------------------------------------*/
void registerStandardIoMappings(tap::Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchUp);
}

void Robot::initSubsystemCommands(tap::Drivers *drivers)
{
    initializeSubsystems();
    registerStandardSubsystems(drivers);
    setDefaultSoldierCommands();
    startStandardCommands(drivers);
    registerStandardIoMappings(drivers);
}

}  // namespace control
