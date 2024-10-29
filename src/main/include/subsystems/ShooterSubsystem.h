#pragma once

#include <units/time.h>
#include <units/angular_velocity.h>

#include <frc/Timer.h>

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/configs/Configurator.hpp"
#include "ctre/phoenix6/configs/Configs.hpp"
#include "ctre/phoenix6/CANcoder.hpp"

#include "Constants.h"

class ShooterSubsystem : public frc2::SubsystemBase{
    public:
        ShooterSubsystem();
        void Periodic();
        void SetVelocity(units::angular_velocity::turns_per_second_t angularVelTarget);
        void RunFor(units::angular_velocity::turns_per_second_t velocity, units::time::second_t time);
        units::angular_velocity::turns_per_second_t GetVelocity();

    private:
        ctre::phoenix6::hardware::TalonFX RightMotor;
        ctre::phoenix6::hardware::TalonFX LeftMotor;
        ctre::phoenix6::controls::MotionMagicVelocityVoltage m_velRequest;

        frc::Timer m_shootTimer;

};