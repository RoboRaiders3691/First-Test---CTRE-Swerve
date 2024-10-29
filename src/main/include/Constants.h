#pragma once

#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/angular_jerk.h>

#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/configs/Configurator.hpp"
#include "ctre/phoenix6/configs/Configs.hpp"
#include "ctre/phoenix6/CANcoder.hpp"

namespace ShooterConstants{
    static constexpr int kShooterMotorRightPort = 31; //Use placements in the 30s because they do not exist yet
    static constexpr int kShooterMotorLeftPort = 32; //Replace with correct motor ports once a shooter has been implemented

    static constexpr ctre::phoenix6::configs::TalonFXConfiguration kShooterMotorConfigs = ctre::phoenix6::configs::TalonFXConfiguration{}
        .WithSlot0(ctre::phoenix6::configs::Slot0Configs{}
            .WithKS(.25) // Add 0.25 V output to overcome static friction
            .WithKV(.12) // A velocity target of 1 rps results in 0.12 V output
            .WithKA(.01) // An acceleration of 1 rps/s requires 0.01 V output
            .WithKP(4.8) // A position error of 2.5 rotations results in 12 V output
            .WithKI(0) // no output for integrated error
            .WithKD(.1) // A velocity error of 1 rps results in 0.1 V output
        )
        .WithMotionMagic(ctre::phoenix6::configs::MotionMagicConfigs{}
            .WithMotionMagicCruiseVelocity(80_tps) // Target cruise velocity of 80 tps
            .WithMotionMagicAcceleration(160_tr_per_s_sq) // Target acceleration of 160 tps/s (0.5 seconds)
            .WithMotionMagicJerk(1600_tr_per_s_cu) // Target jerk of 1600 tps/s/s (0.1 seconds)
        );

        static constexpr units::angular_velocity::turns_per_second_t kArbitraryFeedForwardDriveVelocity = 0_tps;

}