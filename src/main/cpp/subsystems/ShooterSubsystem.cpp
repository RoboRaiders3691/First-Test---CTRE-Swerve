#include <subsystems/ShooterSubsystem.h>

using namespace ShooterConstants;

ShooterSubsystem::ShooterSubsystem() : 
    RightMotor{kShooterMotorRightPort},
    LeftMotor{kShooterMotorLeftPort},
    m_velRequest{kArbitraryFeedForwardDriveVelocity}
    {

    RightMotor.GetConfigurator().Apply(kShooterMotorConfigs);

    LeftMotor.SetControl(ctre::phoenix6::controls::Follower{kShooterMotorRightPort, true});

}

void ShooterSubsystem::Periodic(){



}

void ShooterSubsystem::SetVelocity(units::angular_velocity::turns_per_second_t angularVelTarget){
    RightMotor.SetControl(m_velRequest.WithVelocity(angularVelTarget));
}

units::angular_velocity::turns_per_second_t ShooterSubsystem::GetVelocity(){
    return RightMotor.GetVelocity().GetValue();
}

void ShooterSubsystem::RunFor(units::angular_velocity::turns_per_second_t velocity, units::time::second_t time){

    m_shootTimer.Start();
    m_shootTimer.Reset();

    while(!m_shootTimer.HasElapsed(time)){
        SetVelocity(50_tps);
    }

    SetVelocity(0_tps);
    m_shootTimer.Stop();

}
