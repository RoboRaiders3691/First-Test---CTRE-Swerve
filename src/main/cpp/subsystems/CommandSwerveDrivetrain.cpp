#include "subsystems/CommandSwerveDrivetrain.h"

using namespace subsystems;

void CommandSwerveDrivetrain::Periodic()
{
    /* Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing. */
    if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled()) {
        auto const allianceColor = frc::DriverStation::GetAlliance();
        if (allianceColor) {
            SetOperatorPerspectiveForward(
                *allianceColor == frc::DriverStation::Alliance::kRed
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
            m_hasAppliedOperatorPerspective = true;
        }
    }
}

void CommandSwerveDrivetrain::StartSimThread()
{
    m_lastSimTime = utils::GetCurrentTime();
    m_simNotifier = std::make_unique<frc::Notifier>([this] {
        units::second_t const currentTime = utils::GetCurrentTime();
        auto const deltaTime = currentTime - m_lastSimTime;
        m_lastSimTime = currentTime;

        /* use the measured time delta, get battery voltage from WPILib */
        UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());
    });
    m_simNotifier->StartPeriodic(kSimLoopPeriod);
}

frc::ChassisSpeeds CommandSwerveDrivetrain::ChassisSpeedsCTREToFrc(
    ctre::phoenix6::swerve::impl::ChassisSpeeds inputChassisSpeed
    ){
    frc::ChassisSpeeds newChassisSpeed {inputChassisSpeed.vx, inputChassisSpeed.vy, inputChassisSpeed.omega};
    return newChassisSpeed;
}

ctre::phoenix6::swerve::impl::ChassisSpeeds CommandSwerveDrivetrain::ChassisSpeedsFrcToCTRE(
     frc::ChassisSpeeds inputChassisSpeed
    ){
    ctre::phoenix6::swerve::impl::ChassisSpeeds newChassisSpeed {inputChassisSpeed.vx, inputChassisSpeed.vy, inputChassisSpeed.omega};
    return newChassisSpeed;
}

void CommandSwerveDrivetrain::ConfigurePathPlanner(){

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();

    // Configure the AutoBuilder last
    pathplanner::AutoBuilder::configure(
        [this](){ return this->GetState().Pose; }, // Robot pose supplier
        [this](frc::Pose2d newPose){ this->SeedFieldRelative(newPose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return this->ChassisSpeedsCTREToFrc(GetState().Speeds); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](auto speeds){m_AutoRequest.WithSpeeds(ChassisSpeedsFrcToCTRE(speeds));}, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<pathplanner::PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
            pathplanner::PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            pathplanner::PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );

}
