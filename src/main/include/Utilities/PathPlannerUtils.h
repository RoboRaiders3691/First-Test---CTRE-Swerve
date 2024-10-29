#pragma once

#include <filesystem>
#include <frc/Filesystem.h>

#include <vector>
#include <iostream>

#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <pathplanner/lib/commands/PathPlannerAuto.h>

class PathPlannerUtils{
    public:

        PathPlannerUtils();

        std::vector<std::string> AutosList();

        std::vector<pathplanner::PathPlannerAuto> GetAutos();

    private:
        std::filesystem::path directorypath;

};