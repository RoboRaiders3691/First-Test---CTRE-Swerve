 #include "Utilities/PathPlannerUtils.h"

 PathPlannerUtils::PathPlannerUtils(){

    
  
 }
 
 std::vector<std::string> PathPlannerUtils::AutosList(){

    // Define the directory path to list files from 
    directorypath = frc::filesystem::GetDeployDirectory().append("/pathplanner/autos");

    std::vector<std::string> fileVect;    

    // To check if the directory exists or not 
    if (std::filesystem::exists(directorypath)
        && std::filesystem::is_directory(directorypath)) {
        // Loop through each item (file or subdirectory) in 
        // the directory 
        for (const auto& entry :
            std::filesystem::directory_iterator(directorypath)) {
            std::string entryFileName = entry.path().string();

            //Erase the path to leave just file.extension
            entryFileName.erase(0, directorypath.string().length() + 1);

            //Erase all after '.' this removes the extension so only the file name remains
            entryFileName.erase(entryFileName.find('.'), entryFileName.length() - 1);

            fileVect.push_back(entryFileName);
        }
    }
    else {
        // Handle the case where the directory doesn't exist 
        fileVect.push_back("Error directory path not found.");
    }

    return fileVect;

 }

 
 std::vector<pathplanner::PathPlannerAuto> PathPlannerUtils::GetAutos(){

    std::vector<std::string> availableAutos = AutosList();
    std::vector<pathplanner::PathPlannerAuto> autosVect;

    for(int i = 0; i < int(availableAutos.size()); i++){

        autosVect.push_back(pathplanner::PathPlannerAuto(availableAutos[i]));

    }

    return autosVect;
 }
 