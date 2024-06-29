function [] = context()
    % 경로 추가
    % Get the current folder path
    currentFolderpath = pwd;
    % Change to wrench stability folder
    cd ..;
    % Confirm the change
    wrenchstabilityFolderpath = pwd;
    % Generate a path string that includes all subdirectories
    allSubfoldersPath = genpath(wrenchstabilityFolderpath);
    % Add the generated path string to the MATLAB path
    addpath(allSubfoldersPath);
    cd(currentFolderpath);
end

