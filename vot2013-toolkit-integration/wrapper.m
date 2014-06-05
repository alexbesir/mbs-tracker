function wrapper()
% MovingBackgroundSubtraction wrapper for VOT2013
% Created by Aleksander Besir
% alex.besir@gmail.com
% Faculty of Computer and Information Science Ljubljana, 2014

    %% VOT Framework initialization

    % Call exit command at the end to terminate Matlab
    cleanup = onCleanup(@() exit() );

    % Set random seed to a different value every time
    RandStream.setGlobalStream(RandStream('mt19937ar', 'Seed', sum(clock)));

    tracker_directory = fullfile(fileparts(mfilename('fullpath')), 'tracker');
    warning('off','MATLAB:rmpath:DirNotFound');
    rmpath(tracker_directory);
    addpath(tracker_directory);

    % Read input data
    [imgFileNames, region] = tracker_initialize();
    
    %% Run tracking
    options = struct();
    BB = MBS_track(imgFileNames,region,options);
    
    %% VOT Framework deinitialization
    tracker_deinitialize(BB);

end