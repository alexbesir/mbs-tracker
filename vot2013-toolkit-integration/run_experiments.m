% This script can be used to execute the experiments for a single tracker
% You can copy and modify it to create another experiment launcher

[sequences, experiments] = vot_environment();

tracker = create_tracker('MovingBackgroundSubtraction');

% experiments = experiments(1);
% sequences   = sequences(1);

vot_experiments(tracker, sequences, experiments);

