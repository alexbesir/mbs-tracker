mbs-tracker > vot2013-toolkit integration
=========================================

To integrate the MBS-tracker with VOT2013-toolkit:

1. copy the `MBS_track.m` file to the `tracker` directory
2. update `line 17` in `tracker_MovingBackgroundSubtraction.m` so that it points to your local Matlab installation.

The evauation process is strted by running the `run_experiments.m` script. Be
sure that the directory with this script is in Matlab's path.
