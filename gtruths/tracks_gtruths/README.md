This folder contains all the tracks used to test the planning node, as well as a ground truth corresponding to each track (the expected output). Each folder inside the "tracks_gtruths" folder should have a name ({name}) and two .csv files.

The file named "track_to_gtruth.py" is a python script that allows easier conversion (though manual) from track to ground truth. This file requires the matplotlib and pandas libraries. If you run the file a window will open and a track will be displayed. You can zoom in/out by scrolling the mouse wheel. You can use the tools displayed above the track to view only a portion of the track or drag the track. To add the middle path's points by clicking with your mouse. These points will automatically be exported to a .csv file with name {name}_gtruth.csv where {name} is the name of the track. Make sure to always run the "dependencies_install.sh" file from root since this file depends on matplotlib and pandas libraries to run.

One file should be named {name}.csv and the track should be stored there; each cone should be stored in a separate line and each cone has the following format: x, y, color.

The second file should be named {name}_gtruth.csv and the ground truth should be stored there; one path point should be stored per line, each having the following format: x, y, ideal_velocity.

