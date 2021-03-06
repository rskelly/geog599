# Cubic Spline Trajectory Extraction

This is a command-line program which demonstrates the use of smoothing or approximating splines for the development of a vertical trajectory for UAVs from a point cloud. This contributes to research into a forward-looking, predictive trajectory extraction program.

The program simulates a UAV in flight by extracting points from a point cloud in a pattern which resembles what a forward-facing scanning rangefinder would produce. This dynamic point cloud is subjected to a surface-extraction algorithm, followed by a smoothing spline algorithm. The spline coefficients allow the vehicle to sample the spline trajectory at any point and send the altitude to the flight computer. In this example, the point, surface and spline are drawn on screen, rather than being used for navigation.

![Profile Image](http://dijital.ca/ftp/msc/traj_screenshot.png)

## Building

The usual cmake process applies:

1) mkdir build && cd build
2) cmake ..
3) make

## Profiles

The program depends on pre-prepared point clouds, which must exist in a folder named "data" in the current directory when the program is run (from the command line).

These can be downloaded [here](http://dijital.ca/ftp/msc/profiles.zip).

## Running

To run the program, try (from the build folder):

    bin/pipeline

It will print out a list of available configurations. Then,

    bin/pipeline <config name>

will run the program using a specific configuration.


## Documentation

Basic Doxygen documentatin can be found [here](http://dijital.ca/ftp/msc/traj_docs/index.html).