# Installation guide

Clone the repository
```sh
$ https://github.com/TobiasLundby/ROVI_FinalProject.git
```
Cd into ROVI_FinalProject
```sh
$ cd ROVI_FinalProject
```
In order to load the required files such as markers and motions, you'll need to correct the path in src/RobWorkStudioPlugin.hpp. The section looks like this:

```sh
/* EDIT HERE */
std::string plugin_path = "/home/student/SamplePluginPA10/"; // Path to SamplePluginPA10
int NumberOfPoints = 1; // Number of points to track, can either be 1,2 or 2.
bool Testrun = true; // Determines whether testdata should be generated, see section further down about logging.
/* END EDIT HERE */
```

When ready to build, create build directory
```sh
$ mkdir build
```
Cd into build directory and run cmake ..
```sh
$ cd build
$ cmake ..
```

If cmake success generating the makefile then continue, else fix errors/missing dependencies.
```sh
$ make -j <Number of threads>
```
When compiled, open robwork and load the plugin in ROVI_FinalProject/libs/release.
When the plugin is loaded, you will be able to run the visual servoing using two markers(marker1.ppm, marker3.ppm) or just by tracking the position of the marker(without vision).


When a run has completed, three logs are generated.

```sh
$ log_Markerpose_MarkerMotionX.txtY.csv
```
Where X is the chosen motion and Y is the dt. It contains the transformation of the camera with respect to world frame.

```sh
$ error_Markerpose_MarkerMotionX.txtY.csv
```
It contains the tracking error in x and y in image frame.

```sh
$ q_Markerpose_MarkerMotionX.txtY.csv
```
Contains the configuration of the robot in each timestep.

