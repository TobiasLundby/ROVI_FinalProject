# Installation guide

Clone the repository
```sh
$ https://github.com/TobiasLundby/ROVI_FinalProject.git
```
Cd into ROVI_FinalProject
```sh
$ cd ROVI_FinalProject
```
Create build directory
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
When the plugin is loaded, you will be able to run the visual servoing using two markers(marker1.ppm, marker3.ppm) or just by tracking the position of the marker(without vision). When a run has completed, tow files will be generated. 

```sh
$ q_markerX_sequence_Y.log
```
Contains a log of the robots configuration. X and Y represents the chosen marker and sequence, respectively.
```sh
$ log_markerX_sequence_Y.log
```
Contains the transformation of the camera with respect to world frame.
