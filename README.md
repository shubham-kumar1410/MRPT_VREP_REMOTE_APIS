# mrpt_vrep_bridge

This package is a **work in progress**. Functionality advertised in the README
file probably won't work or may be buggy.

## Description

This repository provides packages and tools for interfacing the Mobile Robot
Programming Toolkit ([MRPT](http://www.mrpt.org/)) with the V-REP robotics
simulator. Following is a list of the functionalities that it provides:

* Remote API for MRPT Graphslam.
* TODO
* ...

## Project layout:

TODO - Describe the layout of each one of the libraries as well as the
functionality of each one of its subdirectories

## Installation
* [Clone the project from Github](https://github.com/shubham-kumar1410/mrpt_vrep_remoteapi)

* Create an environment variable VREP which contains the path to the VREP directory.
```bash
export VREP="path to VREP"
```
* Build with cmake
```bash
mkdir build && cd build
cmake ..
make
```


## Usage

###To run GraphSlam Remote API###

* Run an instance of VREP and load a scene with a fast hokuyu laser scanner.
* Edit the script of the hokuyu laser by changing the contents of the child script with the one located in *mrpt_graphslam_2d/lua_scripts*
* Run the executable built by 
```bash
./graphslamapi node_reg edge_reg optimizer configuration_file
```
The api has been tested with the following specifications:
	* NodeRegistrationDecider : CFixedIntervalsNRD
	* EdgeRegistrationDecider : CICPGoodnessERD
	* Optimizer               : CLevMarqGSO

## Notes

* Code targets exclusively the MRPT-2.x code. No backwards compatibility with
    MRPT 1.5 is provided.

## Demos

TODO

## Contributing

Code in the current repo abides to the same [guidelines as
MRPT](https://github.com/MRPT/mrpt/blob/master/.github/CONTRIBUTING.md).

On using the git VCS, do have the following bullet points in mind:


1. Separate subject from body with a blank line
2. Limit the subject line to 50 characters
3. Capitalize the subject line
4. Do not end the subject line with a period
5. Use the imperative mood in the subject line
6. Wrap the body at 72 characters
7. Use the body to explain what and why vs. how

For more details about the desired style of git commit messages have a read at
these articles:

* https://robots.thoughtbot.com/5-useful-tips-for-a-better-commit-message
* https://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html
* https://chris.beams.io/posts/git-commit/

