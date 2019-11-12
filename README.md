# Ghost ROS Brdige

This package bridges OpenCog's Ghost dialogue subsystem with ROS.
To learn about Ghost this
[README](https://github.com/opencog/opencog/blob/master/opencog/ghost/README.md)
will be useful.

## Building

In order for this package to be built the following dependencies
must be satisfied.
* [AtomSpace](https://github.com/opencog/atomspace.git)
* [Relex](https://github.com/opencog/relex.git)
* [CogServer](https://github.com/opencog/cogserver.git)
* [Attention](https://github.com/opencog/attention.git)
* [OpenCog](https://github.com/opencog/opencog.git)

Installing the above dependencies should be done in the order shown above. Note
that those packages themselves need their respective dependencies. Please check
out the [Dockerfile](../../Dockerfile) in this repo for the order of 
installation and extra dependencies.

## Directories

* [cmake](cmake/) : contains custom CMake config files for finding modules that
  do not have them by themselves.
* [include](include/) : contains cpp header files.
* [launch](launch/) : contains ROS launch file for running the package.
* [lib](lib/) : contains code that's supposed to be independently compiled as a
  library to be linked against the code for the node.
* [rules](rules/) : contains the Ghost rule files.
* [scm](scm/) : contains the Guile Scheme scripts to be loaded before the Ghost
  rules. These scheme scripts are mainly intended to contain schemas needed by
  the Ghost rules and to define utility procedures.
* [src](src/) : contains code for the node.

## Running

The
[Relex opencog server](https://github.com/opencog/relex/blob/master/opencog-server.sh)
needs to be launched before running this node.

It's recommended to use the [launch file](launch/ghost-bridge.launch) to run the
node because of the parameters the node requires to be set before running. These
parameters are:
* `ghost/rule_files` : an array of file paths for ghost rules.
* `ghost/scheme_modules` : an array of file paths for guile scheme modules to
  load.
* `ghost/response_wait_time` : on one-to-one mode, there would be a delay to wait
  for Ghost to come up with a reply after input text has been passed. The time
  unit is Seconds.
* `ghost/relex_hostname` : the hostname of the server running relex.
* `ghost/relex_port` : the port for connecting to the relex server.
