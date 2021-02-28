# UAV-ROS

We are developing an autononmous drone that could compete in IARC Mission 9.

---

# Installing

You can install this package in the traditional or conventional way.

## Traditional

To install this repository into your catkin workspace we first need to know the catkin directory structure

```
catkin_ws
|-- build 				// Where all the generated libraies and executables end up
|-- devel 				// Where our setup scripts are
    |-- setup.bash  	// Set our $ROS_PACKAGE_PATH to the current directory plus other environment stuff.
    |-- setup.sh 		// Set our $ROS_PACKAGE_PATH to the current directory plus other environment stuff.
|-- src
    |-- CMakeLists.txt
    |-- Package1		// Sample Project 
        |-- src
        |-- CMakeLists.txt
        |-- Package.xml
|   .catkin_workspace
```
Now we have a current idea of what our Catkin workspace looks like, now type in the following command to clone the current repository. Let's go to our `catkin_ws/src` directory and clone the current repository.

HTTPS: `git clone https://github.com/UNL-UAV/uav-ros --recursive`

SSH: `git clone git@github.com:UNL-UAV/uav-ros.git --recursive`

Our `catkin_ws` should look like:
```
catkin_ws
|-- build 				// Where all the generated libraies and executables end up
|-- devel 				// Where our setup scripts are
    |-- setup.bash  	// Set our $ROS_PACKAGE_PATH to the current directory plus other environment stuff.
    |-- setup.sh 		// Set our $ROS_PACKAGE_PATH to the current directory plus other environment stuff.
|-- src
    |-- CMakeLists.txt
    |-- Package1		// Sample Project 
        |-- src
        |-- CMakeLists.txt
        |-- Package.xml
    |-- uav-ros			// Our ROS Project
        |-- include 
        |-- src
        |-- CMakeLists.txt
        |-- package.xml
        |-- ...
|   .catkin_workspace
```
You're all done.

## Conventional

Some developers has developed a more conventional and traditional way of programming. Which is not completely bad or wrong. Our project adapt for that

Let say we put all of our local git repository under `~/git/`. You can just clone the repository like normal under the `~/git` directory.

HTTPS: `git clone https://github.com/UNL-UAV/uav-ros`

SSH: `git clone git@github.com:UNL-UAV/uav-ros.git`

```
~/git
|-- test
|-- repo1
|-- repo2
|-- ...
|-- uav-ros			// Our ROS Project
        |-- include 
        |-- src
        |-- premake5.lua
        |-- package.xml
        |-- ...
```

Now we got to make sure ROS can see this repository we have to add `~/git` to the `$ROS_PACKAGE_PATH` global varible. In a terminal type the following command. Making sure that you're replacing `/path/to/git`. We could also add this command into our `~/.bash_rc` file.

`export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/path/to/git`

---

# Building

## Traditional
To build using the traditional method we can invoke the `catkin_make` command

## Conventinal 
To build using the conventional method we can be inside of the repository directory and run the follwoing commands
```
premake5 gmake2
make -C workspace
```
**Note**: You do not have to run `premake5 gmake2` command everytime. You have to run the command on your first build and every `*.cpp` you add after that.

--- 

# Running
To run this with ros you can just use `rosrun uav_ros {executable}`