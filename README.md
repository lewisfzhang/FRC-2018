# FRC 2018

Team 254's 2018 FRC robot code for Lockdown. Lockdown's code is written in Java and is based off of WPILib's Java control system.

The code is divided into several packages, each responsible for a different aspect of the robot function. This README explains the function of each package, some of the variable naming conventions used, and setup instructions. Additional information about each specific class can be found in that class' Java file.

## Setup Instructions

### General
- Clone this repo
- Run `./gradlew` to download gradle and needed FRC libraries
- Run `./gradlew tasks` to see available build options
- Enjoy!

### Eclipse
- Run `./gradlew eclipse`
- Open Eclipse and go to File > Open Projects from File System...
- Set the import source to the FRC-2018 folder then click finish

### IntelliJ
- Run `./gradlew idea`
- Open the FRC-2018.ipr file with IntelliJ

### Building/Deploying to the Robot
- Run `./gradlew deploy` to deploy to the robot in Terminal (Mac) or Powershell (Windows)
- Run `./gradlew build` to build the code.  Use the `--info` flag for more details

## Code Highlights
- Building with Gradle

	Instead of working with ant, we used GradleRIO, which is a powerful plugin for Gradle that allows us to build and deploy our code for FRC. It automatically fetches WPILib, CTRE Toolsuite, and other libraries and is easier to use in different IDEs. 

- Path following with a pure pursuit controller and splines

	To control autonomous driving, the robot utilizes an [pure pursuit controller](src/main/java/com/team254/lib/trajectory/PurePursuitController.java) to control steering when driving paths constructed of [Quintic Hermite Splines](src/main/java/com/team254/lib/spline/QuinticHermiteSpline.java).

- Path generation and visualization via Java app

	[Cheesy Path](cheesy-path), an external Java webapp, is used for fast and simple path creation. The app allows a user to create and visualize autonomous paths.

- Self-test modes for each subsystem

	Each subsystem contains a [checkSystem()](src/main/java/com/team254/frc2018/subsystems/Drive.java#L464) method that tests motor output currents and RPMs.  These self tests allow us to quickly verify that all motors are working properly.

- Scale Detection

	[Cheesy Vision 2.0](dash/CheesyVision2.py) is a Python app that uses OpenCV to track the angle of the scale. This allows us to set our elevator to the right height during autonomous, based on the actual tip of the scale.

## Package Functions
- com.team254.frc2018

	Contains the robot's central functions and holds a file with all numerical constants used throughout the code. For example, the Robot member class controls all routines depending on the robot state.

- com.team254.frc2018.auto

	Handles the excecution of autonomous routines.  Also contains the auto actions and auto modes packages.
	
- com.team254.frc2018.auto.actions

	Contains all actions used during the autonomous period, which all share a common interface, Action (also in this package). Examples include shooting cubes, driving a trajectory, or moving the elevator. Routines interact with the Subsystems, which interact with the hardware.

- com.team254.frc2018.auto.creators

	Contains all the auto mode creators, that select the correct auto mode to run based on user inputted and FMS data.
	
- com.team254.frc2018.auto.modes
	
	Contains all autonomous modes. Autonomous modes consist of a list of autonomous actions excecuted in a certain order.

- com.team254.frc2018.auto.controlboard
	
	Contains all the code for the different control boards. This allows you to use any combination of driverstation joysticks and button board, and Xbox Controllers for both driving and operating. These are controlled by booleans in Constants.java.
	
- com.team254.frc2018.loops

	Loops are routines that run periodically on the robot, such as calculating robot pose, processing vision feedback, or updating subsystems. All Loops implement the Loop interface and are handled (started, stopped, added) by the Looper class, which runs att 200 Hz.
	The Robot class has one main Looper, mEnabledLooper, that runs all loops when the robot is enabled.

- com.team254.frc2018.planners

	Loops are routines that run periodically on the robot, such as calculating robot pose, processing vision feedback, or updating subsystems. All Loops implement the Loop interface and are handled (started, stopped, added) by the Looper class, which runs att 200 Hz.
	The Robot class has one main Looper, mEnabledLooper, that runs all loops when the robot is enabled.
	
- com.team254.frc2018.paths

    Contains the generator for all of the trajectories that the robot drives during autonomous period.

- com.team254.frc2018.statemachines

    Contains the state machines for the Intake and overall Superstructure.

- com.team254.frc2018.states

    Contains states and other classes used in the subsystem and statemachine classes.

- com.team254.frc2018.subsystems
	
	Subsystems are consolidated into one central class per subsystem, all of which implement the Subsystem abstract class. Each Subsystem uses state machines for control.
	Each Subsystem is also a singleton, meaning that there is only one instance of each Subsystem class. To modify a subsystem, one would get the instance of the subsystem and change its desired state. The Subsystem class will work on setting the desired state.
	
- com.team254.lib.drivers

    Contains a set of custom classes for TalonSRX's.
	
- com.team254.lib.geometry

    Contains a set of classes that represent various geometric entities.
	
- com.team254.lib.physics

    Contains classes that model DC motor transmissions and differential drive characterization.

- com.team254.lib.spline

    Contains the code for generating and optimizing splines.

- com.team254.lib.trajectory

    Contains classes for following and storing trajectories.

- com.team254.lib.util

    A collection of assorted utilities classes used in the robot code. Check each .java file for more information.
	
## Variable Naming Conventions
- k*** (i.e. kDriveWheelTrackWidthInches): Final constants, especialy those found in the Constants.java file
- m***  (i.e. mIsHighGear): Private instance variables
