package com.team254.frc2018;

import edu.wpi.first.wpilibj.Solenoid;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

/**
 * A list of constants used by the rest of the robot code. This include physics constants as well as constants
 * determined through calibrations.
 */
public class Constants {
    public static final double kLooperDt = 0.01;

    /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    public static final double kFollowerWheelTrackWidthInches = 25.624;
    public static final double kFollowerWheelBackOffset = 14.19;
    public static final double kFollowerWheelDiameterInches = 2.28;
    public static final double kFollowerWheelDiameterInchesForwards = 2.26317884466;
    public static final double kFollowerWheelDiameterInchesReverse = 2.34640434375;
    public static final double kDriveWheelTrackWidthInches = 26.0;  // TODO: measure
    public static final double kDriveWheelDiameterInches = 4; // TODO: measure
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0; // TODO: measure
    public static final double kDriveWheelDiameterInchesForwards = 3.82694495937;
    public static final double kDriveWheelDiameterInchesReverse = 3.94000307575;
    public static final double kTrackScrubFactor = 1.0;  // Tune me!

    // Tuned dynamics
    public static final double kRobotLinearInertia = 60.0;  // kg TODO tune
    public static final double kRobotAngularInertia = 80.0;  // kg m TODO tune
    public static final double kDriveVIntercept = 0.9;  // V
    public static final double kDriveKv = 0.15;  // V per rad/s
    public static final double kDriveKa = 0.02;  // V per rad/s^2

    // Geometry
    public static final double kCenterToFrontBumperDistance = 16.33;
    public static final double kCenterToIntakeDistance = 23.11;
    public static final double kCenterToRearBumperDistance = 16.99;
    public static final double kCenterToSideBumperDistance = 17.225;

    // Pose of the LIDAR frame w.r.t. the robot frame
    // TUNE ME
    public static final double kLidarXOffset = -3.3211;
    public static final double kLidarYOffset = 0.0;
    public static final double kLidarYawAngleDegrees = 0.0;

    /* LIDAR CONSTANTS */
    public static final int kChezyLidarScanSize = 400;
    public static final int kChezyLidarNumScansToStore = 10;
    public static final String kChezyLidarPath = "/home/root/chezy_lidar";
    public static final double kChezyLidarRestartTime = 2.5;

    /* CONTROL LOOP GAINS */

    // Gearing and mechanical constants.
    public static final double kDriveDownShiftVelocity = 9.5 * 12.0;  // inches per second
    public static final double kDriveDownShiftAngularVelocity = Math.PI / 2.0; // rad/sec
    public static final double kDriveUpShiftVelocity = 11.0 * 12.0;  // inches per second

    // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static final double kDriveHighGearVelocityKp = 1.2;
    public static final double kDriveHighGearVelocityKi = 0.0;
    public static final double kDriveHighGearVelocityKd = 6.0;
    public static final double kDriveHighGearVelocityKf = .15;
    public static final int kDriveHighGearVelocityIZone = 0;
    public static final double kDriveHighGearVelocityRampRate = 240.0;
    public static final double kDriveHighGearMaxSetpoint = 17.0 * 12.0; // 17 fps

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static final double kDriveLowGearPositionKp = 1.0;
    public static final double kDriveLowGearPositionKi = 0.002;
    public static final double kDriveLowGearPositionKd = 100.0;
    public static final double kDriveLowGearPositionKf = .45;
    public static final int kDriveLowGearPositionIZone = 700;
    public static final double kDriveLowGearPositionRampRate = 240.0; // V/s
    public static final double kDriveLowGearMaxVelocity = 6.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches);
    // 6 fps
    // in RPM
    public static final double kDriveLowGearMaxAccel = 18.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); //
    // 18 fps/s
    // in RPM/s

    // PID gains for elevator velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in native units per 100ms.
    // Elevator encoder is CTRE mag encoder which is 4096 native units per revolution.
    public static final double kElevatorHighGearKp =  0.18;//0.15;
    public static final double kElevatorHighGearKi = 0.0;
    public static final double kElevatorHighGearKd = 0; //0.1;
    public static final double kElevatorHighGearKf = 0.082; // lower speed:  0.08;
    public static final int kElevatorHighGearMaxIntegralAccumulator = 500000; //todo: tune me
    public static final int kElevatorHighGearIZone = 0;
    public static final int kElevatorHighGearDeadband = 0;
    public static final int kElevatorHighGearCruiseVelocity = 12500;
    public static final int kElevatorHighGearAcceleration = 33000;//33000;
    public static final double kElevatorRampRate = 0.1;

    public static final double kWristKp = 1.7; //todo: tune me
    public static final double kWristKi = 0.0; //todo: tune me
    public static final double kWristKd = 40.0; //todo: tune me
    public static final double kWristKf = 1.7; //todo: tune me
    public static final int kWristMaxIntegralAccumulator = 500000; //todo: tune me
    public static final int kWristIZone = 500; //todo: tune me
    public static final int kWristDeadband = 25; //todo: tune me
    public static final int kWristCruiseVelocity = 650; //todo: tune me
    public static final int kWristAcceleration = 700; //todo: tune me
    public static final double kWristRampRate = 0.1;

    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/12_Mrd6xKmxCjKtsWNpWZDqT7ukrB9-1KKFCuRrO4aPM/edit#gid=0

    /* I/O */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)

    public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; //use for constructors

    // Drive
    public static final int kLeftDriveMasterId = 5;
    public static final int kLeftDriveSlaveAId = 6;
    public static final int kLeftDriveSlaveBId = 7;
    public static final int kRightDriveMasterId = 12;
    public static final int kRightDriveSlaveAId = 13;
    public static final int kRightDriveSlaveBId = 14;

    // Followers
    public static final int kFollowerLeftAChannelId = 2;
    public static final int kFollowerLeftBChannelId = 3;
    public static final int kFollowerRightAChannelId = 0;
    public static final int kFollowerRightBChannelId = 1;
    public static final int kFollowerRearAChannelId = 4;
    public static final int kFollowerRearBChannelId = 5;


    // Intake
    public static final int kIntakeLeftMasterId = 9;
    public static final int kIntakeRightMasterId = 10;
    public static final int kCanifierId = 0;

    // Elevator
    public static final int kElevatorMasterId = 11;
    public static final int kElevatorRightSlaveId = 8;
    public static final int kElevatorLeftSlaveAId = 1;
    public static final int kElevatorLeftSlaveBId = 2;

    // Wrist
    public static final int KWristMasterId = 15;

    // Solenoids
    public static final int kShifterSolenoidId = 12; // PCM 0, Solenoid 4
    public static final int kIntakeCloseSolenoid = 10;
    public static final int kIntakeClampSolenoid = 9;
    public static final int kForkliftDeploySolenoid = 7;  // CURRENTLY 6 ON PRACTICE!!!
    public static final int kHangerReleaseSolenoid = 6;
    public static final int kFollowerWheelSolenoid = 11;
    public static final int kElevatorShifterSolenoidId = 8;

    // Sensors
    public static final int kIntakeLeftBannerId = 9; //todo: get actual value
    public static final int kIntakeRightBannerId = 10; //todo: get actual value

    // Control Board
    public static final boolean kUseGamepadForDriving = false;
    public static final boolean kUseGamepadForButtons = true;

    public static final int kDriveGamepadPort = 0;
    public static final int kButtonGamepadPort = 2;
    public static final int kMainThrottleJoystickPort = 0;
    public static final int kMainTurnJoystickPort = 1;
    public static final double kJoystickThreshold = 0.5;

    // Height in in after applying turn factor.
    public static final double kElevatorLowSensitivityThreshold = 50.0;
    public static final double kLowSensitivityFactor = 1.0 / 4.0;

    /**
     * Make an {@link Solenoid} instance for the single-number ID of the solenoid
     * <p>
     * Solenoids were wired in an inane method and also not labeled zero indexed.
     * <p>
     * Solenoids 1-4 are on PCM 1, Solenoids 7-4.
     * Solenoids 5-8 are on PCM 0, Solenoids 0-3.
     * Solenoids 9-12 are on PCM 0, Solenoids 7-4.
     *
     * @param solenoidId One of the kXyzSolenoidId constants
     */
    public static Solenoid makeSolenoidForId(int solenoidId) {
        if (solenoidId <= 4) {
            // These solenoids are on PCM 1, wired 1-4 to 7-4.
            return new Solenoid(1, 8 - solenoidId);
        } else if (solenoidId <= 8) {
            // These solenoids are on PCM 0, wired 5-8 to 0-3.
            return new Solenoid(0, solenoidId - 5);
        } else if (solenoidId <= 12) {
            // These solenoids are on PCM 0, wired 9-12 to 7-4.
            return new Solenoid(0, 16 - solenoidId);
        }
        throw new IllegalArgumentException("Solenoid ID not valid: " + solenoidId);
    }

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        return "";
    }
}
