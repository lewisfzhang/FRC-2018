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
    public static double kLooperDt = 0.005;

    /* LIDAR CONSTANTS */
    public static int kScanSize = 400;
    public static int kNumScansToStore = 10;
    public static String kChezyLidarPath = "/home/root/chezy_lidar";
    public static double kLidarRestartTime = 2.5;
    //define a rectangle around the LIDAR at which points should be ignored
    public static double kIgnorePointXMin = -20.0;
    public static double kIgnorePointXMax = 20.0;
    public static double kIgnorePointYMin = -25.0;
    public static double kIgnorePointYMax = 25.0;

    /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    public static double kDriveWheelDiameterInches = 3.419;
    public static double kTrackWidthInches = 26.655;
    public static double kTrackScrubFactor = 0.924;

    // Geometry
    public static double kCenterToFrontBumperDistance = 16.33;
    public static double kCenterToIntakeDistance = 23.11;
    public static double kCenterToRearBumperDistance = 16.99;
    public static double kCenterToSideBumperDistance = 17.225;

    public static double kCenterToBackFollowerWheel = 16.99; // Tune me!

    /* CONTROL LOOP GAINS */

    // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static double kDriveHighGearVelocityKp = 1.2;
    public static double kDriveHighGearVelocityKi = 0.0;
    public static double kDriveHighGearVelocityKd = 6.0;
    public static double kDriveHighGearVelocityKf = .15;
    public static int kDriveHighGearVelocityIZone = 0;
    public static double kDriveHighGearVelocityRampRate = 240.0;
    public static double kDriveHighGearNominalOutput = 0.5;
    public static double kDriveHighGearMaxSetpoint = 17.0 * 12.0; // 17 fps

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static double kDriveLowGearPositionKp = 1.0;
    public static double kDriveLowGearPositionKi = 0.002;
    public static double kDriveLowGearPositionKd = 100.0;
    public static double kDriveLowGearPositionKf = .45;
    public static int kDriveLowGearPositionIZone = 700;
    public static double kDriveLowGearPositionRampRate = 240.0; // V/s
    public static double kDriveLowGearNominalOutput = 0.5; // V
    public static double kDriveLowGearMaxVelocity = 6.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 6 fps
    // in RPM
    public static double kDriveLowGearMaxAccel = 18.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 18 fps/s
    // in RPM/s

    public static double kDriveVoltageCompensationRampRate = 0.0;


    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/12_Mrd6xKmxCjKtsWNpWZDqT7ukrB9-1KKFCuRrO4aPM/edit#gid=0

    /* TALONS */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)

    // Drive
    public static final int kLeftDriveMasterId = 5;
    public static final int kLeftDriveSlaveId = 6;
    public static final int kLeftDriveSlave1Id = 7;
    public static final int kRightDriveMasterId = 11;
    public static final int kRightDriveSlaveId = 13;
    public static final int kRightDriveSlave1Id = 14;

    // Solenoids
    public static final int kShifterSolenoidId = 0; // PCM 0, Solenoid 0
    public static final int kIntakeDeploySolenoidId = 1; // PCM 0, Solenoid 1
    public static final int kHopperSolenoidId = 2; // PCM 0, Solenoid 2
    public static final int kGearWristSolenoid = 7; // PCM 0, Solenoid 7

    // Phone
    public static int kAndroidAppTcpPort = 8254;

    // Goal tracker constants
    public static double kMaxGoalTrackAge = 1.0;
    public static double kMaxTrackerDistance = 18.0;
    public static double kCameraFrameRate = 30.0;
    public static double kTrackReportComparatorStablityWeight = 1.0;
    public static double kTrackReportComparatorAgeWeight = 1.0;

    // Pose of the camera frame w.r.t. the robot frame
    public static double kCameraXOffset = -3.3211;
    public static double kCameraYOffset = 0.0;
    public static double kCameraZOffset = 20.9;
    public static double kCameraPitchAngleDegrees = 29.56; // Measured on 4/26
    public static double kCameraYawAngleDegrees = 0.0;
    public static double kCameraDeadband = 0.0;

    /**
     * Make an {@link Solenoid} instance for the single-number ID of the solenoid
     *
     * @param solenoidId One of the kXyzSolenoidId constants
     */
    public static Solenoid makeSolenoidForId(int solenoidId) {
        return new Solenoid(solenoidId / 8, solenoidId % 8);
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
