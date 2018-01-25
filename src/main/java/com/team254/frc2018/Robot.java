package com.team254.frc2018;

import com.team254.frc2018.lidar.UDPLidar;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot {

    private UDPLidar mUDPLidar;
    private Runtime mRuntime;
    private Process mChezyLidar;

    @Override
    public void robotInit() {
        mRuntime = Runtime.getRuntime();

        try {
            mChezyLidar = mRuntime.exec("/home/root/chezy_lidar");
            // mChezyLidar.waitFor();
        } catch (Exception e) {
            e.printStackTrace();
        }

        mUDPLidar = new UDPLidar();
    }

    @Override
    public void disabledInit() {
        mChezyLidar.destroy();
    }

    @Override
    public void autonomousInit() { }

    @Override
    public void teleopInit() {
        try {
            mChezyLidar = mRuntime.exec("/home/root/chezy_lidar");
            mChezyLidar.waitFor();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}