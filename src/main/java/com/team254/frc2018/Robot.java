package com.team254.frc2018;

import com.team254.frc2018.lidar.LidarProcessor;
import com.team254.frc2018.lidar.LidarServer;
import com.team254.frc2018.loops.Looper;
import edu.wpi.first.wpilibj.IterativeRobot;

public class Robot extends IterativeRobot {
    private LidarProcessor mLidar = LidarProcessor.getInstance();
    private LidarServer mLidarServer = LidarServer.getInstance();

    private Looper mEnabledLooper = new Looper();

    @Override
    public void robotInit() {
        mEnabledLooper.register(LidarProcessor.getInstance());
    }

    @Override
    public void disabledInit() {
        mLidarServer.stop();
        mEnabledLooper.stop();
    }

    @Override
    public void autonomousInit() { }

    @Override
    public void teleopInit() {
        mLidarServer.start();
        mEnabledLooper.start();
    }

    @Override
    public void testInit() { }

    @Override
    public void disabledPeriodic() { }
    
    @Override
    public void autonomousPeriodic() { }

    @Override
    public void teleopPeriodic() {
//        System.out.println(mLidarServer.isLidarConnected());
    }

    @Override
    public void testPeriodic() { }

}