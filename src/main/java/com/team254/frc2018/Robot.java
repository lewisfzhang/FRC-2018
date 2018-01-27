package com.team254.frc2018;

import com.team254.frc2018.lidar.LidarInterface;
import com.team254.frc2018.lidar.LidarServer;
import edu.wpi.first.wpilibj.IterativeRobot;

public class Robot extends IterativeRobot {
    private LidarInterface mLidar = LidarInterface.getInstance();
    private LidarServer mLidarServer = LidarServer.getInstance();

    @Override
    public void robotInit() {
    }

    @Override
    public void disabledInit() {
        mLidarServer.start();
    }

    @Override
    public void autonomousInit() { }

    @Override
    public void teleopInit() {
    }

    @Override
    public void testInit() { }

    @Override
    public void disabledPeriodic() { }
    
    @Override
    public void autonomousPeriodic() { }

    @Override
    public void teleopPeriodic() { }

    @Override
    public void testPeriodic() { }

}