package com.team254.frc2018;

import com.team254.frc2018.lidar.LidarProcessor;
import com.team254.frc2018.lidar.LidarServer;
import com.team254.frc2018.loops.Looper;
import com.team254.frc2018.subsystems.Drive;
import com.team254.lib.util.CheesyDriveHelper;
import edu.wpi.first.wpilibj.IterativeRobot;

import java.util.Arrays;

public class Robot extends IterativeRobot {
//    private LidarProcessor mLidar = LidarProcessor.getInstance();
//    private LidarServer mLidarServer = LidarServer.getInstance();

    private Looper mEnabledLooper = new Looper();
    private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    private IControlBoard mControlBoard = ControlBoard.getInstance();
//
    private final SubsystemManager mSubsystemManager = new SubsystemManager(
            Arrays.asList(Drive.getInstance()));
//
    private Drive mDrive = Drive.getInstance();



    @Override
    public void robotInit() {
        mSubsystemManager.registerEnabledLoops(mEnabledLooper);
//        mEnabledLooper.register(LidarProcessor.getInstance());
    }

    @Override
    public void disabledInit() {
//        mLidarServer.stop();
        mEnabledLooper.stop();
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void teleopInit() {
//        mLidarServer.start();
        mEnabledLooper.start();
    }

    @Override
    public void testInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
        double throttle = mControlBoard.getThrottle();
        double turn = mControlBoard.getTurn();

        mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn(),
                !mControlBoard.getLowGear()));
        mDrive.setHighGear(!mControlBoard.getLowGear());
    }

    @Override
    public void testPeriodic() {
    }

}