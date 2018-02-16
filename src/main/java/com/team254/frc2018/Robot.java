package com.team254.frc2018;

import com.team254.frc2018.loops.Looper;
import com.team254.frc2018.loops.RobotStateEstimator;
import com.team254.frc2018.subsystems.Drive;
import com.team254.frc2018.subsystems.FollowerWheels;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.CrashTracker;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;

import java.util.Arrays;

public class Robot extends IterativeRobot {
    private Looper mEnabledLooper = new Looper();
    private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    private IControlBoard mControlBoard = ControlBoard.getInstance();

    private final SubsystemManager mSubsystemManager = new SubsystemManager(
            Arrays.asList(
                    Drive.getInstance(),
                    FollowerWheels.getInstance()
            )
    );

    private Drive mDrive = Drive.getInstance();

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mEnabledLooper.register(RobotStateEstimator.getInstance());
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();

            Drive.getInstance().zeroSensors();
            FollowerWheels.getInstance().zeroSensors();
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            mEnabledLooper.stop();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();
            FollowerWheels.getInstance().zeroSensors();

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mEnabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        try {
            System.out.println("Starting check systems.");
            mDrive.checkSystem();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        try {
            outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        try {

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopPeriodic() {
        try {
            double throttle = mControlBoard.getThrottle();
            double turn = mControlBoard.getTurn();

            mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn(),
                    !mControlBoard.getLowGear()));
            mDrive.setHighGear(!mControlBoard.getLowGear());

            outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {
    }

    public void outputToSmartDashboard() {
        RobotState.getInstance().outputToSmartDashboard();
        FollowerWheels.getInstance().outputToSmartDashboard();
        Drive.getInstance().outputToSmartDashboard();
    }
}