package com.team254.frc2018;

import com.team254.frc2018.auto.AutoModeExecuter;
import com.team254.frc2018.auto.modes.CharacterizeDrivetrainMode;
import com.team254.frc2018.loops.Looper;
import com.team254.frc2018.loops.RobotStateEstimator;
import com.team254.frc2018.statemachines.SuperstructureStateMachine;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.subsystems.*;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.CrashTracker;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;

public class Robot extends IterativeRobot {
    private Looper mEnabledLooper = new Looper();
    private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    private IControlBoard mControlBoard = ControlBoard.getInstance();

    private final SubsystemManager mSubsystemManager = new SubsystemManager(
            Arrays.asList(
                    Drive.getInstance(),
                    FollowerWheels.getInstance(),
                    Intake.getInstance(),
                    Superstructure.getInstance(),
                    Infrastructure.getInstance()
            )
    );

    private Drive mDrive = Drive.getInstance();
    private Intake mIntake = Intake.getInstance();
    private Wrist mWrist = Wrist.getInstance();
    private Infrastructure mInfrastructure = Infrastructure.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mEnabledLooper.register(RobotStateEstimator.getInstance());

            Wrist.getInstance().zeroSensors();
            Elevator.getInstance().zeroSensors();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        SmartDashboard.putString("Match Cycle", "DISABLED");

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
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        try {
            CrashTracker.logAutoInit();

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());


            AutoModeExecuter mAutoModeExecuter = new AutoModeExecuter();
            mAutoModeExecuter.setAutoMode(new CharacterizeDrivetrainMode());
            mAutoModeExecuter.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        SmartDashboard.putString("Match Cycle", "TELEOP");

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
        SmartDashboard.putString("Match Cycle", "TEST");

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
        SmartDashboard.putString("Match Cycle", "DISABLED");

        try {
            outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        try {

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopPeriodic() {
        SmartDashboard.putString("Match Cycle", "TELEOP");

        try {
            double throttle = mControlBoard.getThrottle();
            double turn = mControlBoard.getTurn();

            mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn(),
                    !mControlBoard.getLowGear()));
            // mDrive.setHighGear(!mControlBoard.getLowGear());

            if (mControlBoard.getScore()) {
                mSuperstructure.setWantedAction(SuperstructureStateMachine.WantedAction.PLACE);
            } else if (mControlBoard.getFarScore()) {
                mSuperstructure.setWantedAction(SuperstructureStateMachine.WantedAction.SHOOT);
            } else if (mControlBoard.getIntake()) {
                mSuperstructure.setWantedAction(SuperstructureStateMachine.WantedAction.INTAKE);
            } else if (mControlBoard.getSwitch()) {
                if (mControlBoard.getBackwardsModifier()) {
                    mSuperstructure.setScoringPosition(
                            SuperstructureConstants.ScoringPositionID.SWITCH_BACKWARDS);
                } else {
                    mSuperstructure.setScoringPosition(
                            SuperstructureConstants.ScoringPositionID.SWITCH);
                }
            } else if (mControlBoard.getHighScale()) {
                if (mControlBoard.getBackwardsModifier()) {
                    mSuperstructure.setScoringPosition(
                            SuperstructureConstants.ScoringPositionID.SCALE_HIGH_BACKWARDS);
                } else {
                    mSuperstructure.setScoringPosition(
                            SuperstructureConstants.ScoringPositionID.SCALE_HIGH);
                }
            } else if (mControlBoard.getNeutralScale()) {
                if (mControlBoard.getBackwardsModifier()) {
                    mSuperstructure.setScoringPosition(
                            SuperstructureConstants.ScoringPositionID.SCALE_NEUTRAL_BACKWARDS);
                } else {
                    mSuperstructure.setScoringPosition(
                            SuperstructureConstants.ScoringPositionID.SCALE_NEUTRAL);
                }
            } else if (mControlBoard.getLowScale()) {
                if (mControlBoard.getBackwardsModifier()) {
                    mSuperstructure.setScoringPosition(
                            SuperstructureConstants.ScoringPositionID.SCALE_LOW_BACKWARDS);
                } else {
                    mSuperstructure.setScoringPosition(
                            SuperstructureConstants.ScoringPositionID.SCALE_LOW);
                }
            } else if (mControlBoard.getJogElevatorDown()) {
                mSuperstructure.setJogDown();
            } else if (mControlBoard.getJogElevatorUp()) {
                mSuperstructure.setJogUp();
            } else if (mControlBoard.getArmIn()) {
                mSuperstructure.setArmIn();
            } else if (mControlBoard.getStow()) {
                mSuperstructure.setWantedAction(SuperstructureStateMachine.WantedAction.STOW);
            } else {
                mSuperstructure.setWantedAction(SuperstructureStateMachine.WantedAction.IDLE);
            }


            outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {
        SmartDashboard.putString("Match Cycle", "TEST");
    }

    public void outputToSmartDashboard() {
        RobotState.getInstance().outputToSmartDashboard();
        FollowerWheels.getInstance().outputToSmartDashboard();
        Drive.getInstance().outputToSmartDashboard();
        Wrist.getInstance().outputToSmartDashboard();
        Intake.getInstance().outputToSmartDashboard();
        Elevator.getInstance().outputToSmartDashboard();
        Infrastructure.getInstance().outputToSmartDashboard();
        mEnabledLooper.outputToSmartDashboard();
        // SmartDashboard.updateValues();
    }
}