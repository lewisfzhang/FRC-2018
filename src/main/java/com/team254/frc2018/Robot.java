package com.team254.frc2018;

import com.team254.frc2018.auto.AutoModeExecuter;
import com.team254.frc2018.auto.modes.CharacterizeHighGearStraight;
import com.team254.frc2018.loops.Looper;
import com.team254.frc2018.loops.RobotStateEstimator;
import com.team254.frc2018.statemachines.IntakeStateMachine;
import com.team254.frc2018.statemachines.SuperstructureStateMachine;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.subsystems.*;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.LatchedBoolean;
import edu.wpi.first.wpilibj.IterativeRobot;
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
    private Elevator mElevator = Elevator.getInstance();

    private LatchedBoolean mRunIntakeReleased = new LatchedBoolean();
    private LatchedBoolean mShootReleased = new LatchedBoolean();

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
            mAutoModeExecuter.setAutoMode(new CharacterizeHighGearStraight());
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
            mIntake.checkSystem();
            mWrist.checkSystem();
            mElevator.checkSystem();

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
            mWrist.resetIfAtLimit();
            mElevator.resetIfAtLimit();
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
                    mDrive.isHighGear()));

            // Manual jaw inputs.
            if (mControlBoard.getOpenJaw()) {
                mIntake.tryOpenJaw();
            } else {
                mIntake.clampJaw();
            }

            // Intaking.
            boolean runIntake = mControlBoard.getRunIntake();
            boolean shoot = mControlBoard.getShoot();
            boolean runIntakeReleased = mRunIntakeReleased.update(!runIntake);
            boolean shootReleased = mShootReleased.update(!shoot);
            if (runIntake) {
                mIntake.getOrKeepCube();
            } else if (shoot) {
                mIntake.shoot();
            } else if (runIntakeReleased || shootReleased) {
                mIntake.setState(IntakeStateMachine.WantedAction.WANT_MANUAL);
                mIntake.setPower(0.0);
            }

            // Rumble
            if (mControlBoard.getRunIntake() && mIntake.getLeftBannerSensor()) {
                mControlBoard.setRumble(true);
            } else {
                mControlBoard.setRumble(false);
            }

            // Presets.
            double desired_height = Double.NaN;
            double desired_angle = Double.NaN;

            // Elevator.
            if (mControlBoard.getGoToHighScaleHeight() && !mControlBoard.getRunIntake()) {
                desired_height = SuperstructureConstants.getHeight(SuperstructureConstants.SuperstructurePositionID.SCALE_HIGH);
            } else if (mControlBoard.getGoToNeutralScaleHeight() && !mControlBoard.getRunIntake()) {
                desired_height = SuperstructureConstants.getHeight(SuperstructureConstants.SuperstructurePositionID.SCALE_NEUTRAL);
            } else if (mControlBoard.getGoToLowScaleHeight() && !mControlBoard.getRunIntake()) {
                desired_height = SuperstructureConstants.getHeight(SuperstructureConstants.SuperstructurePositionID.SCALE_LOW);
            } else if (mControlBoard.getGoToHighScaleHeight() && mControlBoard.getRunIntake()) {
                desired_height = SuperstructureConstants.getHeight(SuperstructureConstants.SuperstructurePositionID.INTAKE_THIRD_LEVEL);
            } else if (mControlBoard.getGoToNeutralScaleHeight() && mControlBoard.getRunIntake()) {
                desired_height = SuperstructureConstants.getHeight(SuperstructureConstants.SuperstructurePositionID.INTAKE_SECOND_LEVEL);
            } else if (mControlBoard.getGoToLowScaleHeight() && mControlBoard.getRunIntake()) {
                desired_height = SuperstructureConstants.getHeight(SuperstructureConstants.SuperstructurePositionID.INTAKE_FLOOR_LEVEL);
            }  else if (mControlBoard.getGoToSwitchHeight()) {
                desired_height = SuperstructureConstants.getHeight(SuperstructureConstants.SuperstructurePositionID.SWITCH);
            } else if (mControlBoard.getGoToStowHeight()) {
                desired_height = SuperstructureConstants.getHeight(SuperstructureConstants.SuperstructurePositionID.STOW);
            } else if (mControlBoard.getHangMode()) {
                mSuperstructure.setHangThrottle(mControlBoard.getHangThrottle());
            }

            // Wrist.
            if (mControlBoard.goToStowWrist()) {
                desired_angle = SuperstructureConstants.getAngle(SuperstructureConstants.SuperstructurePositionID.STOW);
            } else if (mControlBoard.goToIntakingWrist()) {
                desired_angle = SuperstructureConstants.getAngle(SuperstructureConstants.SuperstructurePositionID.INTAKE);
            } else if (mControlBoard.goToVerticalWrist()) {
                desired_angle = SuperstructureConstants.getAngle(SuperstructureConstants.SuperstructurePositionID.VERTICAL);
            } else if (mControlBoard.goToScoringWrist()) {
                desired_angle = SuperstructureConstants.getAngle(SuperstructureConstants.SuperstructurePositionID.SCALE_HIGH_BACKWARDS);
            }

            if (Double.isNaN(desired_angle) && Double.isNaN(desired_height)) {
                mSuperstructure.setWantedAction(SuperstructureStateMachine.WantedAction.IDLE);
            } else if (Double.isNaN(desired_angle)) {
                mSuperstructure.setDesiredHeight(desired_height);
            } else if (Double.isNaN(desired_height)) {
                mSuperstructure.setDesiredAngle(desired_angle);
            }

            // TODO jogging
            if (mControlBoard.getJogElevatorUp()) {
                mSuperstructure.setElevatorJog(24. / 50.);
            } else if (mControlBoard.getJogElevatorDown()) {
                mSuperstructure.setElevatorJog(-24. / 50.);
            }

            if (mControlBoard.getJogWristForward()) {
                mSuperstructure.setWristJog(90. / 50.);
            } else if (mControlBoard.getJogWristBack()) {
                mSuperstructure.setWristJog(-90. / 50.);
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