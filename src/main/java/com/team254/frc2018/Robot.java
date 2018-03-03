package com.team254.frc2018;

import com.team254.frc2018.auto.AutoModeExecuter;
import com.team254.frc2018.auto.AutoModeSelector;
import com.team254.frc2018.loops.Looper;
import com.team254.frc2018.subsystems.RobotStateEstimator;
import com.team254.frc2018.statemachines.IntakeStateMachine;
import com.team254.frc2018.statemachines.SuperstructureStateMachine;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.subsystems.*;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.LatchedBoolean;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;

public class Robot extends IterativeRobot {
    private Looper mEnabledLooper = new Looper();
    private Looper mDisabledLooper = new Looper();
    private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    private IControlBoard mControlBoard = ControlBoard.getInstance();

    private final SubsystemManager mSubsystemManager = new SubsystemManager(
            Arrays.asList(
                    RobotStateEstimator.getInstance(),
                    Drive.getInstance(),
                    FollowerWheels.getInstance(),
                    Superstructure.getInstance(),
                    Intake.getInstance(),
                    Wrist.getInstance(),
                    Elevator.getInstance(),
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
    private LatchedBoolean mRunIntakePressed = new LatchedBoolean();

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            Elevator.getInstance().zeroSensors();

            mRunIntakeReleased.update(true);
            mShootReleased.update(true);

            AutoModeSelector.initAutoModeSelector();
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
            mEnabledLooper.stop();

            Drive.getInstance().zeroSensors();
            FollowerWheels.getInstance().zeroSensors();
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            mDrive.stopLogging();
            mDisabledLooper.start();
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
            mDisabledLooper.stop();

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            Drive.getInstance().zeroSensors();
            FollowerWheels.getInstance().zeroSensors();
            mInfrastructure.setIsDuringAuto(true);

            AutoFieldState.setStartPose(AutoModeSelector.getSelectedStartingPose());

            AutoModeExecuter mAutoModeExecuter = new AutoModeExecuter();
            mAutoModeExecuter.setAutoMode(AutoModeSelector.getSelectedAutoMode());
            mAutoModeExecuter.start();

            mDrive.startLogging();
            mEnabledLooper.start();
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
            mDisabledLooper.stop();

            FollowerWheels.getInstance().zeroSensors();
            mInfrastructure.setIsDuringAuto(false);

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mDrive.stopLogging();
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

            AutoFieldState.setSides(DriverStation.getInstance().getGameSpecificMessage());
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        outputToSmartDashboard();
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

            // When elevator is up, tune sensitivity on tune a little.
            if (mElevator.getInchesOffGround() > Constants.kElevatorLowSensitivityThreshold) {
                turn *= Constants.kLowSensitivityFactor;
            }

            mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn(),
                    mDrive.isHighGear()));

            // Intake/Shoot
            boolean runIntake = mControlBoard.getRunIntake() || mControlBoard.getIntakePosition();
            boolean shoot = mControlBoard.getShoot();
            boolean runIntakeReleased = mRunIntakeReleased.update(!runIntake);
            boolean shootReleased = mShootReleased.update(!shoot);
            boolean intakeAction = false;
            if (runIntake) {
                mIntake.getOrKeepCube();
                intakeAction = true;
            } else if (shoot) {
                intakeAction = true;
                mIntake.shoot();
            } else if (runIntakeReleased || shootReleased) {
                if (mIntake.hasCube()) {
                    mIntake.getOrKeepCube();
                } else {
                    mIntake.setState(IntakeStateMachine.WantedAction.WANT_MANUAL);
                    mIntake.setPower(0.0);
                }
                intakeAction = true;
            }

            // Manual jaw inputs.
            if (mControlBoard.getOpenJaw()) {
                mIntake.tryOpenJaw();
                if (!intakeAction) {
                    mIntake.setState(IntakeStateMachine.WantedAction.WANT_MANUAL);
                    mIntake.setPower(0.0);
                }
            } else {
                mIntake.closeJaw();
            }


            // Rumble
            if (runIntake && mIntake.definitelyHasCube()) {
                mControlBoard.setRumble(true);
            } else {
                mControlBoard.setRumble(false);
            }

            // Presets.
            double desired_height = Double.NaN;
            double desired_angle = Double.NaN;

            if (mControlBoard.getGoToStowHeight()) {
                desired_height = SuperstructureConstants.kStowedPositionHeight;
                desired_angle = SuperstructureConstants.kStowedPositionAngle;
            }

            if (mRunIntakePressed.update(mControlBoard.getIntakePosition())) {
                desired_height = SuperstructureConstants.kIntakePositionHeight;
                desired_angle = SuperstructureConstants.kIntakePositionAngle;
            }

            // Elevator.
            if (mControlBoard.getGoToHighScaleHeight() && !mControlBoard.getIntakePosition()) {
                desired_height = SuperstructureConstants.kScaleHighHeight;
            } else if (mControlBoard.getGoToNeutralScaleHeight() && !mControlBoard.getIntakePosition()) {
                desired_height = SuperstructureConstants.kScaleNeutralHeight;
            } else if (mControlBoard.getGoToLowScaleHeight() && !mControlBoard.getIntakePosition()) {
                desired_height = SuperstructureConstants.kScaleLowHeight;
            } else if (mControlBoard.getGoToHighScaleHeight() && mControlBoard.getIntakePosition()) {
                desired_height = SuperstructureConstants.kIntakeThirdLevelHeight;
            } else if (mControlBoard.getGoToNeutralScaleHeight() && mControlBoard.getIntakePosition()) {
                desired_height = SuperstructureConstants.kIntakeSecondLevelHeight;
            } else if (mControlBoard.getGoToLowScaleHeight() && mControlBoard.getIntakePosition()) {
                desired_height = SuperstructureConstants.kIntakeFloorLevelHeight;
            }  else if (mControlBoard.getGoToSwitchHeight()) {
                desired_height = SuperstructureConstants.kSwitchHeight;
            } else if (mControlBoard.getHangMode()) {
                mSuperstructure.setHangThrottle(mControlBoard.getHangThrottle());
            }

            // Wrist.
            if (mControlBoard.goToStowWrist()) {
                desired_angle = SuperstructureConstants.kStowedPositionAngle;
            } else if (mControlBoard.goToIntakingWrist()) {
                if (mSuperstructure.getScoringHeight() > SuperstructureConstants.kPlacingHighThreshold) {
                    desired_angle = SuperstructureConstants.kPlacingHighAngle;
                } else {
                    desired_angle = SuperstructureConstants.kPlacingLowAngle;
                }
            } else if (mControlBoard.goToVerticalWrist()) {
                desired_angle = SuperstructureConstants.kVerticalAngle;
            } else if (mControlBoard.goToScoringWrist()) {
                desired_angle = SuperstructureConstants.kScoreBackwardsAngle;
            } else if (mControlBoard.goToScoringAngledWrist()) {
                desired_angle = SuperstructureConstants.kScoreForwardAngledAngle;
            }

            // Attempt to fix wrist angle if we will be in an invalid state.
            if (!Double.isNaN(desired_height) && Double.isNaN(desired_angle) &&
                    desired_height > SuperstructureConstants.kClearFirstStageMaxHeight) {
                if (mSuperstructure.getScoringAngle() <
                        SuperstructureConstants.kClearFirstStageMinWristAngle) {
                    desired_angle = SuperstructureConstants.kClearFirstStageMinWristAngle;
                }
            }

            if (Double.isNaN(desired_angle) && Double.isNaN(desired_height)) {
                mSuperstructure.setWantedAction(SuperstructureStateMachine.WantedAction.IDLE);
            } else if (Double.isNaN(desired_angle)) {
                mSuperstructure.setDesiredHeight(desired_height);
            } else if (Double.isNaN(desired_height)) {
                mSuperstructure.setDesiredAngle(desired_angle);
            } else if (!Double.isNaN(desired_angle) && !Double.isNaN(desired_height)) {
                mSuperstructure.setDesiredAngle(desired_angle);
                mSuperstructure.setDesiredHeight(desired_height);
            }

            if (mControlBoard.getJogElevatorUp()) {
                mSuperstructure.setElevatorJog(SuperstructureConstants.kElevatorJogUpThrottle);
            } else if (mControlBoard.getJogElevatorDown()) {
                mSuperstructure.setElevatorJog(SuperstructureConstants.kElevatorJogDownThrottle);
            }

            if (mControlBoard.getJogWristForward()) {
                mSuperstructure.setWristJog(SuperstructureConstants.kWristJogUpThrottle);
            } else if (mControlBoard.getJogWristBack()) {
                mSuperstructure.setWristJog(SuperstructureConstants.kWristJogDownThrottle);
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
        FollowerWheels.getInstance().outputTelemetry();
        Drive.getInstance().outputTelemetry();
        Wrist.getInstance().outputTelemetry();
        Intake.getInstance().outputTelemetry();
        Elevator.getInstance().outputTelemetry();
        Infrastructure.getInstance().outputTelemetry();
        mEnabledLooper.outputToSmartDashboard();
        // SmartDashboard.updateValues();
    }
}