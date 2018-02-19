package com.team254.frc2018.subsystems;

import com.team254.frc2018.Robot;
import com.team254.frc2018.loops.Loop;
import com.team254.frc2018.loops.Looper;
import com.team254.frc2018.planners.SuperstructureMotionPlanner;
import com.team254.frc2018.statemachines.IntakeStateMachine;
import com.team254.frc2018.statemachines.SuperstructureStateMachine;
import com.team254.frc2018.states.IntakeState;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.states.SuperstructureState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The superstructure subsystem is the overarching superclass containing all components of the superstructure: the
 * intake, hopper, feeder, shooter and LEDs. The superstructure subsystem also contains some miscellaneous hardware that
 * is located in the superstructure but isn't part of any other subsystems like the compressor, pressure sensor, and
 * hopper wall pistons.
 * <p>
 * Instead of interacting with subsystems like the feeder and intake directly, the {@link Robot} class interacts with
 * the superstructure, which passes on the commands to the correct subsystem.
 * <p>
 * The superstructure also coordinates actions between different subsystems like the feeder and shooter.
 *
 * @see Subsystem
 */
public class Superstructure extends Subsystem {

    static Superstructure mInstance = null;

    public static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

    private SuperstructureState mState = new SuperstructureState();
    private Elevator mElevator = Elevator.getInstance();
    private Wrist mWrist = Wrist.getInstance();
    private Intake mIntake = Intake.getInstance();

    private SuperstructureStateMachine mStateMachine = new SuperstructureStateMachine();
    private SuperstructureStateMachine.WantedAction mWantedAction =
            SuperstructureStateMachine.WantedAction.IDLE;

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputToSmartDashboard() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {

    }

    private synchronized void updateObservedState(SuperstructureState state) {
        state.height = mElevator.getInchesOffGround();
        state.angle = mWrist.getAngle();
        state.jawClamped = mIntake.getJawState() == IntakeState.JawState.CLAMPED;
        state.hasCube = mIntake.hasCube();

        state.elevatorSentLastTrajectory = mElevator.hasFinishedTrajectory();
        state.wristSentLastTrajectory = mWrist.hasFinishedTrajectory();
    }

    // Update subsystems from planner
    synchronized void setFromCommandState(SuperstructureState commandState) {
        mElevator.setClosedLoopPosition(commandState.height);
        mWrist.setClosedLoopAngle(commandState.angle);

        mIntake.setState(commandState.intakeAction);
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    updateObservedState(mState);
                    SuperstructureState commandState =
                            mStateMachine.update(timestamp, mWantedAction, mState);
                    setFromCommandState(commandState);
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    public synchronized void setScoringPosition(SuperstructureConstants.ScoringPositionID position_id) {
        SuperstructureConstants.ScoringPosition position =
                SuperstructureConstants.kScoringPositions.get(position_id);
        mStateMachine.setScoringPosition(position.height, position.angle);
        mWantedAction = SuperstructureStateMachine.WantedAction.GOTO_SCORE_POSITION;
    }

    public synchronized void setArmIn() {
        double angle = 0.0;
        if (mState.height >= SuperstructureConstants.kClearFirstStageMaxHeight) {
            angle = SuperstructureConstants.kClearFirstStageMinWristAngle;
        }

        mStateMachine.setScoringAngle(angle);
        mWantedAction = SuperstructureStateMachine.WantedAction.GOTO_SCORE_POSITION;
    }

    public synchronized void setJogPosition(double amount) {
        mStateMachine.setScoringHeight(mState.height + amount);
        mWantedAction = SuperstructureStateMachine.WantedAction.GOTO_SCORE_POSITION;
    }

    public synchronized void setWantedAction(SuperstructureStateMachine.WantedAction wantedAction) {
        mWantedAction = wantedAction;
    }
}
