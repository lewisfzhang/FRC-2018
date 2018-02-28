package com.team254.frc2018.subsystems;

import com.team254.frc2018.Robot;
import com.team254.frc2018.loops.ILooper;
import com.team254.frc2018.loops.Loop;
import com.team254.frc2018.statemachines.SuperstructureStateMachine;
import com.team254.frc2018.states.IntakeState;
import com.team254.frc2018.states.SuperstructureCommand;
import com.team254.frc2018.states.SuperstructureState;

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
    private SuperstructureState mState = new SuperstructureState();
    private Elevator mElevator = Elevator.getInstance();
    private Wrist mWrist = Wrist.getInstance();
    private Intake mIntake = Intake.getInstance();
    private SuperstructureStateMachine mStateMachine = new SuperstructureStateMachine();
    private SuperstructureStateMachine.WantedAction mWantedAction =
            SuperstructureStateMachine.WantedAction.IDLE;

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

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

    public synchronized SuperstructureStateMachine.SystemState getSuperStructureState() {
        return mStateMachine.getSystemState();
    }

    private synchronized void updateObservedState(SuperstructureState state) {
        state.height = mElevator.getInchesOffGround();
        state.angle = mWrist.getAngle();
        state.jawClamped = mIntake.getJawState() == IntakeState.JawState.CLAMPED;

        state.elevatorSentLastTrajectory = mElevator.hasFinishedTrajectory();
        state.wristSentLastTrajectory = mWrist.hasFinishedTrajectory();
    }

    // Update subsystems from planner
    synchronized void setFromCommandState(SuperstructureCommand commandState) {
        if (commandState.openLoopElevator) {
            mElevator.setOpenLoop(commandState.openLoopElevatorPercent);
        } else {
            mElevator.setClosedLoopPosition(commandState.height);
        }
        if (commandState.elevatorLowGear) {
            mElevator.setHangMode(true);
        } else {
            mElevator.setHangMode(false);
        }
        mWrist.setClosedLoopAngle(commandState.wristAngle);
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            private SuperstructureCommand mCommand;

            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    updateObservedState(mState);
                    mCommand = mStateMachine.update(timestamp, mWantedAction, mState);
                    setFromCommandState(mCommand);
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    public synchronized double getScoringAngle() {
        return mStateMachine.getScoringAngle();
    }

    public synchronized double getScoringHeight() {
        return mStateMachine.getScoringHeight();
    }

    public synchronized void setDesiredHeight(double height) {
        mStateMachine.setScoringHeight(height);
        mWantedAction = SuperstructureStateMachine.WantedAction.GO_TO_POSITION;
    }

    public synchronized void setDesiredAngle(double angle) {
        mStateMachine.setScoringAngle(angle);
        mWantedAction = SuperstructureStateMachine.WantedAction.GO_TO_POSITION;
    }

    public synchronized void setElevatorJog(double relative_inches) {
        mStateMachine.jogElevator(relative_inches);
        mWantedAction = SuperstructureStateMachine.WantedAction.GO_TO_POSITION;
    }

    public synchronized void setWristJog(double relative_degrees) {
        mStateMachine.jogWrist(relative_degrees);
        mWantedAction = SuperstructureStateMachine.WantedAction.GO_TO_POSITION;
    }


    public synchronized void setHangThrottle(double throttle) {
        mStateMachine.setOpenLoopPower(throttle);
        mWantedAction = SuperstructureStateMachine.WantedAction.HANG;
    }

    public synchronized void setWantedAction(SuperstructureStateMachine.WantedAction wantedAction) {
        mWantedAction = wantedAction;
    }
}
