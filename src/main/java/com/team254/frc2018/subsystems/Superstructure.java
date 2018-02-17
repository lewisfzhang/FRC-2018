package com.team254.frc2018.subsystems;

import com.team254.frc2018.Robot;
import com.team254.frc2018.loops.Loop;
import com.team254.frc2018.loops.Looper;
import com.team254.frc2018.planners.SuperstructureMotionPlanner;
import com.team254.frc2018.statemachines.IntakeStateMachine;
import com.team254.frc2018.states.IntakeState;
import com.team254.frc2018.states.SuperstructureState;
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
    private SuperstructureMotionPlanner mPlanner = new SuperstructureMotionPlanner();


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

    synchronized void updateObservedState(SuperstructureState state) {
        state.height = mElevator.getInchesOffGround();
        state.angle = mWrist.getAngle();
        state.jawClamped = mIntake.getJawState() == IntakeState.JawState.CLAMPED;
    }

    // Update subsystems from planner
    synchronized void setFromCommandState(SuperstructureState commandState) {
        mElevator.setClosedLoopPosition(commandState.height);
        mWrist.setClosedLoopAngle(commandState.angle);
    }

    // Get commanded from user
    synchronized public void set(double height, double angle, IntakeStateMachine.WantedAction intakeAction) {
        updateObservedState(mState);
        SuperstructureState mCommandedState = new SuperstructureState(height, angle);
        mCommandedState.intakeAction = intakeAction;
        mPlanner.setDesiredState(mCommandedState, mState);
    }

    // Get commanded from user
    synchronized public void set(double height, double angle) {
        set(height, angle, IntakeStateMachine.WantedAction.IDLE);
    }


    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(new Loop() {
            int i = 0;
            @Override
            public void onStart(double timestamp) {
                i = 0;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    updateObservedState(mState);
                    SuperstructureState commandState = mPlanner.update(mState);
                    setFromCommandState(commandState);
                }

            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }
}
