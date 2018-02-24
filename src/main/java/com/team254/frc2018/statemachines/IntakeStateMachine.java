package com.team254.frc2018.statemachines;

import com.team254.frc2018.states.IntakeState;
import com.team254.frc2018.states.SuperstructureConstants;

public class IntakeStateMachine {
    public final static double kActuationTime = 0.0;
    public final static double kShootSetpoint = 1.0;
    public final static double kIntakeCubeSetpoint = -1.0;
    public final static double kHoldSetpoint = 0.0;
    public final static double kLostCubeTime = 0.5;

    public enum WantedAction {
        WANT_MANUAL,
        WANT_CUBE
    }

    private enum SystemState {
        OPEN_LOOP,
        KEEPING_CUBE
    }

    private SystemState mSystemState = SystemState.OPEN_LOOP;
    private IntakeState mCommandedState = new IntakeState();
    private double mCurrentStateStartTime = 0;

    private IntakeState.JawState mWantedJawState = IntakeState.JawState.CLAMPED;
    private double mWantedPower = 0.0;

    public synchronized void setWantedJawState(final IntakeState.JawState jaw_state) {
        mWantedJawState = jaw_state;
    }

    public synchronized void setWantedPower(double power) {
        mWantedPower = power;
    }

    public IntakeState update(double timestamp, WantedAction wantedAction, IntakeState currentState) {
        synchronized (IntakeStateMachine.this) {
            SystemState newState;
            double timeInState = timestamp - mCurrentStateStartTime;

            // Handle state transitions
            switch (mSystemState) {
                case OPEN_LOOP:
                    newState = handleOpenLoopTransitions(wantedAction, currentState);
                    break;
                case KEEPING_CUBE:
                    newState = handleKeepingCubeTransitions(wantedAction, currentState);
                    break;
                default:
                    System.out.println("Unexpected intake system state: " + mSystemState);
                    newState = mSystemState;
                    break;
            }
            
            if (newState != mSystemState) {
                System.out.println(timestamp + ": Intake changed state: " + mSystemState + " -> " + newState);
                mSystemState = newState;
                mCurrentStateStartTime = timestamp;
            }
            
            // Handle State outputs
            switch (mSystemState) {
                case OPEN_LOOP:
                    getOpenLoopCommandedState(currentState, mCommandedState);
                    break;
                case KEEPING_CUBE:
                    getKeepingCubeCommandedState(currentState, mCommandedState);
                    break;
                default:
                    getOpenLoopCommandedState(currentState, mCommandedState);
                    break;
            }
        }
        return mCommandedState;
    }

    // IDLE
    private synchronized SystemState handleOpenLoopTransitions(WantedAction wantedAction, IntakeState currentState) {
        if (currentState.seesCube() && mWantedPower <= 0.0 && mWantedJawState != IntakeState.JawState.OPEN) {
            // TODO think about this...
            return SystemState.KEEPING_CUBE;
        }
        switch (wantedAction) {
            case WANT_CUBE:
                return SystemState.KEEPING_CUBE;
            default:
                return SystemState.OPEN_LOOP;
        }
    }
    private synchronized void getOpenLoopCommandedState(IntakeState currentState, IntakeState commandedState) {
        commandedState.setPower(mWantedPower);
        if (mustStayClamped(currentState)) {
            commandedState.jawState = IntakeState.JawState.CLAMPED;
        } else {
            commandedState.jawState = mWantedJawState;
        }
        commandedState.ledState = new IntakeState.LEDState(0.0,0.0,0.0);
    }

    // KEEP_CUBE
    private synchronized SystemState handleKeepingCubeTransitions(WantedAction wantedAction, IntakeState currentState) {
        switch (wantedAction) {
            case WANT_MANUAL:
                return SystemState.OPEN_LOOP;
            default:
                return SystemState.KEEPING_CUBE;
        }
    }
    private synchronized void getKeepingCubeCommandedState(IntakeState currentState, IntakeState commandedState) {
        commandedState.setPower(kIntakeCubeSetpoint);
        final boolean clamp = (currentState.seesCube() && mWantedJawState != IntakeState.JawState.OPEN) || mustStayClamped(currentState);
        final boolean open = !clamp && mWantedJawState == IntakeState.JawState.OPEN;
        if (currentState.seesCube()) {
            commandedState.setPower(kHoldSetpoint);
            commandedState.jawState = clamp ? IntakeState.JawState.CLAMPED : IntakeState.JawState.OPEN;
            commandedState.ledState = new IntakeState.LEDState(1.0, 0.0, 0.0);
        } else {
            commandedState.setPower(kIntakeCubeSetpoint);
            commandedState.jawState = clamp ? IntakeState.JawState.CLAMPED : (mWantedJawState == IntakeState.JawState.OPEN ? IntakeState.JawState.OPEN : IntakeState.JawState.CLOSED);
            commandedState.ledState = new IntakeState.LEDState(0.0, 1.0, 0.0);
        }
    }

    private boolean mustStayClamped(IntakeState state) {
        return state.wristSetpoint < SuperstructureConstants.kAlwaysNeedsJawClampMinAngle ||
                state.wristAngle < SuperstructureConstants.kAlwaysNeedsJawClampMinAngle;
    }
}
