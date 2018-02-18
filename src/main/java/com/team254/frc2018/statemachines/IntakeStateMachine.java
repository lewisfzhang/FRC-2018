package com.team254.frc2018.statemachines;

import com.team254.frc2018.states.IntakeState;
import com.team254.frc2018.states.SuperstructureConstants;


public class IntakeStateMachine {
    public final static double kActuationTime = 0.25;
    public final static double kShootSetpoint = 1.0;
    public final static double kIntakeCubeSetpoint = -1.0;
    public final static double kHoldSetpoint = 0;
    public final static double kShootTime = 0.25;

    public enum WantedAction {
        IDLE,
        INTAKE,
        SHOOT,
        PLACE
    }

    private enum SystemState {
        IDLE,
        INTAKING,
        CLAMPING,
        HOLDING,
        SHOOTING,
        PLACING
    }

    private SystemState mSystemState = SystemState.IDLE;
    private IntakeState mCommandedState = new IntakeState();
    private double mCurrentStateStartTime = 0;

    public IntakeState update(double timestamp, WantedAction wantedAction, IntakeState currentState) {
        synchronized (IntakeStateMachine.this) {
            SystemState newState;
            double timeInState = timestamp - mCurrentStateStartTime;

            // Handle state transitions
            switch (mSystemState) {
                case IDLE:
                    newState = handleIdleTransitions(wantedAction, currentState);
                    break;
                case INTAKING:
                    newState = handleIntakingTransitions(wantedAction, currentState);
                    break;
                case CLAMPING:
                    newState = handleClampingTransitions(timeInState, currentState);
                    break;
                case HOLDING:
                    newState = handleHoldingTransitions(wantedAction, currentState);
                    break;
                case SHOOTING:
                    newState = handleShootingTransitions(timeInState);
                    break;
                case PLACING:
                    newState = handlePlacingTransitions(timeInState);
                    break;
                default:
                    System.out.println("Unexpected intake system state: " + mSystemState);
                    newState = mSystemState;
                    break;
            }
            
            if (newState != mSystemState) {
                System.out.println(timestamp + ": Changed state: " + mSystemState + " -> " + newState);
                mSystemState = newState;
                mCurrentStateStartTime = timestamp;
            }
            
            // Handle State outputs
            switch (mSystemState) {
                case IDLE:
                    getIdleCommandedState(currentState, mCommandedState);
                    break;
                case INTAKING:
                    getIntakingCommandedState(currentState, mCommandedState);
                    break;
                case CLAMPING:
                    getClampingCommandedState(currentState, mCommandedState);
                    break;
                case HOLDING:
                    getHoldingCommandedState(currentState, mCommandedState);
                    break;
                case PLACING:
                    getPlacingCommandedState(currentState, mCommandedState);
                    break;
                case SHOOTING:
                    getShootingCommandedState(currentState, mCommandedState);
                    break;
                default:
                    getIdleCommandedState(currentState, mCommandedState);
                    break;
            }
        }
        return mCommandedState;
    }

    // Idle
    private synchronized void getIdleCommandedState(IntakeState currentState, IntakeState commandedState) {
        commandedState.setPower(0);
        commandedState.jawState = IntakeState.JawState.CLOSED;
    }

    private synchronized SystemState handleIdleTransitions(WantedAction wantedAction, IntakeState currentState) {
        switch (wantedAction) {
            case INTAKE:
                return SystemState.INTAKING;
            default:
                return SystemState.IDLE;
        }
    }

    // Intaking
    private synchronized void getIntakingCommandedState(IntakeState currentState, IntakeState commandedState) {
        commandedState.setPower(kIntakeCubeSetpoint);
        commandedState.jawState = IntakeState.JawState.CLOSED;
    }

    private synchronized SystemState handleIntakingTransitions(WantedAction wantedAction, IntakeState currentState) {
        if (currentState.seesCube()) {
            return SystemState.CLAMPING;
        }

        switch (wantedAction) {
            case IDLE:
                return SystemState.IDLE;
            default:
                return SystemState.INTAKING;
        }
    }

    // Clamping
    private synchronized void getClampingCommandedState(IntakeState currentState, IntakeState commandedState) {
        commandedState.setPower(kIntakeCubeSetpoint);
        commandedState.jawState = IntakeState.JawState.CLAMPED;
    }

    private synchronized SystemState handleClampingTransitions(double timeInState, IntakeState currentState) {
        if (currentState.seesCube()) {
            if (timeInState > kActuationTime) {
                return SystemState.HOLDING;
            } else {
                return SystemState.CLAMPING;
            }
        } else {
            return SystemState.IDLE;
        }
    }

    // Holding
    private synchronized void getHoldingCommandedState(IntakeState currentState, IntakeState commandedState) {
        commandedState.setPower(kHoldSetpoint);
        commandedState.jawState = IntakeState.JawState.CLAMPED;
    }

    private synchronized SystemState handleHoldingTransitions(WantedAction wantedAction, IntakeState currentState) {
        if (!currentState.seesCube()) {
            return SystemState.IDLE;
        }

        switch (wantedAction) {
            case SHOOT:
                return SystemState.SHOOTING;
            case PLACE:
                if (currentState.wristAngle < SuperstructureConstants.kAlwaysNeedsJawClampMinAngle) { // don't kill the elevator
                    // TODO:  make this work
                    // DriverStation.reportError("Can't open the jaw when the wrist is at that angle", false);
                    return SystemState.HOLDING;
                } else {
                    return SystemState.PLACING;
                }
            default:
                return SystemState.HOLDING;
        }
    }

    // Placing
    private synchronized void getPlacingCommandedState(IntakeState currentState, IntakeState commandedState) {
        commandedState.setPower(0);
        commandedState.jawState = IntakeState.JawState.OPEN;
    }

    private synchronized SystemState handlePlacingTransitions(double timeInState) {
        if (timeInState > kActuationTime) {
            return SystemState.IDLE;
        } else {
            return SystemState.PLACING;
        }
    }

    // Shooting
    private synchronized void getShootingCommandedState(IntakeState currentState, IntakeState commandedState) {
        commandedState.setPower(kShootSetpoint);
        commandedState.jawState = IntakeState.JawState.CLAMPED;
    }
    private synchronized SystemState handleShootingTransitions(double timeInState) {
        if (timeInState > kShootTime) {
            return SystemState.IDLE;
        } else {
            return SystemState.SHOOTING;
        }
    }

    // Getters
    public boolean hasCubeClamped() {
        return mSystemState == SystemState.HOLDING;
    }
}
