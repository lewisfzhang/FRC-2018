package com.team254.frc2018.statemachines;

import com.team254.frc2018.planners.SuperstructureMotionPlanner;
import com.team254.frc2018.states.SuperstructureCommand;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.states.SuperstructureState;
import com.team254.frc2018.subsystems.Elevator;
import com.team254.lib.util.Util;

public class SuperstructureStateMachine {
    public enum WantedAction {
        IDLE,
        GO_TO_POSITION,
        HANG  // TODO break into constituent states
    }

    public enum SystemState {
        HOLDING_POSITION,
        MOVING_TO_POSITION,
        HANGING
    }

    private SystemState mSystemState = SystemState.HOLDING_POSITION;

    private SuperstructureCommand mCommand = new SuperstructureCommand();
    private SuperstructureState mCommandedState = new SuperstructureState();
    private SuperstructureState mDesiredEndState = new SuperstructureState();

    private SuperstructureMotionPlanner mPlanner = new SuperstructureMotionPlanner();

    private double mScoringHeight = Elevator.kHomePositionInches;
    private double mScoringAngle = SuperstructureConstants.kStowedAngle;

    private double mOpenLoopPower = 0.0;

    public synchronized void setOpenLoopPower(double power) { mOpenLoopPower = power; }

    public synchronized void setScoringHeight(double inches) {
        mScoringHeight = inches;
    }

    public synchronized double getScoringHeight() {
        return mScoringHeight;
    }

    public synchronized void setScoringAngle(double angle) {
        mScoringAngle = angle;
    }

    public synchronized double getScoringAngle() {
        return mScoringAngle;
    }

    public synchronized void jogElevator(double relative_inches) {
        mScoringHeight += relative_inches;
        mScoringHeight = Math.min(mScoringHeight, SuperstructureConstants.kElevatorMaxHeight);
    }

    public synchronized void jogWrist(double relative_degrees) { mScoringAngle += relative_degrees; }

    public synchronized boolean scoringPositionChanged() {
        return !Util.epsilonEquals(mDesiredEndState.angle, mScoringAngle) ||
                !Util.epsilonEquals(mDesiredEndState.height, mScoringHeight);
    }

    public synchronized SystemState getSystemState() {
        return mSystemState;
    }

    public synchronized SuperstructureCommand update(double timestamp, WantedAction wantedAction,
                                                     SuperstructureState currentState) {
        synchronized (SuperstructureStateMachine.this) {
            SystemState newState;

            // Handle state transitions
            switch (mSystemState) {
                case HOLDING_POSITION:
                    newState = handleHoldingPositionTransitions(wantedAction, currentState);
                    break;
                case MOVING_TO_POSITION:
                    newState = handleMovingToPositionTransitions(wantedAction, currentState);
                    break;
                case HANGING:
                    newState = handleHangingTransitions(wantedAction, currentState);
                    break;
                default:
                    System.out.println("Unexpected superstructure system state: " + mSystemState);
                    newState = mSystemState;
                    break;
            }

            if (newState != mSystemState) {
                System.out.println(timestamp + ": Superstructure changed state: " + mSystemState + " -> " + newState);
                mSystemState = newState;
            }

            // Pump elevator planner only if not jogging.
            if (!mCommand.openLoopElevator) {
                mCommandedState = mPlanner.update(currentState);
                mCommand.height = mCommandedState.height;
                mCommand.wristAngle = mCommandedState.angle;
            }

            // Handle state outputs
            switch (mSystemState) {
                case HOLDING_POSITION:
                    getHoldingPositionCommandedState();
                    break;
                case MOVING_TO_POSITION:
                    getMovingToPositionCommandedState();
                    break;
                default:
                    getHangingCommandedState();
                    break;
            }

            return mCommand;
        }
    }

    private void updateMotionPlannerDesired(SuperstructureState currentState) {
        System.out.println("Setting motion planner to height: " + mDesiredEndState.height
                + " angle: " + mDesiredEndState.angle);

        mDesiredEndState.angle = mScoringAngle;
        mDesiredEndState.height = mScoringHeight;

        // Push into elevator planner.
        if (!mPlanner.setDesiredState(mDesiredEndState, currentState)) {
            System.out.println("Unable to set elevator planner!");
        }

        mScoringAngle = mDesiredEndState.angle;
        mScoringHeight = mDesiredEndState.height;
    }

    private SystemState handleDefaultTransitions(WantedAction wantedAction, SuperstructureState currentState) {
        if (wantedAction == WantedAction.GO_TO_POSITION) {
            if (scoringPositionChanged()) {
                updateMotionPlannerDesired(currentState);
            } else if (mPlanner.isFinished(currentState)) {
                return SystemState.HOLDING_POSITION;
            }
            return SystemState.MOVING_TO_POSITION;
        } else if (wantedAction == WantedAction.HANG) {
            return SystemState.HANGING;
        } else {
            if (mSystemState == SystemState.MOVING_TO_POSITION && !mPlanner.isFinished(currentState)) {
                return SystemState.MOVING_TO_POSITION;
            } else {
                return SystemState.HOLDING_POSITION;
            }
        }
    }

    // HOLDING_POSITION
    private SystemState handleHoldingPositionTransitions(WantedAction wantedAction,
                                               SuperstructureState currentState) {
        return handleDefaultTransitions(wantedAction, currentState);
    }
    private void getHoldingPositionCommandedState() {
        mCommand.elevatorLowGear = false;
        mCommand.openLoopElevator = false;
    }

    // MOVING_TO_POSITION
    private SystemState handleMovingToPositionTransitions(WantedAction wantedAction,
                                                         SuperstructureState currentState) {

        return handleDefaultTransitions(wantedAction, currentState);
    }
    private void getMovingToPositionCommandedState() {
        mCommand.elevatorLowGear = false;
        mCommand.openLoopElevator = false;
    }

    // HANGING
    private SystemState handleHangingTransitions(WantedAction wantedAction,
                                                 SuperstructureState currentState) {
        return handleDefaultTransitions(wantedAction, currentState);
    }
    private void getHangingCommandedState() {
        mCommand.elevatorLowGear = true;
        mCommand.wristAngle = SuperstructureConstants.kWristMaxAngle;
        mCommand.openLoopElevator = true;
        mCommand.openLoopElevatorPercent = mOpenLoopPower;
    }
}
