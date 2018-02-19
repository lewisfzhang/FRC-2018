package com.team254.frc2018.statemachines;

import com.team254.frc2018.planners.SuperstructureMotionPlanner;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.states.SuperstructureState;
import com.team254.frc2018.subsystems.Elevator;
import com.team254.lib.util.Util;

public class SuperstructureStateMachine {
    public enum WantedAction {
        IDLE,
        INTAKE,
        GOTO_SCORE_POSITION,
        PLACE,
        SHOOT,
        STOW,
    }

    public enum SystemState {
        START,
        STOWED,
        STOWED_WITH_CUBE,
        IN_SCORING_POSITION,
        PLACING,
        SHOOTING,
        MOVING,
        INTAKING,
        INTAKE_POSITION,
    }

    private SystemState mSystemStateAfterMoving = SystemState.STOWED;
    private SystemState mSystemState = SystemState.START;
    private WantedAction mLastWantedAction = WantedAction.IDLE;

    private SuperstructureState mCommandedState = new SuperstructureState();
    private SuperstructureState mDesiredEndState = new SuperstructureState();
    private IntakeStateMachine.WantedAction mIntakeActionDuringMove =
            IntakeStateMachine.WantedAction.IDLE;
    private double mCurrentStateStartTime = 0.0;

    private SuperstructureMotionPlanner mPlanner = new SuperstructureMotionPlanner();

    private double mScoringHeight = Elevator.kHomePositionInches;
    private double mScoringAngle = SuperstructureConstants.kStowedAngle;

    public synchronized void setScoringPosition(double inches, double angle) {
        mScoringHeight = inches;
        mScoringAngle = angle;
    }

    public synchronized boolean scoringPositionChanged() {
        return !Util.epsilonEquals(mDesiredEndState.angle, mScoringAngle) ||
                !Util.epsilonEquals(mDesiredEndState.height, mScoringHeight);
    }

    public synchronized SuperstructureState update(double timestamp, WantedAction wantedAction,
                                      SuperstructureState currentState) {
        synchronized (SuperstructureStateMachine.this) {
            SystemState newState;

            double timeInState = timestamp - mCurrentStateStartTime;

            // Handle state transitions
            switch (mSystemState) {
                case START:
                    newState = handleStartTransitions(wantedAction, currentState);
                    break;
                case STOWED:
                    newState = handleStowedTransitions(wantedAction, currentState);
                    break;
                case STOWED_WITH_CUBE:
                    newState = handleStowedWithCubeTransitions(wantedAction, currentState);
                    break;
                case MOVING:
                    newState = handleMovingTransitions(wantedAction, currentState);
                    break;
                case INTAKING:
                    newState = handleIntakingTransitions(wantedAction, currentState);
                    break;
                case INTAKE_POSITION:
                    newState = handleIntakePositionTransitions(wantedAction, currentState);
                    break;
                case IN_SCORING_POSITION:
                    newState = handleScoringPositionTransitions(wantedAction, currentState);
                    break;
                case PLACING:
                    newState = handlePlacingTransitions(timeInState, wantedAction, currentState);
                    break;
                case SHOOTING:
                    newState = handleShootingTransitions(timeInState, wantedAction, currentState);
                    break;
                default:
                    System.out.println("Unexpected superstructure system state: " + mSystemState);
                    newState = mSystemState;
                    break;
            }

            if (newState != mSystemState) {
                System.out.println(timestamp + ": Superstructure changed state: " + mSystemState + " -> " + newState);
                mSystemState = newState;
                mCurrentStateStartTime = timestamp;
            }

            // Pump elevator planner
            mCommandedState = mPlanner.update(currentState);

            // Handle state outputs
            switch (mSystemState) {
                case STOWED:
                    getStowedCommandedState();
                    break;
                case STOWED_WITH_CUBE:
                    getStowedWithCubeCommandedState();
                    break;
                case MOVING:
                    getMovingCommandedState();
                    break;
                case INTAKING:
                    getIntakingCommandedState();
                    break;
                case INTAKE_POSITION:
                    getIntakePositionCommandedState();
                    break;
                case IN_SCORING_POSITION:
                    getScoringPositionCommandedState();
                    break;
                case PLACING:
                    getPlacingCommandedState();
                    break;
                case SHOOTING:
                    getShootingCommandedState();
                    break;
                default:
                    getStowedCommandedState();
                    break;
            }

            mLastWantedAction = wantedAction;
            return mCommandedState;
        }
    }

    private void updateMotionPlannerDesired(SystemState desiredState,
                                            SuperstructureState currentState) {
        mSystemStateAfterMoving = desiredState;
        switch (desiredState) {
            case INTAKING:
                // Set elevator to intake position.
                mDesiredEndState.height = Elevator.kHomePositionInches;
                mDesiredEndState.angle = SuperstructureConstants.kIntakeAngle;
                mDesiredEndState.jawClamped = true;
                mDesiredEndState.intakeAction = IntakeStateMachine.WantedAction.IDLE;
                break;
            case STOWED_WITH_CUBE:
                mDesiredEndState.height = Elevator.kHomePositionInches;
                mDesiredEndState.angle = SuperstructureConstants.kStowedWithCubeAngle;
                mDesiredEndState.jawClamped = true;
                mDesiredEndState.intakeAction = IntakeStateMachine.WantedAction.INTAKE;
                break;
            case STOWED:
                mDesiredEndState.height = Elevator.kHomePositionInches;
                mDesiredEndState.angle = SuperstructureConstants.kStowedAngle;
                mDesiredEndState.jawClamped = true;
                mDesiredEndState.intakeAction = IntakeStateMachine.WantedAction.IDLE;
                break;
            case IN_SCORING_POSITION:
                mDesiredEndState.height = mScoringHeight;
                mDesiredEndState.angle = mScoringAngle;
                mDesiredEndState.jawClamped = true;
                mDesiredEndState.intakeAction = IntakeStateMachine.WantedAction.IDLE;
                break;
            default:
                System.out.println("Fell through on motion planner states.");
                break;
        }

        System.out.println("Setting motion planner to height: " + mDesiredEndState.height
                + " angle: " + mDesiredEndState.angle);

        // Populate the scoring angle and height just in case the state transitions to SHOOTING.
        // Which transitions to IN_SCORING_POSITION
        mScoringAngle = mDesiredEndState.angle;
        mScoringHeight = mDesiredEndState.height;

        // Push into elevator planner.
        if (!mPlanner.setDesiredState(mDesiredEndState, currentState)) {
            System.out.println("Unable to set elevator planner!");
        }
    }

    // START
    private SystemState handleStartTransitions(WantedAction wantedAction,
                                               SuperstructureState currentState) {
        if (currentState.hasCube) {
            updateMotionPlannerDesired(SystemState.STOWED_WITH_CUBE, currentState);
            return SystemState.MOVING;
        } else {
            updateMotionPlannerDesired(SystemState.STOWED, currentState);
            return SystemState.MOVING;
        }
    }

    // STOWED
    private SystemState handleStowedTransitions(WantedAction wantedAction,
                                                SuperstructureState currentState) {
        // We actually have a cube, go to stowed with it.
        if (currentState.hasCube) {
            updateMotionPlannerDesired(SystemState.STOWED_WITH_CUBE, currentState);
            return SystemState.MOVING;
        }
        switch (wantedAction) {
            case INTAKE:
                updateMotionPlannerDesired(SystemState.INTAKING, currentState);
                return SystemState.MOVING;
            case IDLE:
            default:
                return SystemState.STOWED;
        }
    }
    private void getStowedCommandedState() {
        mCommandedState.intakeAction = IntakeStateMachine.WantedAction.IDLE;
    }

    // MOVING
    private SystemState handleMovingTransitions(WantedAction wantedAction,
                                                SuperstructureState currentState) {
        // If we encounter a new end state while moving, push into motion planner and obey new state.
        if (wantedAction != mLastWantedAction) {
            switch (wantedAction) {
                case GOTO_SCORE_POSITION:
                    if (scoringPositionChanged()) {
                        updateMotionPlannerDesired(SystemState.IN_SCORING_POSITION, currentState);
                    }
                    break;
                case INTAKE:
                    if (!currentState.hasCube) {
                        updateMotionPlannerDesired(SystemState.INTAKING, currentState);
                    }
                    break;
                case STOW:
                    if (currentState.hasCube) {
                        updateMotionPlannerDesired(SystemState.STOWED_WITH_CUBE, currentState);
                    } else {
                        updateMotionPlannerDesired(SystemState.STOWED, currentState);
                    }
                    break;
            }
        }

        // Determine if we are not done with planner.
        if (!mPlanner.isFinished(currentState)) {
            return SystemState.MOVING;
        }

        // Otherwise, transition to the state since we are done moving. And reset action.
        mIntakeActionDuringMove = IntakeStateMachine.WantedAction.IDLE;
        return mSystemStateAfterMoving;
    }
    private void getMovingCommandedState() {
        mCommandedState.intakeAction = mIntakeActionDuringMove;
    }

    // INTAKING
    private SystemState handleIntakingTransitions(WantedAction wantedAction,
                                                  SuperstructureState currentState) {
        // Check if we have a cube.
        if (currentState.hasCube) {
            updateMotionPlannerDesired(SystemState.STOWED_WITH_CUBE, currentState);
            return SystemState.STOWED_WITH_CUBE;
        }

        switch (wantedAction) {
            case INTAKE:
                return SystemState.INTAKING;
            default:
                return SystemState.INTAKE_POSITION;
        }
    }
    private void getIntakingCommandedState() {
        mCommandedState.intakeAction = IntakeStateMachine.WantedAction.INTAKE;
    }

    // INTAKE POSITION
    private SystemState handleIntakePositionTransitions(WantedAction wantedAction,
                                                        SuperstructureState currentState) {
        // Check if we have a cube.
        if (currentState.hasCube) {
            updateMotionPlannerDesired(SystemState.STOWED_WITH_CUBE, currentState);
            return SystemState.STOWED_WITH_CUBE;
        }

        switch (wantedAction) {
            case INTAKE:
                return SystemState.INTAKING;
            case GOTO_SCORE_POSITION:
                updateMotionPlannerDesired(SystemState.IN_SCORING_POSITION, currentState);
                return SystemState.MOVING;
            case STOW:
                updateMotionPlannerDesired(SystemState.STOWED, currentState);
                return SystemState.MOVING;
            default:
                return SystemState.INTAKE_POSITION;
        }
    }
    private void getIntakePositionCommandedState() {
        mCommandedState.intakeAction = IntakeStateMachine.WantedAction.INTAKE_POSITION;
    }

    // STOWED_WITH_CUBE
    private SystemState handleStowedWithCubeTransitions(WantedAction wantedAction,
                                                        SuperstructureState currentState) {
        if (!currentState.hasCube){
            // If we lose a cube, go back to STOWED.  Don't really need to hit moving state...
            updateMotionPlannerDesired(SystemState.STOWED, currentState);
            return SystemState.MOVING;
        }

        switch (wantedAction) {
            case GOTO_SCORE_POSITION:
                updateMotionPlannerDesired(SystemState.IN_SCORING_POSITION, currentState);
                return SystemState.MOVING;
            case SHOOT:
                return SystemState.SHOOTING;
            default:
                return SystemState.STOWED_WITH_CUBE;
        }
    }
    private void getStowedWithCubeCommandedState() {
        mCommandedState.intakeAction = IntakeStateMachine.WantedAction.IDLE;
    }

    // SCORING_POSITION
    private SystemState handleScoringPositionTransitions(WantedAction wantedAction,
                                                         SuperstructureState currentState) {
        switch (wantedAction) {
            case PLACE:
                // DO NOT ALLOW PLACING with certain angles.
                if (currentState.angle < SuperstructureConstants.kPlacingMinAngle) {
                    System.out.println("Unable to place with angle: " + currentState.angle);
                    return SystemState.IN_SCORING_POSITION;
                }
                return SystemState.PLACING;
            case SHOOT:
                return SystemState.SHOOTING;
            case STOW:
                if (currentState.hasCube) {
                    updateMotionPlannerDesired(SystemState.STOWED_WITH_CUBE, currentState);
                } else {
                    updateMotionPlannerDesired(SystemState.STOWED, currentState);
                }
                return SystemState.MOVING;
            case INTAKE:
                if (!currentState.hasCube) {
                    updateMotionPlannerDesired(SystemState.INTAKING, currentState);
                    return SystemState.MOVING;
                }
                return SystemState.IN_SCORING_POSITION;
            case GOTO_SCORE_POSITION:
                // Check if we need to move.
                if (scoringPositionChanged()) {
                    updateMotionPlannerDesired(SystemState.IN_SCORING_POSITION, currentState);
                   return SystemState.MOVING;
                }
                // FALL THROUGH INTENDED
            default:
                return SystemState.IN_SCORING_POSITION;
        }
    }
    private void getScoringPositionCommandedState() {
        mCommandedState.intakeAction = IntakeStateMachine.WantedAction.IDLE;
    }

    // PLACING
    private SystemState handlePlacingTransitions(double timeInState, WantedAction wantedAction,
                                                 SuperstructureState currentState) {
        if (timeInState < SuperstructureConstants.kMinTimePlacing) {
            return SystemState.PLACING;
        }
        switch (wantedAction) {
            case IDLE:
                // Take the current state and set the angle to stowed angle.
                mScoringAngle = SuperstructureConstants.kStowedAngle;
                mIntakeActionDuringMove = IntakeStateMachine.WantedAction.PLACE;
                updateMotionPlannerDesired(SystemState.IN_SCORING_POSITION, currentState);
                return SystemState.MOVING;
            default:
                return SystemState.PLACING;
        }
    }
    private void getPlacingCommandedState() {
        mCommandedState.intakeAction = IntakeStateMachine.WantedAction.PLACE;
    }

    // SHOOTING
    private SystemState handleShootingTransitions(double timeInState,
                                                  WantedAction wantedAction,
                                                  SuperstructureState currentState) {
        if (timeInState < SuperstructureConstants.kMinTimeShooting) {
            return SystemState.SHOOTING;
        }
        switch (wantedAction) {
            case IDLE:
                // Take the current state and set the angle to stowed angle.
                mScoringAngle = SuperstructureConstants.kStowedAngle;
                mIntakeActionDuringMove = IntakeStateMachine.WantedAction.SHOOT;
                updateMotionPlannerDesired(SystemState.IN_SCORING_POSITION, currentState);
                return SystemState.MOVING;
            default:
                return SystemState.SHOOTING;
        }
    }
    private void getShootingCommandedState() {
        mCommandedState.intakeAction =  IntakeStateMachine.WantedAction.SHOOT;
    }
}
