package com.team254.frc2018.planners;

import com.team254.frc2018.states.ElevatorState;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.lib.util.Util;

import java.util.LinkedList;
import java.util.Optional;

public class ElevatorMotionPlanner {
    class SubCommand {
        public SubCommand(ElevatorState endState) {
            mEndState = endState;
        }
        public ElevatorState mEndState;
        public double mHeightThreshold = .75;
        public double mWristThreshold = 2;

        public boolean isFinished(ElevatorState currentState) {
            return mEndState.isInRange(currentState, mHeightThreshold, mWristThreshold);
        }
    }

    protected ElevatorState mCommandedState = new ElevatorState();
    protected ElevatorState mIntermediateCommandState = new ElevatorState();
    protected LinkedList<SubCommand> mCommandQueue = new LinkedList<>();
    protected Optional<SubCommand> mCurrentCommand = Optional.empty();

    public boolean setDesiredState(ElevatorState desiredStateIn, ElevatorState currentState) {
        ElevatorState desiredState = new ElevatorState(desiredStateIn);

        // Limit stupid inputs
        desiredState.angle = Util.limit(desiredState.angle, SuperstructureConstants.kWristMinAngle, SuperstructureConstants.kWristMaxAngle);
        desiredState.height = Util.limit(desiredState.height, SuperstructureConstants.kElevatorMinHeight, SuperstructureConstants.kElevatorMaxHeight);

        ElevatorState swapJaw = new ElevatorState(currentState);
        swapJaw.jawClosed = desiredState.jawClosed;

        // Immediate return, totally illegal commands
        if (desiredState.inIllegalZone() || desiredState.inIllegalJawZone() || swapJaw.inIllegalJawZone()) { // if we cant move there ever
            // return false, let the sender optionally deal with the error
            return false;
        }

        // Everything beyond this is probably do-able; clear queue
        mCommandQueue.clear();

        // The following are bad commands, but fixable. Massage them into legal-ness
        boolean willEndClearingFirstStage = desiredState.height > SuperstructureConstants.kClearFirstStageMinHeight;
        if (willEndClearingFirstStage && (desiredState.angle <= SuperstructureConstants.kClearFirstStageMinWristAngle)) {
            desiredState.angle = SuperstructureConstants.kClearFirstStageMinWristAngle;
        }

        // Compute zone infraction states
        boolean startingInCrossBarZone =  currentState.height <= SuperstructureConstants.kIllegalCrossbarStowMaxHeight &&
                currentState.height >= SuperstructureConstants.kIllegalCrossbarStowMinHeight;

        boolean endingInCrossBarZone =  desiredState.height <= SuperstructureConstants.kIllegalCrossbarStowMaxHeight &&
                desiredState.height >= SuperstructureConstants.kIllegalCrossbarStowMinHeight;

        boolean elevatorWillCrossThroughCrossBarZone = (currentState.height > SuperstructureConstants.kIllegalCrossbarStowMaxHeight &&
                desiredState.height < SuperstructureConstants.kIllegalCrossbarStowMaxHeight) ||
                (currentState.height < SuperstructureConstants.kIllegalCrossbarStowMinHeight &&
                desiredState.height > SuperstructureConstants.kIllegalCrossbarStowMinHeight);

        boolean wristWillCrossThroughCrossBarZone = currentState.angle < SuperstructureConstants.kIllegalCrossbarStowMinAngle ||
                desiredState.angle < SuperstructureConstants.kIllegalCrossbarStowMinAngle;

        boolean willCrossThroughCrossBarZone = (elevatorWillCrossThroughCrossBarZone ||
                endingInCrossBarZone ||
                startingInCrossBarZone) && wristWillCrossThroughCrossBarZone ;

        boolean movingFar = Math.abs(desiredState.height - currentState.height) > SuperstructureConstants.kWristStowForMinElevatorMoveDistance;

        // Break desired state into fixed movements
        if (movingFar || willCrossThroughCrossBarZone) {  // Stow intake when we are making large movements or into a bad zone
            // Stow wrist
            mCommandQueue.add(new SubCommand(new ElevatorState(currentState.height, SuperstructureConstants.kWristStowedPosition, true)));
            // Move elevator
            mCommandQueue.add(new SubCommand(new ElevatorState(desiredState.height, SuperstructureConstants.kWristStowedPosition, true)));
            // Move wrist to end state
            mCommandQueue.add(new SubCommand(new ElevatorState(desiredState.height, desiredState.angle, true)));
        } else {
            mCommandQueue.add(new SubCommand(desiredState));
        }

        // Reset current command to start executing on next iteration
        mCurrentCommand = Optional.empty();

        return true; // this is a legal move
    }

    void reset(ElevatorState currentState) {
        mIntermediateCommandState = currentState;
        mCommandQueue.clear();
        mCurrentCommand = Optional.empty();
    }

    ElevatorState update(ElevatorState currentState) {
        if (!mCurrentCommand.isPresent() && !mCommandQueue.isEmpty()) {
            mCurrentCommand = Optional.of(mCommandQueue.remove());
        }

        if (mCurrentCommand.isPresent()) {
            SubCommand subCommand = mCurrentCommand.get();
            mIntermediateCommandState = subCommand.mEndState;
            if (subCommand.isFinished(currentState) && !mCommandQueue.isEmpty()) {
                // Let the current command persist until there is something in the queue. or not. desired outcome unclear.
                mCurrentCommand = Optional.empty();
            }
        } else {
            mIntermediateCommandState = currentState;
        }

        mCommandedState.angle = Util.limit(mIntermediateCommandState.angle, SuperstructureConstants.kWristMinAngle, SuperstructureConstants.kWristMaxAngle);
        mCommandedState.height = Util.limit(mIntermediateCommandState.height, SuperstructureConstants.kElevatorMinHeight, SuperstructureConstants.kElevatorMaxHeight);

        return mCommandedState;
    }
}
