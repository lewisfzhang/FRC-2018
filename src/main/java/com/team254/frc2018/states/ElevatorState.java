package com.team254.frc2018.states;

import com.team254.lib.util.Util;

public class ElevatorState {
    public double height = SuperstructureConstants.kElevatorMinHeight;
    public double angle = SuperstructureConstants.kWristMinAngle;
    public boolean jawClosed = true;

    public ElevatorState(double height, double angle, boolean jawClosed) {
        this.height = height;
        this.angle = angle;
        this.jawClosed = jawClosed;
    }

    public ElevatorState(double height, double angle) {
        this(height, angle, true);
    }

    public ElevatorState(ElevatorState other) {
        this.height = other.height;
        this.angle = other.angle;
        this.jawClosed = other.jawClosed;
    }

    public ElevatorState() {
        this(SuperstructureConstants.kElevatorMinHeight, SuperstructureConstants.kWristMinAngle, true);
    }

    public boolean inIllegalZone(boolean allowSmallErrors) {
        double kAllowableWristAngleError = allowSmallErrors ? 5.5 : 0;
        double kAllowableElevatorHeightError = allowSmallErrors ? 1 : 0;

        if (height >= SuperstructureConstants.kIllegalCrossbarStowMinHeight + kAllowableElevatorHeightError &&
                height < SuperstructureConstants.kIllegalCrossbarStowMaxHeight - kAllowableElevatorHeightError &&
                angle < SuperstructureConstants.kIllegalCrossbarStowMinAngle - kAllowableWristAngleError) {
            return true;
        }

        return false;
    }

    public boolean inIllegalZone() {
        return inIllegalZone(false);
    }

    public boolean inIllegalJawZone() {
        return angle < SuperstructureConstants.kAlwaysNeedsJawClampMinAngle && !jawClosed;
    }

    public boolean isInRange(ElevatorState otherState, double heightThreshold, double wristThreshold) {
        return Util.epsilonEquals(otherState.height, height, heightThreshold) &&
                Util.epsilonEquals(otherState.angle, angle, wristThreshold);

    }

    @Override
    public String toString() {
        return "" + height + " / " + angle + " / " + jawClosed;
    }

}
