package com.team254.frc2018.states;

public class SuperstructureConstants {
    public static final double kWristMaxAngle = 180.0;
    public static final double kElevatorMaxHeight = 83.5;
    public static final double kElevatorMinHeight = 4.9; // ?
    public static final double kWristMinAngle = 0.0;
    public static final double kWristStowForMinElevatorMoveDistance = 10.0;
    public static final double kWristStowedPosition = 90.0;

    public static final double kIllegalCrossbarStowMinHeight = 30.5;
    public static final double kIllegalCrossbarStowMaxHeight = 43.5;
    public static final double kIllegalCrossbarStowMinAngle = 90.0;

    public static final double kClearFirstStageMinHeight = 43.5;
    public static final double kClearFirstStageMinWristAngle = 60.0;

    public static final double kAlwaysNeedsJawClampMinAngle = kClearFirstStageMinWristAngle;
}
