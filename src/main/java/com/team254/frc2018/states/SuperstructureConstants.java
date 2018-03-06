package com.team254.frc2018.states;

public class SuperstructureConstants {
    public static final double kWristMinAngle = 0.0;
    public static final double kWristMaxAngle = 180.0;
    public static final double kElevatorMaxHeight = 84.0;
    public static final double kElevatorMinHeight = 4.0;

    public static final double kClearFirstStageMaxHeight = 33.0;
    public static final double kClearFirstStageMinWristAngle = 45.0;

    public static final double kAlwaysNeedsJawClampMinAngle = kClearFirstStageMinWristAngle;

    public static final double kElevatorLongRaiseDistance = 28.0;
    public static final double kElevatorApproachingThreshold = 12.0;

    public final static double kStowedAngle = 90.0;

    // This is in inches / ~20ms
    public final static double kElevatorJogThrottle = 30.0 / 50.0;

    // This is in degrees / ~20ms
    public final static double kWristJogThrottle = 110.0 / 25.0;

    // In inches, the height to use the kPlacingHighAngle.
    public final static double kPlacingHighThreshold = 33.0;

    // Presets.

    // Combinations.
    public final static double kStowedPositionHeight = 0.0;
    public final static double kStowedPositionAngle = 0.0;

    public final static double kIntakePositionHeight = 0.0;
    public final static double kIntakePositionAngle = 180.0;

    // Elevator Heights.
    public final static double kScaleHighHeight = 82.0;
    public final static double kScaleNeutralHeight = 75.0;
    public final static double kScaleLowHeight = 63.0;

    public final static double kScaleHighHeightBackwards = 75.0;
    public final static double kScaleNeutralHeightBackwards = 65.0;
    public final static double kScaleLowHeightBackwards = 55.0;

    public final static double kIntakeThirdLevelHeight = 25.5;
    public final static double kIntakeSecondLevelHeight = 14.5;
    public final static double kIntakeFloorLevelHeight = 0.0;

    public final static double kSwitchHeight = 30.0;
    public final static double kSwitchHeightBackwards = 27.0;

    // Wrist Angles.
    public final static double kVerticalAngle = 90.0;
    public final static double kScoreBackwardsAngle = 45.0;
    public final static double kScoreForwardAngledAngle = 135.0;
    public final static double kScoreSwitchBackwardsAngle = 0.0;

    public final static double kPlacingLowAngle = 175.0;
    public final static double kPlacingHighAngle = 175.0;
    public final static double kWeakShootAngle = 130.0;
}
