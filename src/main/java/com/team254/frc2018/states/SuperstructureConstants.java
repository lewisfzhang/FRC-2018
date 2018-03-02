package com.team254.frc2018.states;

import com.team254.frc2018.subsystems.Elevator;

import java.util.HashMap;

public class SuperstructureConstants {
    public static final double kWristMinAngle = 0.0;
    public static final double kWristMaxAngle = 180.0;
    public static final double kElevatorMaxHeight = 83.5;
    public static final double kElevatorMinHeight = 4.9; // ?
    public static final double kWristStowedPosition = 90.0;

    public static final double kClearFirstStageMaxHeight = 33.0;
    public static final double kClearFirstStageMinWristAngle = 45.0;

    public static final double kAlwaysNeedsJawClampMinAngle = kClearFirstStageMinWristAngle;

    public static final double kElevatorLongRaiseDistance = 24.0;
    public static final double kElevatorApproachingThreshold = 12.0;

    public final static double kStowedAngle = 90.0;

    public final static double kElevatorJogUpThrottle = 24.0 / 50.0;
    public final static double kElevatorJogDownThrottle = -24.0 / 50.0;

    public final static double kWristJogUpThrottle = 90.0 / 25.0;
    public final static double kWristJogDownThrottle = -90.0 / 25.0;

    // In inches, the height to use the kPlacingHighAngle.
    public final static double kPlacingHighThreshold = 5.0;

    // Presets.

    // Combinations.
    public final static double kStowedPositionHeight = 0.0;
    public final static double kStowedPositionAngle = 0.0;

    public final static double kIntakePositionHeight = 0.0;
    public final static double kIntakePositionAngle = 180.0;

    // Elevator Heights.
    public final static double kScaleHighHeight = 80.0;
    public final static double kScaleNeutralHeight = 70.0;
    public final static double kScaleLowHeight = 60.0;

    public final static double kIntakeThirdLevelHeight = 25.5;
    public final static double kIntakeSecondLevelHeight = 14.5;
    public final static double kIntakeFloorLevelHeight = 0.0;

    public final static double kSwitchHeight = 30.0;

    // Wrist Angles.
    public final static double kVerticalAngle = 90.0;
    public final static double kScoreBackwardsAngle = 45.0;
    public final static double kScoreForwardAngledAngle = 135.0;

    public final static double kPlacingLowAngle = 175.0;
    public final static double kPlacingHighAngle = 155.0;
    public final static double kWeakShootAngle = 130.0;
}
