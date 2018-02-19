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
    public final static double kStowedWithCubeAngle = 0.0;
    public final static double kIntakeAngle = 180.0;

    public final static double kPlacingMinAngle = 145.0;
    public final static double kMinTimePlacing = 0.25;
    public final static double kMinTimeShooting = 0.25;

    public final static double kEarlyIntakeStartHeight = 10.0;
    public final static double kEarlyIntakeStartAngle = 135.0;

    // Should be -
    public final static double kJogUpPercent = -0.4;
    public final static double kJogDownPercent = 0.25;

    public enum ScoringPositionID {
        SWITCH,
        SWITCH_BACKWARDS,
        SCALE_LOW,
        SCALE_LOW_BACKWARDS,
        SCALE_NEUTRAL,
        SCALE_NEUTRAL_BACKWARDS,
        SCALE_HIGH,
        SCALE_HIGH_BACKWARDS,
    }

    public static class ScoringPosition {
        public double height = Elevator.kHomePositionInches;
        public double angle = kStowedAngle;

        public ScoringPosition(double in_height, double in_angle) {
            height = in_height;
            angle = in_angle;
        }
    }

    public static HashMap<ScoringPositionID, ScoringPosition> kScoringPositions =
            new HashMap<ScoringPositionID, ScoringPosition>() {
                {
                    put(ScoringPositionID.SWITCH, new ScoringPosition(30.0, 165.0));
                    put(ScoringPositionID.SWITCH_BACKWARDS,
                            new ScoringPosition(30.0, 0.0));

                    put(ScoringPositionID.SCALE_LOW, new ScoringPosition(60.0, 165.0));
                    put(ScoringPositionID.SCALE_LOW_BACKWARDS,
                            new ScoringPosition(60.0, 45.0));

                    put(ScoringPositionID.SCALE_NEUTRAL,
                            new ScoringPosition(70.0, 165.0));
                    put(ScoringPositionID.SCALE_NEUTRAL_BACKWARDS,
                            new ScoringPosition(70.0, 45.0));

                    put(ScoringPositionID.SCALE_HIGH, new ScoringPosition(80.0, 165.0));
                    put(ScoringPositionID.SCALE_HIGH_BACKWARDS,
                            new ScoringPosition(80.0, 45.0));
                }
    };
}
