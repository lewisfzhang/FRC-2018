package com.team254.frc2018.lidar;

import java.util.LinkedHashMap;
import java.util.Map;

import com.team254.frc2018.RobotState;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

/**
 * Represents a single point from the lidar
 */
class LidarPoint {
    public final double timestamp;
    public final double angle;
    public final double distance;
    private RobotState mRobotState = RobotState.getInstance();
    
    private final static int MAX_ENTRIES = 10;
    private final static LinkedHashMap<Double, Pose2d> mRobotPoseMap = new LinkedHashMap<Double, Pose2d>() {
        @Override
        protected boolean removeEldestEntry(Map.Entry<Double, Pose2d> eldest) {
            return this.size() > MAX_ENTRIES;
        }
    };

    public static final double MM_TO_IN = 1 / 25.4; // 1 inch = 25.4 millimeters

    public LidarPoint(double timestamp, double angle, double distance) {
        this.timestamp = timestamp;
        this.angle = angle;
        this.distance = distance * MM_TO_IN;
    }

    public Translation2d toCartesian() {
        double radians = Math.toRadians(angle);
        Translation2d cartesian = new Translation2d(Math.cos(radians) * distance, Math.sin(radians) * distance);

        if (excludePoint(cartesian.x(), cartesian.y())) {
            return null;
        }

        Pose2d robotPose;
        if (mRobotPoseMap.containsKey(timestamp)) {
            robotPose = mRobotPoseMap.get(timestamp);
        } else {
            robotPose = mRobotState.getFieldToLidar(timestamp);
            mRobotPoseMap.put(timestamp, robotPose);
        }

        robotPose.transformBy(Pose2d.fromTranslation(cartesian));
        return robotPose.getTranslation();
    }


    private static final double FIELD_WIDTH = 27*12, FIELD_HEIGHT = 54*12;
    private static final double RECT_RX = FIELD_WIDTH/5, RECT_RY = FIELD_HEIGHT/2;
    private static final double FIELD_CX = FIELD_WIDTH/2, FIELD_CY = FIELD_HEIGHT/2;
    private static final double RECT_X_MIN = FIELD_CX-RECT_RX, RECT_X_MAX = FIELD_CX+RECT_RX,
                                RECT_Y_MIN = FIELD_CY-RECT_RY, RECT_Y_MAX = FIELD_CY+RECT_RY;

    public static boolean excludePoint(double x, double y) {
        return x < RECT_X_MIN || x > RECT_X_MAX ||
               y < RECT_Y_MIN || y > RECT_Y_MAX;
    }
}