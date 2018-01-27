package com.team254.frc2018.lidar;

import com.team254.lib.util.math.Translation2d;

/**
 * Represents a single point from the lidar
 */
class LidarPoint {
    double timestamp;
    double angle;
    double distance;

    public LidarPoint(double timestamp, double angle, double distance) {
        this.timestamp = timestamp;
        this.angle = angle;
        this.distance = distance;
    }

    public Translation2d toCartesian() {
        double radians = Math.toRadians(angle);
        //todo: fuse robot pose
        return new Translation2d(Math.cos(radians) * distance, Math.sin(radians) * distance);
    }
}