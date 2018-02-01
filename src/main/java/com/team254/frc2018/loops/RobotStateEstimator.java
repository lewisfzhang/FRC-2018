package com.team254.frc2018.loops;

import com.team254.frc2018.Kinematics;
import com.team254.frc2018.RobotState;
import com.team254.frc2018.subsystems.Drive;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;

public class RobotStateEstimator implements Loop {
    static RobotStateEstimator instance_ = new RobotStateEstimator();

    public static RobotStateEstimator getInstance() {
        return instance_;
    }

    RobotStateEstimator() {
    }

    private RobotState robot_state_ = RobotState.getInstance();
    private Drive drive_ = Drive.getInstance();
    private double left_encoder_prev_distance_ = 0.0;
    private double right_encoder_prev_distance_ = 0.0;
    private double back_encoder_prev_distance_ = 0.0;

    @Override
    public synchronized void onStart(double timestamp) {
        left_encoder_prev_distance_ = drive_.getLeftFollowerDistance();
        right_encoder_prev_distance_ = drive_.getRightFollowerDistance();
        back_encoder_prev_distance_ = drive_.getBackFollowerDistance();
    }

    @Override
    public synchronized void onLoop(double timestamp) {
        final double left_distance = drive_.getLeftFollowerDistance();
        final double right_distance = drive_.getRightFollowerDistance();
        final double back_distance = drive_.getBackFollowerDistance();
        final double delta_left = left_distance - left_encoder_prev_distance_;
        final double delta_right = right_distance - right_encoder_prev_distance_;
        final double delta_back = back_distance - back_encoder_prev_distance_;
        final Rotation2d gyro_angle = drive_.getHeading();
        final Twist2d odometry_velocity = robot_state_.generateOdometryFromSensors(
                delta_left, delta_right, delta_back, gyro_angle);
        final Twist2d predicted_velocity = Kinematics.forwardKinematics(drive_.getLeftFollowerVelocity(),
                drive_.getRightFollowerVelocity(), drive_.getBackFollowerVelocity());
        robot_state_.addObservations(timestamp, odometry_velocity, predicted_velocity);
        left_encoder_prev_distance_ = left_distance;
        right_encoder_prev_distance_ = right_distance;
        back_encoder_prev_distance_ = back_distance;
    }

    @Override
    public void onStop(double timestamp) {
        // no-op
    }

}

