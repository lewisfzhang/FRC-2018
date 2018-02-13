package com.team254.frc2018.loops;

import com.team254.frc2018.Kinematics;
import com.team254.frc2018.RobotState;
import com.team254.frc2018.subsystems.Drive;
import com.team254.frc2018.subsystems.FollowerWheels;
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
    private FollowerWheels followers_ = FollowerWheels.getInstance();
    private double left_encoder_prev_distance_ = 0.0;
    private double right_encoder_prev_distance_ = 0.0;
    private double back_encoder_prev_distance_ = 0.0;

    @Override
    public synchronized void onStart(double timestamp) {
        left_encoder_prev_distance_ = followers_.getLeftDistance();
        right_encoder_prev_distance_ = followers_.getRightDistance();
        back_encoder_prev_distance_ = followers_.getRearDistance();
    }

    @Override
    public synchronized void onLoop(double timestamp) {
        final double left_distance = followers_.getLeftDistance();
        final double right_distance = followers_.getRightDistance();
        final double back_distance = followers_.getRearDistance();
        final double delta_left = left_distance - left_encoder_prev_distance_;
        final double delta_right = right_distance - right_encoder_prev_distance_;
        final double delta_back = back_distance - back_encoder_prev_distance_;
        final Rotation2d gyro_angle = drive_.getHeading();
        final Twist2d odometry_velocity = robot_state_.generateOdometryFromSensors(
                delta_left, delta_right, delta_back, gyro_angle);
        final Twist2d predicted_velocity = Kinematics.forwardKinematics(followers_.getLeftVelocity(),
                followers_.getRightVelocity(), followers_.getRearVelocity());
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

