package com.team254.frc2018;

import com.team254.frc2018.subsystems.Drive;
import com.team254.frc2018.subsystems.Limelight;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class RobotState {
    private static RobotState instance_ = new RobotState();

    public static RobotState getInstance() {
        return instance_;
    }

    private static final int kObservationBufferSize = 100;
    private static final double kMinStability = 0.5;
    private final double exchange_differential_height_ = 24.5;
    private final Rotation2d limelight_pitch_ = Rotation2d.fromDegrees(32.0);
    private GoalTracker exchange_tracker_ = new GoalTracker();

    private static final Pose2d kVehicleToLidar = new Pose2d(
            new Translation2d(Constants.kLidarXOffset, Constants.kLidarYOffset), Rotation2d.fromDegrees(Constants
            .kLidarYawAngleDegrees));

    private static final Pose2d kVehicleToLidar = new Pose2d(
            new Translation2d(Constants.kLidarXOffset, Constants.kLidarYOffset), Rotation2d.fromDegrees(Constants
            .kLidarYawAngleDegrees));

    // FPGATimestamp -> RigidTransform2d or Rotation2d
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private Twist2d vehicle_velocity_predicted_;
    private Twist2d vehicle_velocity_measured_;
    private double distance_driven_;

    private RobotState() {
        reset(0, new Pose2d());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        Drive.getInstance().setHeading(initial_field_to_vehicle.getRotation());
        vehicle_velocity_predicted_ = Twist2d.identity();
        vehicle_velocity_measured_ = Twist2d.identity();
        distance_driven_ = 0.0;
    }

    public synchronized void resetDistanceDriven() {
        distance_driven_ = 0.0;
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        return getLatestFieldToVehicle().getValue()
                .transformBy(Pose2d.exp(vehicle_velocity_predicted_.scaled(lookahead_time)));
    }

    public synchronized Pose2d getFieldToLidar(double timestamp) {
        return getFieldToVehicle(timestamp).transformBy(kVehicleToLidar);
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Twist2d measured_velocity,
                                             Twist2d predicted_velocity) {
        addFieldToVehicleObservation(timestamp,
                Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), measured_velocity));
        vehicle_velocity_measured_ = measured_velocity;
        vehicle_velocity_predicted_ = predicted_velocity;
    }

    public synchronized Twist2d generateOdometryFromSensors(double left_encoder_delta_distance, double
            right_encoder_delta_distance, Rotation2d current_gyro_angle) {
        final Pose2d last_measurement = getLatestFieldToVehicle().getValue();
        final Twist2d delta = Kinematics.forwardKinematics(last_measurement.getRotation(),
                left_encoder_delta_distance, right_encoder_delta_distance,
                current_gyro_angle);
        distance_driven_ += delta.dx; //do we care about dy here?
        return delta;
    }

    public synchronized double getDistanceDriven() {
        return distance_driven_;
    }

    public synchronized Twist2d getPredictedVelocity() {
        return vehicle_velocity_predicted_;
    }

    public synchronized Twist2d getMeasuredVelocity() {
        return vehicle_velocity_measured_;
    }

    public void outputToSmartDashboard() {
        Pose2d odometry = getLatestFieldToVehicle().getValue();
        SmartDashboard.putNumber("Robot Pose X", odometry.getTranslation().x());
        SmartDashboard.putNumber("Robot Pose Y", odometry.getTranslation().y());
        SmartDashboard.putNumber("Robot Pose Theta", odometry.getRotation().getDegrees());
        SmartDashboard.putNumber("Robot Linear Velocity", vehicle_velocity_measured_.dx);
    }


    public void addVisionUpdate(double timestamp, List<Limelight.TargetInfo> vision_update) {
        if (vision_update.size() != 2)
            return;
        List<Translation2d> positions = new ArrayList<>();
        Pose2d robot_pose = getFieldToVehicle(timestamp).transformBy(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180.0)));
        for (Limelight.TargetInfo target : vision_update) {
            Pose2d robot_to_target = new Pose2d(getTranslationFromTargetInfo(target), Rotation2d.identity());
            positions.add(robot_pose.transformBy(robot_to_target).getTranslation());
        }
        exchange_tracker_.update(timestamp, getTargetPoseFromPositions(positions.get(0), positions.get(1), robot_pose));
    }

    public void addVisionUpdate(double timestamp, Limelight.TargetInfo vision_update) {
        Pose2d robot_pose = getFieldToVehicle(timestamp).transformBy(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180.0)));
        Pose2d robot_to_target = new Pose2d(getTranslationFromTargetInfo(vision_update), Rotation2d.identity());
        Pose2d target_pos = new Pose2d(robot_pose.transformBy(robot_to_target).getTranslation(), Rotation2d.identity());
        exchange_tracker_.update(timestamp, target_pos);
    }

    public synchronized Optional<Pose2d> getFieldToExchange() {
        List<GoalTracker.TrackReport> reports = exchange_tracker_.getTracks();
        if (!reports.isEmpty() && reports.get(0).stability > kMinStability) {
            GoalTracker.TrackReport report = reports.get(0);
            return Optional.of(report.field_to_goal);
        } else {
            return Optional.empty();
        }
    }

    private Translation2d getTranslationFromTargetInfo(Limelight.TargetInfo target) {
        Rotation2d vertical_angle_to_target = (limelight_pitch_.rotateBy(target.verticalAngle));
        double perpendicular_distance = exchange_differential_height_ / vertical_angle_to_target.tan();
        double scalar = perpendicular_distance / target.horizontalAngle.cos();
        return target.horizontalAngle.toTranslation().scale(scalar);
    }

    private static Pose2d getTargetPoseFromPositions(Translation2d left, Translation2d right, Pose2d robot_pose) {
        Translation2d target_center = left.interpolate(right, 0.5);
        Translation2d target_to_robot = new Translation2d(target_center, robot_pose.getTranslation());
        Rotation2d parallel_angle = new Rotation2d(new Translation2d(left, right), true);
        target_to_robot = target_to_robot.rotateBy(parallel_angle.inverse());
        Rotation2d target_direction = Rotation2d.fromDegrees(90.0 * Math.signum(target_to_robot.y()));
        Rotation2d target_rotation = target_direction.rotateBy(parallel_angle);
        return new Pose2d(target_center, target_rotation);
    }
}
