package com.team254.frc2018.planners;

import com.team254.frc2018.Constants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.physics.DCMotorTransmission;
import com.team254.lib.physics.DifferentialDrive;
import com.team254.lib.trajectory.DistanceView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.TrajectoryUtil;
import com.team254.lib.trajectory.timing.DifferentialDriveDynamicsConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingUtil;
import com.team254.lib.util.Units;

import java.util.Arrays;
import java.util.List;

public class DriveMotionPlanner {
    private static final double kMaxDx = 2.0;
    private static final double kMaxDy = 0.25;
    private static final double kMaxDTheta = Math.toRadians(5.0);

    final DifferentialDrive mModel;

    TrajectoryIterator<TimedState<Pose2dWithCurvature>> mCurrentTrajectory;
    double mLastTime = Double.POSITIVE_INFINITY;

    public DriveMotionPlanner() {
        final DCMotorTransmission transmission = new DCMotorTransmission(1.0 / Constants.kDriveKv,
                Units.inches_to_meters(Constants.kDriveWheelRadiusInches) * Units.inches_to_meters(Constants
                        .kDriveWheelRadiusInches) *
                        Constants.kRobotLinearInertia / (2.0 * Constants.kDriveKa), Constants.kDriveVIntercept);
        mModel = new DifferentialDrive(
                Constants.kRobotLinearInertia,
                Constants.kRobotAngularInertia,
                Units.inches_to_meters(Constants.kDriveWheelDiameterInches / 2.0),
                Units.inches_to_meters(Constants.kDriveWheelTrackWidthInches / 2.0),
                transmission, transmission
        );
    }

    public void setTrajectory(final TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        mCurrentTrajectory = trajectory;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            final List<Pose2d> waypoints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        // Create a trajectory from splines.
        final Trajectory<Pose2dWithCurvature> trajectory = TrajectoryUtil.trajectoryFromSplineWaypoints(waypoints,
                kMaxDx,
                kMaxDy, kMaxDTheta);
        // Create the constraint that the robot must be able to traverse the trajectory without ever applying more
        // than the specified voltage.
        final DifferentialDriveDynamicsConstraint<Pose2dWithCurvature> drive_constraints = new
                DifferentialDriveDynamicsConstraint<>(mModel, max_voltage);

        // Generate the timed trajectory.
        Trajectory<TimedState<Pose2dWithCurvature>> timed_trajectory = TimingUtil.timeParameterizeTrajectory(new
                        DistanceView<>(trajectory), kMaxDx, Arrays.asList(drive_constraints),
                0.0, 0.0, max_vel, max_accel);
        return timed_trajectory;
    }

    public static class Output {
        public Output() {
        }

        public Output(double left_velocity, double right_velocity, double left_feedforward_voltage, double
                right_feedforward_voltage) {
            this.left_velocity = left_velocity;
            this.right_velocity = right_velocity;
            this.left_feedforward_voltage = left_feedforward_voltage;
            this.right_feedforward_voltage = right_feedforward_voltage;
        }

        public double left_velocity;  // rad/s
        public double right_velocity;  // rad/s

        public double left_feedforward_voltage;
        public double right_feedforward_voltage;
    }

    public Output update(double timestamp, final Pose2d current_state) {
        if (mCurrentTrajectory == null) return new Output();

        if (mCurrentTrajectory.getProgress() == 0.0 && !Double.isFinite(mLastTime)) {
            mLastTime = timestamp;
        }

        final double t = timestamp - mLastTime;
        mLastTime = timestamp;
        final TimedState<Pose2dWithCurvature> goal = mCurrentTrajectory.advance(t).state();
        if (!mCurrentTrajectory.isDone()) {
            // Generate feedforward voltages.
            final DifferentialDrive.DriveDynamics dynamics = mModel.solveInverseDynamics(
                    new DifferentialDrive.ChassisState(Units.inches_to_meters(goal.velocity()),
                            goal.velocity() * goal.state().getCurvature()),
                    new DifferentialDrive.ChassisState(Units.inches_to_meters(goal.acceleration()),
                            goal.acceleration() * goal.state().getCurvature()));

            final Pose2d robot_to_goal = current_state.inverse().transformBy(goal.state().getPose());
            DifferentialDrive.ChassisState adjusted_velocity = new DifferentialDrive.ChassisState();
            adjusted_velocity.linear = dynamics.chassis_velocity.linear * robot_to_goal.getRotation().cos() + Constants
                    .kPathKX * robot_to_goal.getTranslation().x();
            adjusted_velocity.angular = dynamics.chassis_velocity.angular +
                    (dynamics.chassis_velocity.linear >= 0.0 ? 1.0 : 0.0) * Constants.kPathKY * robot_to_goal
                            .getTranslation().y() + Constants.kPathKTheta * robot_to_goal.getRotation().getRadians();

            // Compute adjusted left and right wheel velocities.
            final DifferentialDrive.WheelState wheel_velocities = mModel.solveInverseKinematics(adjusted_velocity);
            final double left_voltage = dynamics.voltage.left + (wheel_velocities.left - dynamics.wheel_velocity.left) / mModel.left_transmission().speed_per_volt();
            final double right_voltage = dynamics.voltage.right + (wheel_velocities.right - dynamics.wheel_velocity.right) / mModel.right_transmission().speed_per_volt();

            // System.out.println(dynamics.toCSV());
            return new Output(wheel_velocities.left, wheel_velocities.right, left_voltage, right_voltage);
        } else {
            // Possibly switch to a pose stabilizing controller?
            return new Output();
        }
    }

    public boolean isDone() {
        return mCurrentTrajectory != null && mCurrentTrajectory.isDone();
    }
}
