package com.team254.frc2018.planners;

import com.team254.frc2018.Constants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.physics.DCMotorTransmission;
import com.team254.lib.physics.DifferentialDrive;
import com.team254.lib.trajectory.*;
import com.team254.lib.trajectory.timing.DifferentialDriveDynamicsConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;
import com.team254.lib.trajectory.timing.TimingUtil;
import com.team254.lib.util.CSVWritable;
import com.team254.lib.util.Units;
import com.team254.lib.util.Util;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

public class DriveMotionPlanner implements CSVWritable {
    private static final double kMaxDx = 2.0;
    private static final double kMaxDy = 0.25;
    private static final double kMaxDTheta = Math.toRadians(5.0);

    public enum FollowerType {
        FEEDFORWARD_ONLY,
        PURE_PURSUIT,
        PID
    }

    FollowerType mFollowerType = FollowerType.PURE_PURSUIT;

    public void setFollowerType(FollowerType type) {
        mFollowerType = type;
    }

    final DifferentialDrive mModel;

    TrajectoryIterator<TimedState<Pose2dWithCurvature>> mCurrentTrajectory;
    boolean mIsReversed = false;
    double mLastTime = Double.POSITIVE_INFINITY;
    TimedState<Pose2dWithCurvature> mSetpoint = new TimedState<>(Pose2dWithCurvature.identity());
    Pose2d mError = Pose2d.identity();
    Output mOutput = new Output();

    public DriveMotionPlanner() {
        final DCMotorTransmission transmission = new DCMotorTransmission(
                1.0 / Constants.kDriveKvStraight,
                1.0 / Constants.kDriveKvTurnInPlace,
                Units.inches_to_meters(Constants.kDriveWheelRadiusInches) * Units.inches_to_meters(Constants
                        .kDriveWheelRadiusInches) * Constants.kRobotLinearInertia / (2.0 * Constants.kDriveKaStraight),
                Units.inches_to_meters(Constants.kDriveWheelRadiusInches) * Constants.kRobotAngularInertia / (2.0 * Constants.kDriveKaTurnInPlace),
                Constants.kDriveVInterceptStraight,
                Constants.kDriveVInterceptTurnInPlace);
        mModel = new DifferentialDrive(
                Constants.kRobotLinearInertia,
                Constants.kRobotAngularInertia,
                Units.inches_to_meters(Constants.kDriveWheelDiameterInches / 2.0),
                Units.inches_to_meters(Constants.kDriveWheelTrackWidthInches / 2.0 * Constants.kTrackScrubFactor),
                transmission, transmission
        );
    }

    public void setTrajectory(final TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        mCurrentTrajectory = trajectory;
        mSetpoint = trajectory.getState();
        for (int i = 0; i < trajectory.trajectory().length(); ++i) {
            if (trajectory.trajectory().getState(i).velocity() > Util.kEpsilon) {
                mIsReversed = false;
                break;
            } else if (trajectory.trajectory().getState(i).velocity() < -Util.kEpsilon) {
                mIsReversed = true;
                break;
            }
        }
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        List<Pose2d> waypoints_maybe_flipped = waypoints;
        final Pose2d flip = Pose2d.fromRotation(new Rotation2d(-1, 0, false));
        // TODO re-architect the spline generator to support reverse.
        if (reversed) {
            waypoints_maybe_flipped = new ArrayList<>(waypoints.size());
            for (int i = 0; i < waypoints.size(); ++i) {
                waypoints_maybe_flipped.add(waypoints.get(i).transformBy(flip));
            }
        }

        // Create a trajectory from splines.
        Trajectory<Pose2dWithCurvature> trajectory = TrajectoryUtil.trajectoryFromSplineWaypoints(
                waypoints_maybe_flipped, kMaxDx, kMaxDy, kMaxDTheta);

        if (reversed) {
            List<Pose2dWithCurvature> flipped = new ArrayList<>(trajectory.length());
            for (int i = 0; i < trajectory.length(); ++i) {
                flipped.add(new Pose2dWithCurvature(trajectory.getState(i).getPose().transformBy(flip), -trajectory.getState(i).getCurvature()));
            }
            trajectory = new Trajectory<>(flipped);
        }
        // Create the constraint that the robot must be able to traverse the trajectory without ever applying more
        // than the specified voltage.
        final DifferentialDriveDynamicsConstraint<Pose2dWithCurvature> drive_constraints = new
                DifferentialDriveDynamicsConstraint<>(mModel, max_voltage);
        List<TimingConstraint<Pose2dWithCurvature>> all_constraints = new ArrayList<>();
        all_constraints.add(drive_constraints);
        if (constraints != null) {
            all_constraints.addAll(constraints);
        }
        // Generate the timed trajectory.
        Trajectory<TimedState<Pose2dWithCurvature>> timed_trajectory = TimingUtil.timeParameterizeTrajectory(reversed, new
                DistanceView<>(trajectory), kMaxDx, all_constraints, 0.0, 0.0, max_vel, max_accel);
        return timed_trajectory;
    }

    @Override
    public String toCSV() {
        DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format(mOutput.left_velocity) + "," + fmt.format(mOutput.right_velocity) + "," + fmt.format
                (mOutput.left_feedforward_voltage) + "," + fmt.format(mOutput.right_feedforward_voltage) + "," +
                mSetpoint.toCSV();
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

        public void flip() {
            double tmp_left_velocity = left_velocity;
            left_velocity = -right_velocity;
            right_velocity = -tmp_left_velocity;

            double tmp_left_feedforward = left_feedforward_voltage;
            left_feedforward_voltage = -right_feedforward_voltage;
            right_feedforward_voltage = -tmp_left_feedforward;
        }
    }

    protected Output updatePID(DifferentialDrive.DriveDynamics dynamics, Pose2d current_state) {DifferentialDrive.ChassisState adjusted_velocity = new DifferentialDrive.ChassisState();
        // Feedback on longitudinal error (distance).
        final double kPathKX = 5.0;
        final double kPathKY = 1.0;
        final double kPathKTheta = 5.0;
        adjusted_velocity.linear = dynamics.chassis_velocity.linear + kPathKX * Units.inches_to_meters
                (mError.getTranslation().x());
        adjusted_velocity.angular = dynamics.chassis_velocity.angular + dynamics.chassis_velocity.linear * kPathKY * Units.inches_to_meters(mError.getTranslation().y()) + kPathKTheta * mError.getRotation().getRadians();

        double curvature = adjusted_velocity.angular / adjusted_velocity.linear;
        if (Double.isInfinite(curvature)) {
            adjusted_velocity.linear = 0.0;
            adjusted_velocity.angular = dynamics.chassis_velocity.angular;
        }

        // Compute adjusted left and right wheel velocities.
        final DifferentialDrive.WheelState wheel_velocities = mModel.solveInverseKinematics(adjusted_velocity);
        final double left_voltage = dynamics.voltage.left + (wheel_velocities.left - dynamics.wheel_velocity
                .left) / mModel.left_transmission().speed_per_volt(curvature);
        final double right_voltage = dynamics.voltage.right + (wheel_velocities.right - dynamics.wheel_velocity
                .right) / mModel.right_transmission().speed_per_volt(curvature);

        return new Output(wheel_velocities.left, wheel_velocities.right, left_voltage, right_voltage);
    }

    protected Output updatePurePursuit(DifferentialDrive.DriveDynamics dynamics, Pose2d current_state) {
        double lookahead_time = Constants.kPathLookaheadTime;
        final double kLookaheadSearchDt = 0.01;
        TimedState<Pose2dWithCurvature> lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
        double actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
        while (actual_lookahead_distance < Constants.kPathMinLookaheadDistance &&
                mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
            lookahead_time += kLookaheadSearchDt;
            lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
            actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
        }
        if (actual_lookahead_distance < Constants.kPathMinLookaheadDistance) {
            lookahead_state = new TimedState<>(new Pose2dWithCurvature(lookahead_state.state()
                    .getPose().transformBy(Pose2d.fromTranslation(new Translation2d(
                            (mIsReversed ? -1.0 : 1.0) *(Constants.kPathMinLookaheadDistance - actual_lookahead_distance), 0.0))), 0.0), lookahead_state.t()
                    , lookahead_state.velocity(), lookahead_state.acceleration());
        }

        DifferentialDrive.ChassisState adjusted_velocity = new DifferentialDrive.ChassisState();
        // Feedback on longitudinal error (distance).
        adjusted_velocity.linear = dynamics.chassis_velocity.linear + Constants.kPathKX * Units.inches_to_meters
                (mError.getTranslation().x());

        // Use pure pursuit to peek ahead along the trajectory and generate a new curvature.
        final PurePursuitController.Arc<Pose2dWithCurvature> arc = new PurePursuitController.Arc<>(current_state,
                lookahead_state.state());

        double curvature = 1.0 / Units.inches_to_meters(arc.radius);
        if (Double.isInfinite(curvature)) {
            adjusted_velocity.linear = 0.0;
            adjusted_velocity.angular = dynamics.chassis_velocity.angular;
        } else {
            adjusted_velocity.angular = curvature * dynamics.chassis_velocity.linear;
        }

        // Compute adjusted left and right wheel velocities.
        final DifferentialDrive.WheelState wheel_velocities = mModel.solveInverseKinematics(adjusted_velocity);
        final double left_voltage = dynamics.voltage.left + (wheel_velocities.left - dynamics.wheel_velocity
                .left) / mModel.left_transmission().speed_per_volt(curvature);
        final double right_voltage = dynamics.voltage.right + (wheel_velocities.right - dynamics.wheel_velocity
                .right) / mModel.right_transmission().speed_per_volt(curvature);

        return new Output(wheel_velocities.left, wheel_velocities.right, left_voltage, right_voltage);
    }

    public Output update(double timestamp, Pose2d current_state) {
        if (mCurrentTrajectory == null) return new Output();

        if (mCurrentTrajectory.getProgress() == 0.0 && !Double.isFinite(mLastTime)) {
            mLastTime = timestamp;
        }

        final double t = timestamp - mLastTime;
        mLastTime = timestamp;
        TrajectorySamplePoint<TimedState<Pose2dWithCurvature>> sample_point = mCurrentTrajectory.advance(t);
        mSetpoint = sample_point.state();

        TimedState<Pose2dWithCurvature> prev_pt = mCurrentTrajectory.trajectory().getState(sample_point.index_floor());
        TimedState<Pose2dWithCurvature> next_pt = mCurrentTrajectory.trajectory().getState(sample_point.index_ceil());
        double dcurvature_ds = (mIsReversed ? -1.0 : 1.0) * Units.meters_to_inches(next_pt.state().getCurvature() - prev_pt.state().getCurvature()) /
                Units.inches_to_meters(prev_pt.state().distance(next_pt.state()));
        if (Double.isNaN(dcurvature_ds)) {
            dcurvature_ds = 0.0;
        }
        // Compute the derivative of curvature.
        if (!mCurrentTrajectory.isDone()) {
            // Generate feedforward voltages.
            final double velocity_m = Units.inches_to_meters(mSetpoint.velocity());
            final double curvature_m = 1.0 / Units.inches_to_meters(1.0 / mSetpoint.state().getCurvature());
            final double acceleration_m = Units.inches_to_meters(mSetpoint.acceleration());
            final DifferentialDrive.DriveDynamics dynamics = mModel.solveInverseDynamics(
                    new DifferentialDrive.ChassisState(velocity_m, velocity_m * curvature_m),
                    new DifferentialDrive.ChassisState(acceleration_m,
                            acceleration_m * curvature_m + velocity_m * velocity_m * dcurvature_ds));
            mError = current_state.inverse().transformBy(mSetpoint.state().getPose());

            // HACK scale up angular velocity for feedforward voltage.
            final double kScalingFactor = 2.0;
            final double scaled_curvature_m = curvature_m * kScalingFactor;
            DifferentialDrive.DriveDynamics hack_dynamics = mModel.solveInverseDynamics(
                    new DifferentialDrive.ChassisState(velocity_m, velocity_m * scaled_curvature_m),
                    new DifferentialDrive.ChassisState(acceleration_m,
                            acceleration_m * scaled_curvature_m + velocity_m * velocity_m * dcurvature_ds));
            dynamics.voltage = hack_dynamics.voltage;

            if (mFollowerType == FollowerType.FEEDFORWARD_ONLY) {
                mOutput = new Output(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right, dynamics.voltage
                        .left, dynamics.voltage.right);
            } else if (mFollowerType == FollowerType.PURE_PURSUIT) {
                mOutput = updatePurePursuit(dynamics, current_state);
            } else if (mFollowerType == FollowerType.PID) {
                mOutput = updatePID(dynamics, current_state);
            }
        } else {
            // TODO Possibly switch to a pose stabilizing controller?
            mOutput = new Output();
        }
        return mOutput;
    }

    public boolean isDone() {
        return mCurrentTrajectory != null && mCurrentTrajectory.isDone();
    }

    public Pose2d error() {
        return mError;
    }

    public TimedState<Pose2dWithCurvature> setpoint() {
        return mSetpoint;
    }
}
