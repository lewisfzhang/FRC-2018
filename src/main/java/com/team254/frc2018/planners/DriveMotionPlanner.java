package com.team254.frc2018.planners;

import com.team254.frc2018.Constants;
import com.team254.lib.geometry.*;
import com.team254.lib.physics.DCMotorTransmission;
import com.team254.lib.physics.DifferentialDrive;
import com.team254.lib.trajectory.DistanceView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.TrajectoryUtil;
import com.team254.lib.trajectory.timing.DifferentialDriveDynamicsConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;
import com.team254.lib.trajectory.timing.TimingUtil;
import com.team254.lib.util.CSVWritable;
import com.team254.lib.util.Units;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

public class DriveMotionPlanner implements CSVWritable {
    private static final double kMaxDx = 2.0;
    private static final double kMaxDy = 0.25;
    private static final double kMaxDTheta = Math.toRadians(5.0);

    final DifferentialDrive mModel;

    TrajectoryIterator<TimedState<Pose2dWithCurvature>> mCurrentTrajectory;
    boolean mIsReversed = false;
    double mLastTime = Double.POSITIVE_INFINITY;
    TimedState<Pose2dWithCurvature> mSetpoint = new TimedState<>(Pose2dWithCurvature.identity());
    Pose2d mError = Pose2d.identity();
    Output mOutput = new Output();

    public DriveMotionPlanner() {
        final DCMotorTransmission transmission = new DCMotorTransmission(1.0 / Constants.kDriveKv,
                Units.inches_to_meters(Constants.kDriveWheelRadiusInches) * Units.inches_to_meters(Constants
                        .kDriveWheelRadiusInches) *
                        Constants.kRobotLinearInertia / (2.0 * Constants.kDriveKa), Constants.kDriveVIntercept);
        mModel = new DifferentialDrive(
                Constants.kRobotLinearInertia,
                Constants.kRobotAngularInertia,
                Units.inches_to_meters(Constants.kDriveWheelDiameterInches / 2.0),
                Units.inches_to_meters(Constants.kDriveWheelTrackWidthInches / 2.0 * Constants.kTrackScrubFactor),
                transmission, transmission
        );
    }

    public void setTrajectory(final TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory, boolean isReversed) {
        mCurrentTrajectory = trajectory;
        mSetpoint = trajectory.getState();
        mIsReversed = isReversed;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
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
        List<TimingConstraint<Pose2dWithCurvature>> all_constraints = new ArrayList<>();
        all_constraints.add(drive_constraints);
        if (constraints != null) {
            all_constraints.addAll(constraints);
        }
        // Generate the timed trajectory.
        Trajectory<TimedState<Pose2dWithCurvature>> timed_trajectory = TimingUtil.timeParameterizeTrajectory(new
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
    }

    public Output update(double timestamp, Pose2d current_state) {
        if (mCurrentTrajectory == null) return new Output();

        if (mCurrentTrajectory.getProgress() == 0.0 && !Double.isFinite(mLastTime)) {
            mLastTime = timestamp;
        }
        if(mIsReversed) {
            current_state = current_state.transformBy(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180)));
        }

        final double t = timestamp - mLastTime;
        mLastTime = timestamp;
        final double kLookaheadTime = 0.5;
        mSetpoint = mCurrentTrajectory.advance(t).state();
        final TimedState<Pose2dWithCurvature> lookahead_state = mCurrentTrajectory.preview(kLookaheadTime).state();
        if (!mCurrentTrajectory.isDone()) {
            // Generate feedforward voltages.
            final DifferentialDrive.DriveDynamics dynamics = mModel.solveInverseDynamics(
                    new DifferentialDrive.ChassisState(Units.inches_to_meters(mSetpoint.velocity()),
                            mSetpoint.velocity() * mSetpoint.state().getCurvature()),
                    new DifferentialDrive.ChassisState(Units.inches_to_meters(mSetpoint.acceleration()),
                            mSetpoint.acceleration() * mSetpoint.state().getCurvature()));


            mError = current_state.inverse().transformBy(mSetpoint.state().getPose());
            final Pose2d lookahead_error = current_state.inverse().transformBy(lookahead_state.state().getPose());
            DifferentialDrive.ChassisState adjusted_velocity = new DifferentialDrive.ChassisState();
            adjusted_velocity.linear = dynamics.chassis_velocity.linear + Constants
                    .kPathKX * Units.inches_to_meters(mError.getTranslation().x());

            double curvature = dynamics.chassis_velocity.angular / dynamics.chassis_velocity.linear;
            if (Double.isNaN(curvature)) {
                curvature = 0.0;
            }
            curvature +=
                    (dynamics.chassis_velocity.linear >= 0.0 ? 1.0 : -1.0) * Constants.kPathKY * Units
                            .inches_to_meters(lookahead_error
                                    .getTranslation().y()) + Constants.kPathKTheta * lookahead_error.getRotation()
                            .getRadians();
            adjusted_velocity.angular = curvature * dynamics.chassis_velocity.linear;


            // Compute adjusted left and right wheel velocities.
            final DifferentialDrive.WheelState wheel_velocities = mModel.solveInverseKinematics(adjusted_velocity);
            final double left_voltage = dynamics.voltage.left + (wheel_velocities.left - dynamics.wheel_velocity
                    .left) / mModel.left_transmission().speed_per_volt();
            final double right_voltage = dynamics.voltage.right + (wheel_velocities.right - dynamics.wheel_velocity
                    .right) / mModel.right_transmission().speed_per_volt();

            // System.out.println(dynamics.toCSV());
            if(mIsReversed) {
                mOutput = new Output(-wheel_velocities.right, -wheel_velocities.left, -right_voltage, -left_voltage);
            } else {
                mOutput = new Output(wheel_velocities.left, wheel_velocities.right, left_voltage, right_voltage);
            }
        } else {
            // Possibly switch to a pose stabilizing controller?
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
