package com.team254.frc2018.planners;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.team254.frc2018.Constants;
import com.team254.frc2018.subsystems.Drive;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.physics.DCMotorTransmission;
import com.team254.lib.physics.DifferentialDrive;
import com.team254.lib.trajectory.*;
import com.team254.lib.trajectory.timing.DifferentialDriveDynamicsConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;
import com.team254.lib.trajectory.timing.TimingUtil;
import com.team254.lib.util.Units;

import java.util.ArrayList;
import java.util.List;

public class MotionProfilePlanner {
    private static final double kMaxDx = 2.0;
    private static final double kMaxDy = 0.25;
    private static final double kMaxDTheta = Math.toRadians(5.0);
    private static final double kDt = 0.01;  // s

    final DifferentialDrive mModel;
    boolean mIsReversed = false;
    private Trajectory<TimedState<Pose2dWithCurvature>> mTrajectory;
    private TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTimeIterator;
    private DistanceView<TimedState<Pose2dWithCurvature>> mDistanceView;
    private IMotorController mMaster;
    private SetValueMotionProfile mSetValue = SetValueMotionProfile.Disable;
    private MotionProfileStatus mStatus = new MotionProfileStatus();
    private double mHeadingUnwrapped;  // degrees

    private enum State {
        INACTIVE,
        PREFILL,
        ACTIVE,
        DONE
    }

    private State mState;

    private static final double kLookahead = 0.1;  // seconds
    private static final int kMinPoints = (int) Math.ceil(kLookahead / kDt);

    public MotionProfilePlanner(IMotorController master) {
        mMaster = master;
        mMaster.changeMotionControlFramePeriod(10);
        mMaster.configMotionProfileTrajectoryPeriod((int) (kDt * 1000.0), 100);
        mState = State.INACTIVE;
        final DCMotorTransmission transmission = new DCMotorTransmission(1.0 / Constants.kDriveKv,
                Units.inches_to_meters(Constants.kDriveWheelRadiusInches) * Units.inches_to_meters(Constants
                        .kDriveWheelRadiusInches) *
                        Constants.kRobotLinearInertia / (2.0 * Constants.kDriveKa), Constants.kDriveVIntercept);
        mModel = new DifferentialDrive(
                Constants.kRobotLinearInertia,
                Constants.kRobotAngularInertia,
                Units.inches_to_meters(Constants.kDriveWheelDiameterInches / 2.0),
                Units.inches_to_meters(Constants.kDriveWheelTrackWidthInches / 2.0 * Constants.kTrackScrubFactor),
                transmission, transmission);
    }

    public void setTrajectory(final Trajectory<TimedState<Pose2dWithCurvature>> trajectory, boolean
            isReversed) {
        mTrajectory = trajectory;
        mTimeIterator = new TrajectoryIterator<>(new TimedView<>(trajectory));
        mDistanceView = new DistanceView<>(trajectory);
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

    public void update() {
        switch (mState) {
            case INACTIVE:
                // Start prefilling.
                maybeFill();
                mSetValue = SetValueMotionProfile.Disable;
                mState = State.PREFILL;
                break;
            case PREFILL:
                // Check if prefill is done.
                if (mStatus.topBufferCnt > kMinPoints) {
                    mSetValue = SetValueMotionProfile.Enable;
                    mState = State.ACTIVE;
                }
                break;
            case ACTIVE:
                // Monitor progress.
                maybeFill();
                if (mStatus.isLast && mStatus.activePointValid) {
                    mSetValue = SetValueMotionProfile.Hold;
                    mState = State.DONE;
                }
                break;
            case DONE:
                // Don't do anything.
                break;
        }
        mMaster.getMotionProfileStatus(mStatus);
    }

    private void maybeFill() {
        int count = mStatus.topBufferCnt;
        if (mState == State.INACTIVE) {
            mMaster.clearMotionProfileTrajectories();
        }
        TrajectoryPoint point = new TrajectoryPoint();
        while (count < kMinPoints && !mTimeIterator.isDone()) {
            TrajectorySamplePoint<TimedState<Pose2dWithCurvature>> goal;
            if (mTimeIterator.getProgress() == 0.0) {
                // First point.
                goal = mTimeIterator.advance(0.0);
                mHeadingUnwrapped = goal.state().state().getRotation().getDegrees();
            } else {
                goal = mTimeIterator.advance(kDt);
            }
            // Convert inches to encoder ticks.
            point.position = (mDistanceView.distance_at_index(goal.index_floor()) + mTrajectory.getState(goal
                    .index_floor()).distance(goal.state())) / (Math.PI * Constants.kDriveWheelDiameterInches) * 4096.0;
            // Convert inches/s to encoder ticks/100ms.
            point.velocity = goal.state().velocity() / (10.0 * Math.PI * Constants.kDriveWheelDiameterInches) * 4096.0;
            mHeadingUnwrapped += Rotation2d.fromDegrees(mHeadingUnwrapped).inverse().rotateBy(goal.state().state()
                    .getRotation()).getDegrees();
            point.auxiliaryPos = mHeadingUnwrapped * (Constants.kDriveTurnUnits / 360.0);

            point.profileSlotSelect0 = Drive.kDistanceSlot;
            point.profileSlotSelect1 = Drive.kTurnSlot;
            point.isLastPoint = mTimeIterator.isDone();
            mMaster.pushMotionProfileTrajectory(point);
            ++count;
        }
    }

    public void reset() {
        mMaster.clearMotionProfileTrajectories();
        mSetValue = SetValueMotionProfile.Disable;
        mState = State.INACTIVE;
    }

    public SetValueMotionProfile getSetValue() {
        return mSetValue;
    }

    public boolean isDone() {
        return mState == State.DONE;
    }
}
