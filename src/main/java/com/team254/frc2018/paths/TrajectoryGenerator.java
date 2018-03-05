package com.team254.frc2018.paths;

import com.team254.frc2018.planners.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;
import com.team254.lib.trajectory.timing.VelocityLimitRegionConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 100.;
    private static final double kMaxAccel = 100.;
    private static final double kMaxVoltage = 9.0;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories(Pose2d startPose) {
        System.out.println("Generating trajectories...");
        mTrajectorySet = new TrajectorySet(startPose);
        System.out.println("Finished trajectory generation");
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    public class TrajectorySet {
        // switch + scale hybrid paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> rightStartToRightScale;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rightScaleToFence;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rightStartToRightSwitch;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rightScaleToLeftFence;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rightStartToLeftScale;
        public final Trajectory<TimedState<Pose2dWithCurvature>> leftScaleToLeftFence;

        // simple switch paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> centerStartToLeftSwitch;
        public final Trajectory<TimedState<Pose2dWithCurvature>> centerStartToRightSwitch;

        // scale only paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> rightFenceToRightScale;
        public final Trajectory<TimedState<Pose2dWithCurvature>> leftFenceToLeftScale;

        private TrajectorySet(Pose2d startPose) {
            rightStartToRightScale = getRightStartToRightScale(startPose);
            rightScaleToFence = getRightScaleToFence(startPose);
            rightStartToRightSwitch = getRightStartToRightSwitch(startPose);
            rightScaleToLeftFence = getRightScaleToLeftFence(startPose);
            rightStartToLeftScale = getRightStartToLeftScale(startPose);
            leftScaleToLeftFence = getLeftScaleToLeftFence(startPose);

            centerStartToLeftSwitch = getCenterStartToLeftSwitch(startPose);
            centerStartToRightSwitch = getCenterStartToRightSwitch(startPose);

            rightFenceToRightScale = getRightFenceToRightScale(startPose);
            leftFenceToLeftScale = getLeftFenceToLeftScale(startPose);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightStartToRightScale(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(120.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(252.0, 22.0), Rotation2d.fromDegrees(20.0)));

            return generateTrajectory(waypoints, null, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightScaleToFence(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(252.0, 22.0), Rotation2d.fromDegrees(200.0)));
            waypoints.add(new Pose2d(new Translation2d(207.0, 43.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(waypoints, null, 30.0, 30.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightStartToRightSwitch(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(startPose);
            waypoints.add(new Pose2d(new Translation2d(120.0, 36.0), Rotation2d.fromDegrees(90.0)));

            return generateTrajectory(waypoints, null, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        //doesn't work
        private Trajectory<TimedState<Pose2dWithCurvature>> getRightScaleToLeftFence(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(252.0, 22.0), Rotation2d.fromDegrees(200.0)));
            waypoints.add(new Pose2d(new Translation2d(220.0, 60.0), Rotation2d.fromDegrees(90.0)));
            waypoints.add(new Pose2d(new Translation2d(225.0, 190.0), Rotation2d.fromDegrees(70.0)));
            waypoints.add(new Pose2d(new Translation2d(220.0, 200.0), Rotation2d.fromDegrees(90.0)));

            List<TimingConstraint<Pose2dWithCurvature>> constraints = new ArrayList<>();
            constraints.add(new VelocityLimitRegionConstraint<>(
                    new Translation2d(160.0, 100.0), new Translation2d(260.0, 250.0), 30.0));
            constraints.add(new VelocityLimitRegionConstraint<>(
                    new Translation2d(160.0, 0), new Translation2d(260.0, 60), 45.0));


            return generateTrajectory(waypoints, null, 45, 45, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightStartToLeftScale(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(160.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(213.0, 60.0), Rotation2d.fromDegrees(90.0)));
            waypoints.add(new Pose2d(new Translation2d(213.0, 170.0), Rotation2d.fromDegrees(90.0)));
            waypoints.add(new Pose2d(new Translation2d(250.0, 200.0), Rotation2d.fromDegrees(0.0)));

            List<TimingConstraint<Pose2dWithCurvature>> constraints = new ArrayList<>();
            constraints.add(new VelocityLimitRegionConstraint<>(
                    new Translation2d(160.0, -60.0), new Translation2d(260.0, 60.0), 45.0));

            constraints.add(new VelocityLimitRegionConstraint<>(
                    new Translation2d(160.0, 170.0), new Translation2d(260.0, 250.0), 45.0));

            return generateTrajectory(waypoints, constraints, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLeftScaleToLeftFence(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(250.0, 200.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(180, 180.0), Rotation2d.fromDegrees(160.0)));

            return generateTrajectory(waypoints, null, 30.0, 30.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getCenterStartToLeftSwitch(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(100, 60.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(waypoints, null, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getCenterStartToRightSwitch(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(100, -46.0), Rotation2d.fromDegrees(-40.0)));

            return generateTrajectory(waypoints, null, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightFenceToRightScale(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(startPose);
            waypoints.add(new Pose2d(new Translation2d(120.0, 0.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(waypoints, null, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLeftFenceToLeftScale(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(startPose);
            waypoints.add(new Pose2d(new Translation2d(120.0, 0.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(waypoints, null, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
    }
}
