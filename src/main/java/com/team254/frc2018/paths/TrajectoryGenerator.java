package com.team254.frc2018.paths;

import com.team254.frc2018.planners.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 100.;
    private static final double kMaxAccel = 100.;
    private static final double kMaxVoltage = 9.0;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories(Pose2d startPose) {
        if(mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet(startPose);
            System.out.println("Finished trajectory generation");
        }
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
        public final Trajectory<TimedState<Pose2dWithCurvature>> rightScaleToRightFence;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rightScaleToRightFence2;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rightStartToRightSwitch;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rightScaleToLeftFence;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rightStartToLeftScale;
        public final Trajectory<TimedState<Pose2dWithCurvature>> leftScaleToLeftFence;
        public final Trajectory<TimedState<Pose2dWithCurvature>> leftScaleToLeftFence2;

        // simple switch paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> centerStartToLeftSwitch;
        public final Trajectory<TimedState<Pose2dWithCurvature>> centerStartToRightSwitch;

        // scale only paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> rightFenceToRightScale;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rightFence2ToRightScale;
        public final Trajectory<TimedState<Pose2dWithCurvature>> leftFenceToLeftScale;
        public final Trajectory<TimedState<Pose2dWithCurvature>> leftFence2ToLeftScale;

        private TrajectorySet(Pose2d startPose) {
            rightStartToRightScale = getRightStartToRightScale(startPose);
            rightScaleToRightFence = getRightScaleToRightFence(startPose);
            rightScaleToRightFence2 = getRightScaleToRightFence2(startPose);
            rightStartToRightSwitch = getRightStartToRightSwitch(startPose);
            rightScaleToLeftFence = getRightScaleToLeftFence(startPose);
            rightStartToLeftScale = getRightStartToLeftScale(startPose);

            leftScaleToLeftFence = getLeftScaleToLeftFence(startPose);
            leftScaleToLeftFence2 = getLeftScaleToLeftFence2(startPose);

            centerStartToLeftSwitch = getCenterStartToLeftSwitch(startPose);
            centerStartToRightSwitch = getCenterStartToRightSwitch(startPose);

            rightFenceToRightScale = getRightFenceToRightScale(startPose);
            rightFence2ToRightScale = getRightFence2ToRightScale(startPose);
            leftFenceToLeftScale = getLeftFenceToLeftScale(startPose);
            leftFence2ToLeftScale = getLeftFence2ToLeftScale(startPose);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightStartToRightScale(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(120.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(250.0, 22.0), Rotation2d.fromDegrees(10.0)));

            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxAccel)), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightScaleToRightFence(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(250.0, 22.0), Rotation2d.fromDegrees(190.0)));
            waypoints.add(new Pose2d(new Translation2d(208.0, 32.0), Rotation2d.fromDegrees(135.0)).transformBy(Pose2d.fromTranslation(new Translation2d(-12.0, 0.0))));
            waypoints.add(new Pose2d(new Translation2d(208.0, 32.0), Rotation2d.fromDegrees(135.0)));

            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxAccel / 2.0)), kMaxVelocity / 2.0, kMaxAccel / 2.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightScaleToRightFence2(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(250.0, 22.0), Rotation2d.fromDegrees(195.0)));
            waypoints.add(new Pose2d(new Translation2d(202.0, 32.0 + 28.0), Rotation2d.fromDegrees(135.0)).transformBy(Pose2d.fromTranslation(new Translation2d(-12.0, 0.0))));
            waypoints.add(new Pose2d(new Translation2d(202.0, 32.0 + 28.0), Rotation2d.fromDegrees(135.0)));

            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxAccel / 2.0)), kMaxVelocity / 2.0, kMaxAccel / 2.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightStartToRightSwitch(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(startPose);
            waypoints.add(new Pose2d(new Translation2d(120.0, 36.0), Rotation2d.fromDegrees(90.0)));

            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxAccel)), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        //doesn't work
        private Trajectory<TimedState<Pose2dWithCurvature>> getRightScaleToLeftFence(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            // TODO these are wrong!
            waypoints.add(new Pose2d(new Translation2d(250.0, 200.0), Rotation2d.fromDegrees(170.0)));
            waypoints.add(new Pose2d(new Translation2d(208.0, 190.0), Rotation2d.fromDegrees(225.0)).transformBy(Pose2d.fromTranslation(new Translation2d(-12.0, 0.0))));
            waypoints.add(new Pose2d(new Translation2d(208.0, 190.0), Rotation2d.fromDegrees(225.0)));

            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxAccel)), 45, 45, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightStartToLeftScale(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(160.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(225.0, 60.0), Rotation2d.fromDegrees(90.0)));
            waypoints.add(new Pose2d(new Translation2d(225.0, 180.0), Rotation2d.fromDegrees(90.0)));
            waypoints.add(new Pose2d(new Translation2d(250.0, 205.0), Rotation2d.fromDegrees(-10.0)));

            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxAccel)), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLeftScaleToLeftFence(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(250.0, 205.0), Rotation2d.fromDegrees(170.0)));
            waypoints.add(new Pose2d(new Translation2d(208.0, 195.0), Rotation2d.fromDegrees(225.0)).transformBy(Pose2d.fromTranslation(new Translation2d(-12.0, 0.0))));
            waypoints.add(new Pose2d(new Translation2d(208.0, 195.0), Rotation2d.fromDegrees(225.0)));

            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxAccel / 2)), kMaxAccel / 2, kMaxAccel / 2, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLeftScaleToLeftFence2(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(250.0, 205.0), Rotation2d.fromDegrees(165.0)));
            waypoints.add(new Pose2d(new Translation2d(202.0, 195 - 28.0), Rotation2d.fromDegrees(225.0)).transformBy(Pose2d.fromTranslation(new Translation2d(-12.0, 0.0))));
            waypoints.add(new Pose2d(new Translation2d(202.0, 195 - 28.0), Rotation2d.fromDegrees(225.0)));

            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxAccel / 2.0)), kMaxVelocity / 2.0, kMaxAccel / 2.0, kMaxVoltage);
        }


        private Trajectory<TimedState<Pose2dWithCurvature>> getCenterStartToLeftSwitch(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(100, 60.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxAccel)), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getCenterStartToRightSwitch(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(100, -46.0), Rotation2d.fromDegrees(-40.0)));

            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxAccel)), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightFenceToRightScale(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(208.0, 32.0), Rotation2d.fromDegrees(-45.0)));
            waypoints.add(new Pose2d(new Translation2d(250.0, 22.0), Rotation2d.fromDegrees(15.0)));

            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxAccel / 2)), kMaxVelocity / 2, kMaxAccel / 2, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightFence2ToRightScale(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(202.0, 32.0 + 28.0), Rotation2d.fromDegrees(-45.0)));
            waypoints.add(new Pose2d(new Translation2d(250.0, 22.0), Rotation2d.fromDegrees(15.0)));

            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxAccel / 2)), kMaxVelocity / 2, kMaxAccel / 2, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLeftFenceToLeftScale(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(208.0, 195.0), Rotation2d.fromDegrees(45.0)));
            waypoints.add(new Pose2d(new Translation2d(250.0, 205.0), Rotation2d.fromDegrees(-15.0)));

            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxAccel / 2)), kMaxVelocity / 2, kMaxAccel / 2, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLeftFence2ToLeftScale(Pose2d startPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(208.0, 195.0 - 28.0), Rotation2d.fromDegrees(45.0)));
            waypoints.add(new Pose2d(new Translation2d(250.0, 205.0), Rotation2d.fromDegrees(-15.0)));

            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxAccel / 2)), kMaxVelocity / 2, kMaxAccel / 2, kMaxVoltage);
        }
    }
}
