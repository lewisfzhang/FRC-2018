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
    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();

    private final DriveMotionPlanner mMotionPlanner;
    private Trajectory<TimedState<Pose2dWithCurvature>> mTestLine;
    private Trajectory<TimedState<Pose2dWithCurvature>> mTestCurve;


    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> getTestLine() {
        return mTestLine;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> getTestCurve() {
        return mTestCurve;
    }

    public void generateTrajectories() {
        System.out.println("Generating trajectories...");
        generateTestLine();
        generateTestCurve();
        System.out.println("Finished trajectory generation");

    }

    private void generateTestLine() {
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
        waypoints.add(new Pose2d(new Translation2d(120.0, 0.0), Rotation2d.fromDegrees(0.0)));

        mTestLine = generateTrajectory(waypoints, null, 120.0, 144.0, 9.0);
    }

    private void generateTestCurve() {
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
        waypoints.add(new Pose2d(new Translation2d(155.0, 0.0), Rotation2d.fromDegrees(0.0)));
        waypoints.add(new Pose2d(new Translation2d(205.0, 50.0), Rotation2d.fromDegrees(90.0)));
        waypoints.add(new Pose2d(new Translation2d(205.0, 137.0), Rotation2d.fromDegrees(90.0)));
        waypoints.add(new Pose2d(new Translation2d(255.0, 187.0), Rotation2d.fromDegrees(0.0)));

        final VelocityLimitRegionConstraint<Pose2dWithCurvature> slow_region = new VelocityLimitRegionConstraint<>(
                new Translation2d(150.0, 70.0), new Translation2d(300.0, 300.0), 60.0);

        mTestCurve = generateTrajectory(waypoints, Arrays.asList(slow_region), 120.0, 144.0, 9.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(waypoints, constraints, max_vel, max_accel, max_voltage);
    }
}
