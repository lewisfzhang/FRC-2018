package com.team254.frc2018.planners;

import com.team254.frc2018.Constants;
import com.team254.frc2018.Kinematics;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.CentripetalAccelerationConstraint;
import org.junit.jupiter.api.Test;

import java.util.Arrays;

public class DriveMotionPlannerTest {

    @Test
    public void testForwardSwerveRight() {
        DriveMotionPlanner motion_planner = new DriveMotionPlanner();
        motion_planner.setFollowerType(DriveMotionPlanner.FollowerType.FEEDFORWARD_ONLY);
        motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(motion_planner.generateTrajectory
                (Arrays.asList(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(120.0, -36.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(240.0, -36.0), Rotation2d.identity())),
                        Arrays.asList(new CentripetalAccelerationConstraint(120.0)),
                        120.0, 120.0, 10.0))), false);

        double t = 0.0;
        Pose2d pose = motion_planner.setpoint().state().getPose();
        while (!motion_planner.isDone()) {
            motion_planner.update(t, pose);
            pose = motion_planner.mSetpoint.state().getPose();//.transformBy(new Pose2d(new Translation2d(0.0, 1.0),
            // Rotation2d.fromDegrees(2.0)));

            System.out.println(t + "," + motion_planner.toCSV());
            t += 0.01;
        }
    }

    @Test
    public void testForwardSwerveLeft() {
        DriveMotionPlanner motion_planner = new DriveMotionPlanner();
        motion_planner.setFollowerType(DriveMotionPlanner.FollowerType.FEEDFORWARD_ONLY);
        motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(motion_planner.generateTrajectory
                (Arrays.asList(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(120.0, 36.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(240.0, 36.0), Rotation2d.identity())),
                        null,
                        120.0, 120.0, 10.0))), false);
        double t = 0.0;
        Pose2d pose = motion_planner.setpoint().state().getPose();
        while (!motion_planner.isDone()) {
            motion_planner.update(t, pose);
            pose = motion_planner.mSetpoint.state().getPose();//.transformBy(new Pose2d(new Translation2d(0.0, -1.0),
            // Rotation2d.fromDegrees(-2.0)));
            System.out.println(t + "," + motion_planner.toCSV());
            t += 0.01;
        }
    }

    @Test
    public void testReverseSwerveLeft() {
        DriveMotionPlanner motion_planner = new DriveMotionPlanner();
        motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(motion_planner.generateTrajectory
                (Arrays.asList(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(120.0, 36.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(240.0, 36.0), Rotation2d.identity())),
                        null,
                        120.0, 120.0, 10.0))), true);
        double t = 0.0;
        Pose2d pose = motion_planner.setpoint().state().getPose().transformBy(new Pose2d(Translation2d.identity(),
                Rotation2d.fromDegrees(180.0)));
        while (!motion_planner.isDone()) {
            motion_planner.update(t, pose);
            pose = motion_planner.mSetpoint.state().getPose().transformBy(new Pose2d(new Translation2d(0.0, -1.0),
                    Rotation2d.fromDegrees(178.0)));
            System.out.println(t + "," + motion_planner.toCSV());
            t += 0.01;
        }
    }

    @Test
    public void testFollowerReachesGoal() {
        final DriveMotionPlanner motion_planner = new DriveMotionPlanner();
        motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(motion_planner.generateTrajectory
                (Arrays.asList(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(120.0, -36.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(240.0, -36.0), Rotation2d.identity())),
                        null,
                        120.0, 120.0, 10.0))), false);
        final double dt = 0.01;
        double t = 0.0;
        Pose2d pose = motion_planner.setpoint().state().getPose();
        while (!motion_planner.isDone()) {
            DriveMotionPlanner.Output output = motion_planner.update(t, pose);
            Twist2d delta = Kinematics.forwardKinematics(output.left_velocity * dt * Constants
                    .kDriveWheelDiameterInches / 2.0, output.right_velocity * dt * Constants
                    .kDriveWheelDiameterInches / 2.0);
            pose = pose.transformBy(Pose2d.exp(delta));
            t += dt;
            System.out.println(motion_planner.setpoint().toCSV() + "," + pose.toCSV());
        }
        System.out.println(pose);
    }
}
