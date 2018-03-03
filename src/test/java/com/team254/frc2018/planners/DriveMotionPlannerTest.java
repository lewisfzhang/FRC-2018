package com.team254.frc2018.planners;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.TrajectoryIterator;
import org.junit.jupiter.api.Test;

import java.util.Arrays;

public class DriveMotionPlannerTest {

    @Test
    public void test() {
        DriveMotionPlanner motion_planner = new DriveMotionPlanner();
        motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(motion_planner.generateTrajectory
                (Arrays.asList(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.identity()),
                new Pose2d(new Translation2d(120.0, 36.0), Rotation2d.identity())), 120.0, 120.0, 10.0))));

        double t = 0.0;
        while (!motion_planner.isDone()) {
            DriveMotionPlanner.Output output = motion_planner.update(t);
            //System.out.println("t: " + t + ", leftv: " + output.left_feedforward_voltage + ", rightv: " + output
            //        .right_feedforward_voltage);
            t += 0.01;
        }

    }
}
