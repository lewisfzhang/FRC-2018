package com.team254.frc2018.planners;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import org.junit.jupiter.api.Test;

import java.util.Arrays;

public class DriveMotionPlannerTest {

    @Test
    public void test() {
        DriveMotionPlanner motion_planner = new DriveMotionPlanner();
        motion_planner.generateTrajectory(Arrays.asList(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.identity()),
                new Pose2d(new Translation2d(120.0, 0.0), Rotation2d.identity())), 120.0, 120.0, 12.0);


    }
}
