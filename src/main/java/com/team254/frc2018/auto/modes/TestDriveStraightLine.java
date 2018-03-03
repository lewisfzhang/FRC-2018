package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.subsystems.Superstructure;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.timing.VelocityLimitRegionConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TestDriveStraightLine extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
        waypoints.add(new Pose2d(new Translation2d(155.0, 0.0), Rotation2d.fromDegrees(0.0)));
        waypoints.add(new Pose2d(new Translation2d(205.0, 50.0), Rotation2d.fromDegrees(90.0)));
        waypoints.add(new Pose2d(new Translation2d(205.0, 137.0), Rotation2d.fromDegrees(90.0)));
        waypoints.add(new Pose2d(new Translation2d(255.0, 187.0), Rotation2d.fromDegrees(0.0)));


        final VelocityLimitRegionConstraint<Pose2dWithCurvature> slow_region = new VelocityLimitRegionConstraint<>(
                new Translation2d(150.0, 70.0), new Translation2d(300.0, 300.0), 60.0);

        //runAction(new SetIntaking(false, false));
        runAction(new DriveTrajectory(waypoints, Arrays.asList(slow_region), 120.0, 144.0, 9.0));
        //runAction(new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeight, SuperstructureConstants.kPlacingLowAngle, true));
        //runAction(new PlaceCube());

    }
}
