package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.subsystems.Superstructure;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TestDriveStraightLine extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
        waypoints.add(new Pose2d(new Translation2d(120.0, -36.0), Rotation2d.fromDegrees(0.0)));
        waypoints.add(new Pose2d(new Translation2d(240.0, -36.0), Rotation2d.fromDegrees(0.0)));
        //runAction(new SetIntaking(false, false));
        runAction(new DriveTrajectory(waypoints, 120.0, 120.0, 10.0));
        //runAction(new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeight, SuperstructureConstants.kPlacingLowAngle, true));
        //runAction(new PlaceCube());

    }
}
