package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class SimpleSwitchMode extends AutoModeBase {

    DriveTrajectory mDriveCommand;

    public SimpleSwitchMode(boolean driveToLeftSwitch) {
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
        waypoints.add(new Pose2d(new Translation2d(10, 0.0), Rotation2d.fromDegrees(0.0)));
        waypoints.add(new Pose2d(new Translation2d(100, driveToLeftSwitch ? 50.0 : -50.0), Rotation2d.fromDegrees(90.0)));
        mDriveCommand = new DriveTrajectory(waypoints, null, 70, 120, 9.0);

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SetIntaking(false, false));
        runAction(mDriveCommand);
        runAction(new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeight, SuperstructureConstants.kPlacingLowAngle, true));
        runAction(new WaitUntilCrossXBoundaryCommand(95));
        runAction(new PlaceCube());
    }
}
