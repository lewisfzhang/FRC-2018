package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SimpleSwitchMode extends AutoModeBase {

    final boolean mGoLeft;

    public SimpleSwitchMode(boolean driveToLeftSwitch) {
        mGoLeft = driveToLeftSwitch;
    }


    @Override
    protected void routine() throws AutoModeEndedException {
        Trajectory<TimedState<Pose2dWithCurvature>> trajectory;
        if(mGoLeft) {
            trajectory = TrajectoryGenerator.getInstance().getTrajectorySet().centerStartToLeftSwitch;
        } else {
            trajectory = TrajectoryGenerator.getInstance().getTrajectorySet().centerStartToRightSwitch;
        }

        System.out.println("Running Simple switch");
        runAction(new SetIntaking(false, false));


        runAction(new ParallelAction(
                Arrays.asList(
                        (new DriveTrajectory(trajectory, true)),
                        (new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeightBackwards, SuperstructureConstants.kStowedPositionAngle, true))
                )
        ));


        runAction(new ShootCube());
    }
}
