package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import java.util.Arrays;

public class RightStartSwitchMode extends AutoModeBase {

    final boolean mGoLeft;

    public RightStartSwitchMode(boolean driveToLeftSwitch) {
        mGoLeft = driveToLeftSwitch;
    }


    @Override
    protected void routine() throws AutoModeEndedException {
        Trajectory<TimedState<Pose2dWithCurvature>> trajectory;
        if(mGoLeft) {
            trajectory = TrajectoryGenerator.getInstance().getTrajectorySet().rightStartToLeftSwitch;
        } else {
            trajectory = TrajectoryGenerator.getInstance().getTrajectorySet().rightStartToRightSwitch;
        }

        System.out.println("Running Simple switch");
        runAction(new SetIntaking(false, false));


        runAction(new ParallelAction(
                Arrays.asList(
                        (new DriveTrajectory(trajectory, true, true)),
                        (new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeight, SuperstructureConstants.kStowedPositionAngle, true))
                )
        ));


        runAction(new ShootCube());
    }
}
