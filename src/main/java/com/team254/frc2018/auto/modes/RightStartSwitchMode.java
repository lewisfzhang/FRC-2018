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
    final boolean mRobotOnLeft;

    public RightStartSwitchMode(boolean robotStartedOnLeft, boolean switchIsLeft) {
        mRobotOnLeft = robotStartedOnLeft;
        mGoLeft = switchIsLeft;
    }


    @Override
    protected void routine() throws AutoModeEndedException {
        Trajectory<TimedState<Pose2dWithCurvature>> trajectory;
        if(mGoLeft == mRobotOnLeft) {
            trajectory = TrajectoryGenerator.getInstance().getTrajectorySet().sideStartToNearSwitch.get(mRobotOnLeft);
        } else {
            trajectory = TrajectoryGenerator.getInstance().getTrajectorySet().sideStartToFarSwitch.get(mRobotOnLeft);
        }

        System.out.println("Running Simple switch");
        runAction(new SetIntaking(false, false));


        runAction(new ParallelAction(
                Arrays.asList(
                        (new DriveTrajectory(trajectory, true)),
                        (new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeight, SuperstructureConstants.kStowedPositionAngle, true))
                )
        ));


        runAction(new ShootCube());
    }
}