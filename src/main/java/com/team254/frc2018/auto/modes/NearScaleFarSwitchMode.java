package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.lib.geometry.Translation2d;

import java.util.Arrays;

public class NearScaleFarSwitchMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private boolean mStartedLeft;

    public NearScaleFarSwitchMode(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SetIntaking(false, false));

        runAction(new ParallelAction(
                Arrays.asList(
                        new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartToNearScale.get(mStartedLeft), true),
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitUntilInsideRegion(new Translation2d(130.0, -20.0), new Translation2d
                                                (260, 50), mStartedLeft),
                                        new SetSuperstructurePosition(SuperstructureConstants.kScaleLowHeight,
                                                SuperstructureConstants.kScoreBackwardsAngle, true),
                                        new WaitUntilInsideRegion(new Translation2d(245.0, -1000.0), new Translation2d
                                                (260, 1000), mStartedLeft),
                                        new ShootCube(0.66)
                                )
                        )
                )
        ));

        runAction(new ParallelAction(
                Arrays.asList(
                        new SeriesAction(
                                Arrays.asList(
                                        new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence.get(mStartedLeft)),
                                        new WaitAction(0.2) //give intake more time to pick up cube
                                )
                        ),
                        new SetIntaking(true, false)
                )
        ));

        runAction(new ParallelAction(
                Arrays.asList(
                        new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearFenceToFarSwitch.get(mStartedLeft)),
                        new SeriesAction(
                                Arrays.asList(
                                        new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeight,
                                                SuperstructureConstants.kScoreSwitchBackwardsAngle, true)
                                )
                        )
                )
        ));
        runAction(new ShootCube(0.66));

    }
}
