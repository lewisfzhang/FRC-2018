package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.auto.AutoConstants;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.lib.geometry.Translation2d;

import java.util.Arrays;

// TODO fix
public class FarScaleNearSwitchMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private boolean mStartedLeft;
    private DriveTrajectory mSideStartToFarScale;
    private DriveTrajectory mFarScaleToFarFence;
    private DriveTrajectory mFarFenceToNearSwitch;

    public FarScaleNearSwitchMode(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;
        mSideStartToFarScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartToFarScale.get(mStartedLeft), true);
        mFarScaleToFarFence = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().farScaleToFarFence.get(mStartedLeft));
        mFarFenceToNearSwitch = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().farFenceToNearSwitch.get(mStartedLeft));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SetIntaking(false, false));

        // Score first cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mSideStartToFarScale,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitUntilInsideRegion(new Translation2d(130.0, 170.0), new Translation2d
                                                (260, 200.0), mStartedLeft),
                                        new SetSuperstructurePosition(SuperstructureConstants.kScaleLowHeight,
                                                SuperstructureConstants.kScoreBackwardsAngle, true),
                                        new WaitUntilInsideRegion(new Translation2d(245.0, 170.0), new Translation2d
                                                (260, 1000), mStartedLeft),
                                        new ShootCube(AutoConstants.kMediumShootPower)
                                )
                        )
                )
        ));

        runAction(new ParallelAction(
                Arrays.asList(
                        new SeriesAction(
                                Arrays.asList(
                                        mFarScaleToFarFence,
                                        new WaitAction(AutoConstants.kWaitForCubeTime)
                                )
                        ),
                        new SeriesAction(
                                Arrays.asList(
                                        new SetIntaking(true, false)
                                )
                        )
                )
        ));

        runAction(new ParallelAction(
                Arrays.asList(
                        mFarFenceToNearSwitch,
                        new SeriesAction(
                                Arrays.asList(
                                        new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeight,
                                                SuperstructureConstants.kScoreSwitchBackwardsAngle, true)
                                )
                        )
                )
        ));
        runAction(new ShootCube(AutoConstants.kMediumShootPower));
    }
}
