package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.auto.AutoConstants;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.lib.geometry.Translation2d;

import java.util.Arrays;

public class FarScaleOnlyMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final boolean mStartedLeft;

    private DriveTrajectory mSideStartToFarScale;
    private DriveTrajectory mFarScaleToFarFence;
    private DriveTrajectory mFarFenceToFarScale;
    private DriveTrajectory mFarScaleToFarFence2;
    private DriveTrajectory mFarFence2ToFarScale;
    private DriveTrajectory mFarScaleToFarFence3;

    public FarScaleOnlyMode(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;
        mSideStartToFarScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartToFarScale.get(mStartedLeft), true);
        mFarScaleToFarFence = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().farScaleToFarFence.get(mStartedLeft));
        mFarFenceToFarScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().farFenceToFarScale.get(mStartedLeft));
        mFarScaleToFarFence2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().farScaleToFarFence2.get(mStartedLeft));
        mFarFence2ToFarScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().farFence2ToFarScale.get(mStartedLeft));
        mFarScaleToFarFence3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().farScaleToFarFence3.get(mStartedLeft));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running far scale only");

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

        // Get second cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mFarScaleToFarFence,
                        new SetIntaking(true, false)
                )
        ));
        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));

        // Score second cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mFarFenceToFarScale,
                        new SeriesAction(
                                Arrays.asList(
                                        new SetSuperstructurePosition(SuperstructureConstants.kScaleLowHeight - 8.0,
                                                SuperstructureConstants.kScoreBackwardsAngle, true),
                                        new WaitUntilInsideRegion(new Translation2d(245.0, -1000.0), new Translation2d
                                                (260, 1000), mStartedLeft),
                                        new ShootCube(AutoConstants.kStrongShootPower)
                                )
                        )
                )
        ));

        // Get third cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mFarScaleToFarFence2,
                        new SetIntaking(true, false)
                )
        ));
        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));

        // Score third cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mFarFence2ToFarScale,
                        new SeriesAction(
                                Arrays.asList(
                                        new SetSuperstructurePosition(SuperstructureConstants.kScaleLowHeight - 8.0,
                                                SuperstructureConstants.kScoreBackwardsAngle, true),
                                        new WaitUntilInsideRegion(new Translation2d(245.0, -1000.0), new Translation2d
                                                (260, 1000), mStartedLeft),
                                        new ShootCube(AutoConstants.kFullShootPower)
                                )
                        )
                )
        ));

        // Get fourth cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mFarScaleToFarFence3,
                        new SetIntaking(true, false)
                )
        ));

    }
}
