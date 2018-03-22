package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.auto.AutoConstants;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.lib.geometry.Translation2d;

import java.util.Arrays;

public class NearScaleOnlyMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final boolean mStartedLeft;
    private DriveTrajectory mSideStartToNearScale;
    private DriveTrajectory mNearScaleToNearFence;
    private DriveTrajectory mNearFenceToNearScale;
    private DriveTrajectory mNearScaleToNearFence2;
    private DriveTrajectory mNearFence2ToNearScale;
    private DriveTrajectory mNearScaleToNearFence3;
    private DriveTrajectory mNearFence3ToNearScale;

    public NearScaleOnlyMode(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;
        mSideStartToNearScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartToNearScale.get(mStartedLeft), true);
        mNearScaleToNearFence = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence.get(mStartedLeft));
        mNearFenceToNearScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearFenceToNearScale.get(mStartedLeft));
        mNearScaleToNearFence2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence2.get(mStartedLeft));
        mNearFence2ToNearScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearFence2ToNearScale.get(mStartedLeft));
        mNearScaleToNearFence3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence3.get(mStartedLeft));
        mNearFence3ToNearScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearFence3ToNearScale.get(mStartedLeft));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running easy scale only");

        runAction(new SetIntaking(false, false));

        // Score first cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mSideStartToNearScale,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitUntilInsideRegion(new Translation2d(130.0, -20.0), new Translation2d
                                                (260, 50), mStartedLeft),
                                        new SetSuperstructurePosition(SuperstructureConstants.kScaleLowHeight,
                                                SuperstructureConstants.kScoreBackwardsAngle, true),
                                        new WaitUntilInsideRegion(new Translation2d(245.0, -1000.0), new Translation2d
                                                (260, 1000), mStartedLeft),
                                        new ShootCube(AutoConstants.kStrongShootPower)
                                )
                        )
                )
        ));

        // Get second cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mNearScaleToNearFence,
                        new SetIntaking(true, false)
                )
        ));
        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));

        // Score second cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mNearFenceToNearScale,
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
                        mNearScaleToNearFence2,
                        new SetIntaking(true, false)
                )
        ));
        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));

        // Score third cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mNearFence2ToNearScale,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(AutoConstants.kWaitForCubeTime), //drive backwards for a little with the intake down so it gets the chance to pick up cubes jammed against the wall
                                        new SetSuperstructurePosition(SuperstructureConstants.kScaleLowHeight - 8.0,
                                                SuperstructureConstants.kScoreBackwardsAngle, true),
                                        new WaitUntilInsideRegion(new Translation2d(245.0, -1000.0), new Translation2d
                                                (260, 1000), mStartedLeft),
                                        new ShootCube(AutoConstants.kStrongShootPower)
                                )
                        )
                )
        ));

        // Get fourth cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mNearScaleToNearFence3,
                        new SetIntaking(true, false)
                )
        ));
        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));

        // Score fourth cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mNearFence3ToNearScale,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(AutoConstants.kWaitForCubeTime), //drive backwards for a little with the intake down so it gets the chance to pick up cubes jammed against the wall
                                        new SetSuperstructurePosition(SuperstructureConstants.kScaleLowHeight - 8.0,
                                                SuperstructureConstants.kScoreBackwardsAngle, true),
                                        new WaitUntilInsideRegion(new Translation2d(245.0, -1000.0), new Translation2d
                                                (260, 1000), mStartedLeft),
                                        new ShootCube(AutoConstants.kStrongShootPower)
                                )
                        )
                )
        ));
    }
}
