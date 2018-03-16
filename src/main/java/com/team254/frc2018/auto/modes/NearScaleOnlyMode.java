package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.lib.geometry.Translation2d;

import java.util.Arrays;

public class NearScaleOnlyMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final boolean mStartedLeft;

    public NearScaleOnlyMode(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running easy scale only");

        runAction(new SetIntaking(false, false));

        // Score first cube
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

        // Get second cube
        runAction(new ParallelAction(
                Arrays.asList(
                        new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence.get(mStartedLeft)),
                        new SetIntaking(true, false)
                )
        ));
        runAction(new WaitAction(0.1));

        // Score second cube
        runAction(new ParallelAction(
                Arrays.asList(
                        new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearFenceToNearScale.get(mStartedLeft)),
                        new SeriesAction(
                                Arrays.asList(
                                        new SetSuperstructurePosition(SuperstructureConstants.kScaleLowHeight - 8.0,
                                                SuperstructureConstants.kScoreBackwardsAngle, true),
                                        new WaitUntilInsideRegion(new Translation2d(245.0, -1000.0), new Translation2d
                                                (260, 1000), mStartedLeft),
                                        new ShootCube(0.66)
                                )
                        )
                )
        ));

        // Get third cube
        runAction(new ParallelAction(
                Arrays.asList(
                        new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence2.get(mStartedLeft)),
                        new SetIntaking(true, false)
                )
        ));
        runAction(new WaitAction(0.1));

        // Score third cube
        runAction(new ParallelAction(
                Arrays.asList(
                        new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearFence2ToNearScale.get(mStartedLeft)),
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(0.2), //drive backwards for a little with the intake down so it gets the chance to pick up cubes jammed against the wall
                                        new SetSuperstructurePosition(SuperstructureConstants.kScaleLowHeight - 8.0,
                                                SuperstructureConstants.kScoreBackwardsAngle, true),
                                        new WaitUntilInsideRegion(new Translation2d(245.0, -1000.0), new Translation2d
                                                (260, 1000), mStartedLeft),
                                        new ShootCube(0.66)
                                )
                        )
                )
        ));

    }
}
