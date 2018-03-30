package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.auto.AutoConstants;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.subsystems.Drive;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.DriveSignal;

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

    private double mFarFenceWaitTime, mFarFence2WaitTime, mFarFence3WaitTime;

    public FarScaleOnlyMode(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;
        mSideStartToFarScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartToFarScale.get(mStartedLeft), true);
        mFarScaleToFarFence = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().farScaleToFarFence.get(mStartedLeft));
        mFarFenceToFarScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().farFenceToFarScale.get(mStartedLeft));
        mFarScaleToFarFence2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().farScaleToFarFence2.get(mStartedLeft));
        mFarFence2ToFarScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().farFence2ToFarScale.get(mStartedLeft));
        mFarScaleToFarFence3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().farScaleToFarFence3.get(mStartedLeft));

        mFarFenceWaitTime = mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence.get(mStartedLeft).getLastState().t() - 0.1;
        mFarFence2WaitTime = mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence2.get(mStartedLeft).getLastState().t() - 0.1;
        mFarFence3WaitTime = mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence3.get(mStartedLeft).getLastState().t() - 0.1;
    }

    @Override
    public void done() {
        Drive.getInstance().setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        // System.out.println("Running far scale only");

        // Score first cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mSideStartToFarScale,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(1.0),
                                        new SetIntaking(false, false)
                                )
                        ),
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitUntilInsideRegion(new Translation2d(130.0, 150.0), new Translation2d
                                                (260, 200.0), mStartedLeft),
                                        new SetSuperstructurePosition(SuperstructureConstants.kScaleLowHeight,
                                                SuperstructureConstants.kScoreBackwardsAngle, true),
                                        new WaitUntilInsideRegion(new Translation2d(245.0, 150.0), new Translation2d
                                                (260, 1000), mStartedLeft),
                                        new ShootCube(AutoConstants.kStrongShootPower)
                                )
                        )
                )
        ));

        // Get second cube
        runAction(new ParallelAction(
                Arrays.asList(
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(0.25),
                                        mFarScaleToFarFence
                                )
                        ),
                        new OpenCloseJawAction(true),
                        new SetIntaking(true, false),
                        new SeriesAction(Arrays.asList(
                                new WaitAction(mFarFenceWaitTime),
                                new OpenCloseJawAction(false)
                        ))
                )
        ));
        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));

        // Score second cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mFarFenceToFarScale,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(AutoConstants.kWaitForCubeTime),
                                        new SetIntaking(false, true),
                                        new SetSuperstructurePosition(SuperstructureConstants.kScaleLowHeight - 8.0,
                                                SuperstructureConstants.kScoreBackwardsAngle, true),
                                        new WaitUntilInsideRegion(new Translation2d(245.0, -1000.0), new Translation2d
                                                (260, 1000), mStartedLeft),
                                        new ShootCube(AutoConstants.kFullShootPower)
                                )
                        )
                )
        ));

        // Get third cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mFarScaleToFarFence2,
                        new OpenCloseJawAction(true),
                        new SetIntaking(true, false),
                        new SeriesAction(Arrays.asList(
                                new WaitAction(mFarFence2WaitTime),
                                new OpenCloseJawAction(false)
                        ))
                )
        ));
        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));

        // Score third cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mFarFence2ToFarScale,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(AutoConstants.kWaitForCubeTime),
                                        new SetIntaking(false, true),
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
//        runAction(new ParallelAction(
//                Arrays.asList(
//                        mFarScaleToFarFence3,
//                        new OpenCloseJawAction(true),
//                        new SetIntaking(true, false),
//                        new SeriesAction(Arrays.asList(
//                                new WaitAction(mFarFence3WaitTime),
//                                new OpenCloseJawAction(false)
//                        ))
//                )
//        ));
    }
}
