package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.auto.AutoConstants;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.lib.geometry.Translation2d;

import java.util.Arrays;

public class NearScaleNearSwitchMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final boolean mStartedLeft;
    private DriveTrajectory mSideStartToNearScale;
    private DriveTrajectory mNearScaleToNearFence;

    public NearScaleNearSwitchMode(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;
        mSideStartToNearScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartToNearScale.get(mStartedLeft), true);
        mNearScaleToNearFence = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence.get(mStartedLeft));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running scale + switch");

        runAction(new SetIntaking(false, false));

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

        runAction(new ParallelAction(
                Arrays.asList(
                        mNearScaleToNearFence,
                        new SetIntaking(true, false)
                )
        ));

        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));
        runAction(new OpenLoopDrive(-0.3, -0.3, 0.25, false));
        runAction(new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeight, SuperstructureConstants
                .kPlacingHighAngle, true));
        if(mStartedLeft) {
            runAction(new OpenLoopDrive(0.55, 0.45, 0.5, false));
        } else {
            runAction(new OpenLoopDrive(0.45, 0.55, 0.5, false));
        }
        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));
        runAction(new ShootCube(AutoConstants.kWeakShootPower));
    }
}
