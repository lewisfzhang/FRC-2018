package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.auto.AutoConstants;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.subsystems.Drive;
import com.team254.lib.geometry.Translation2d;

import java.util.Arrays;

public class FarScaleFarSwitchMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private final boolean mStartedLeft;
    private DriveTrajectory mSideStartToFarScale;
    private DriveTrajectory mFarScaleToFarFence;


    public FarScaleFarSwitchMode(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;
        mSideStartToFarScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartToFarScale.get(mStartedLeft), true);
        mFarScaleToFarFence = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().farScaleToFarFence.get(mStartedLeft));
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
                                        new ShootCube(AutoConstants.kStrongShootPower)
                                )
                        )
                )
        ));

        runAction(new ParallelAction(
                Arrays.asList(
                        mFarScaleToFarFence,
                        new SeriesAction(
                                Arrays.asList(
                                        new SetIntaking(true, false)
                                )
                        )
                )
        ));

        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));
        runAction(new OpenLoopDrive(-0.3, -0.3, 0.25, false));
        runAction(new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeight, SuperstructureConstants
                .kPlacingHighAngle, true));

        if(mStartedLeft) {
            runAction(new OpenLoopDrive(0.45, 0.55, 0.5, false));
        } else {
            runAction(new OpenLoopDrive(0.55, 0.45, 0.5, false));
        }

        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));
        runAction(new ShootCube(AutoConstants.kWeakShootPower));

    }
}
