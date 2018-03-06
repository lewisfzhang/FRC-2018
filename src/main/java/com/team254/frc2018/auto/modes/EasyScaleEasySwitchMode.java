package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.lib.geometry.Translation2d;

import java.util.Arrays;

public class EasyScaleEasySwitchMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final boolean startedLeft;

    public EasyScaleEasySwitchMode(boolean robotStartedOnLeft) {
        startedLeft = robotStartedOnLeft;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running scale + switch");

        runAction(new SetIntaking(false, false));

        runAction(new ParallelAction(
                Arrays.asList(
                        new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rightStartToRightScale, true, true),
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitUntilInsideRegion(new Translation2d(130.0, -20.0), new Translation2d
                                                (260, 50)),
                                        new SetSuperstructurePosition(SuperstructureConstants.kScaleLowHeight,
                                                SuperstructureConstants.kScoreBackwardsAngle, true),
                                        new WaitUntilInsideRegion(new Translation2d(245.0, -1000.0), new Translation2d
                                                (260, 1000)),
                                        new ShootCube(0.66)
                                )
                        )
                )
        ));

        runAction(new ParallelAction(
                Arrays.asList(
                        new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rightScaleToRightFence, false, false),
                        new SetIntaking(true, false)
                )
        ));

        runAction(new WaitAction(0.5));
        runAction(new OpenLoopDrive(-0.3, -0.3, 0.25, false));
        runAction(new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeight, SuperstructureConstants
                .kPlacingHighAngle, true));
        runAction(new OpenLoopDrive(0.5, 0.5, 0.5, false));
        runAction(new WaitAction(0.25));
        runAction(new ShootCube(0.5));
    }
}
