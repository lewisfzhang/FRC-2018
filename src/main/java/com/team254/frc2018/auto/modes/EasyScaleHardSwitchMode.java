package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.lib.geometry.Translation2d;

import java.util.Arrays;


public class EasyScaleHardSwitchMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final boolean startedLeft;

    public EasyScaleHardSwitchMode(boolean robotStartedOnLeft) {
        startedLeft = robotStartedOnLeft;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        /*
        runAction(new SetIntaking(false, false));

        runAction(new ParallelAction(
                Arrays.asList(
                        new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().startToNearScale.get(startedLeft), true, true),
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
<<<<<<< HEAD
                        new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rightScaleToRightFence, false, false),
                        new SetIntaking(true, false)
=======
                        new SeriesAction(
                                Arrays.asList(
                                        new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearScaleToFarFence.get(startedLeft), false, true)
                                )
                        ),
                        new SeriesAction(
                                Arrays.asList(
                                        new SetSuperstructurePosition(SuperstructureConstants.kStowedPositionHeight, SuperstructureConstants.kStowedPositionAngle, false),
                                        new WaitUntilInsideRegion(new Translation2d(160.0, 180.0), new Translation2d(260, 250)),
                                        new SetIntaking(true, true),
                                        new OverrideTrajectory(),
                                        new WaitAction(0.1)
                                )
                        )
>>>>>>> Add mirroring of left/right trajectories
                )
        ));

        runAction(new ParallelAction(
                Arrays.asList(
                        new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rightFenceToLeftSwitch, true, false),
                        new SeriesAction(
                                Arrays.asList(
                                        new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeight,
                                                SuperstructureConstants.kScoreSwitchBackwardsAngle, true)
                                )
                        )
                )
        ));
        runAction(new ShootCube(0.66));
*/
    }
}

