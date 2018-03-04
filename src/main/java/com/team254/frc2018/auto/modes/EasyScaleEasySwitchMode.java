package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.subsystems.Drive;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import java.util.Arrays;

public class EasyScaleEasySwitchMode extends AutoModeBase {

    private final boolean startedLeft;

    public EasyScaleEasySwitchMode(boolean robotStartedOnLeft) {
        startedLeft = robotStartedOnLeft;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Simple switch");

        runAction(new SetIntaking(false, false));

        Trajectory<TimedState<Pose2dWithCurvature>> trajectory;
        if(startedLeft) {
            //dont have a path for this yet
            trajectory = TrajectoryGenerator.getInstance().getTrajectorySet().rightStartToRightScale;
        } else {
            trajectory = TrajectoryGenerator.getInstance().getTrajectorySet().rightStartToRightScale;
        }

        runAction(new ParallelAction(
                Arrays.asList(
                        new DriveTrajectory(trajectory, true, true),
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitUntilInsideRegion(new Translation2d(150, -100), new Translation2d(280, 100)),
                                        new SetSuperstructurePosition(SuperstructureConstants.kScaleNeutralHeightBackwards, SuperstructureConstants.kScoreBackwardsAngle, true)
                                )
                        )
                )
        ));

        runAction(new ShootCube());

        Trajectory<TimedState<Pose2dWithCurvature>> intakeTrajectory;
        if(startedLeft) {
            //dont have a path for this yet
            intakeTrajectory = TrajectoryGenerator.getInstance().getTrajectorySet().rightScaleToFence;
        } else {
            intakeTrajectory = TrajectoryGenerator.getInstance().getTrajectorySet().rightScaleToFence;
        }

        runAction(new ParallelAction(
        Arrays.asList(
                new DriveTrajectory(intakeTrajectory, false),
                new SeriesAction(
                        Arrays.asList(
                                new SetIntaking(true, true),
                                new OverrideTrajectory()
                        )
                )
        )));

        runAction(new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeight, SuperstructureConstants.kPlacingHighAngle, true));
        runAction(new PlaceCube());


    }
}
