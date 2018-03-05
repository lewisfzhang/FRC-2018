package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.statemachines.IntakeStateMachine;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.subsystems.Drive;
import com.team254.frc2018.subsystems.Intake;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import java.util.Arrays;

public class EasyScaleEasySwitchMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final boolean startedLeft;

    public EasyScaleEasySwitchMode(boolean robotStartedOnLeft) {
        startedLeft = robotStartedOnLeft;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Simple switch");

        runAction(new SetIntaking(false, false));

        runAction(new ParallelAction(
                Arrays.asList(
                        new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rightStartToRightScale, true, true),
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitUntilInsideRegion(new Translation2d(130.0, -20.0), new Translation2d(260, 50)),
                                        new SetSuperstructurePosition(SuperstructureConstants.kScaleHighHeight, SuperstructureConstants.kScoreBackwardsAngle, true),
                                        new WaitAction(0.5)
                                )
                        )
                )
        ));

        runAction(new ShootCube());

        runAction(new ParallelAction(
                Arrays.asList(
                        new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rightScaleToFence, false, true),
                        new SeriesAction(
                                Arrays.asList(
                                        new SetIntaking(true, true),
                                        new OverrideTrajectory(),
                                        new WaitAction(0.1)
                                        )
                        )
                )
        ));


        runAction(new ParallelAction(
                Arrays.asList(
                        new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeight, SuperstructureConstants.kPlacingHighAngle, true),
                        new OpenLoopDrive(0.3, 0.3, 0.5, false)
                )
        ));
        new WaitAction(0.25);
        runAction(new ShootCube());

    }
}
