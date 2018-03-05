package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.statemachines.IntakeStateMachine;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.subsystems.Intake;
import com.team254.lib.geometry.Translation2d;

import java.util.Arrays;

public class HardScaleHardSwitchMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    public HardScaleHardSwitchMode(boolean robotStartedOnLeft) {

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SetIntaking(false, false));

        runAction(new ParallelAction(
                Arrays.asList(
                        new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rightStartToLeftScale, true, true),
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitUntilInsideRegion(new Translation2d(160, 170), new Translation2d(260, 250)),
                                        new SetSuperstructurePosition(SuperstructureConstants.kScaleHighHeight, SuperstructureConstants.kScoreBackwardsAngle, true)
                                )
                        )
                )
        ));
        runAction(new ShootCube());

        runAction(new ParallelAction(
                Arrays.asList(
                        new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().leftScaleToLeftFence, false, true),
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
