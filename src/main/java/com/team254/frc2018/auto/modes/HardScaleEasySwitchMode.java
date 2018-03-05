package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.lib.geometry.Translation2d;

import java.util.Arrays;

public class HardScaleEasySwitchMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();


    public HardScaleEasySwitchMode(boolean robotStartedOnLeft) {

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SetIntaking(false, false));

        runAction(new ParallelAction(
                Arrays.asList(
                        new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rightStartToRightSwitch, true, true),
                        new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeight, SuperstructureConstants.kScoreSwitchBackwardsAngle, true)
                )
        ));

        runAction(new ShootCube());
    }
}
