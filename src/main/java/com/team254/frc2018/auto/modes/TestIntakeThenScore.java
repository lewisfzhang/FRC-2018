package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.subsystems.*;
import com.team254.lib.physics.DriveCharacterization;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TestIntakeThenScore extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SetIntaking(false, false));
        runAction(new ParallelAction(
                Arrays.asList(
                        new OpenLoopDrive(-0.5, -0.5, 2.0, false),
                        new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeight, SuperstructureConstants.kStowedPositionAngle, true)
                )
        ));

        runAction(new ShootCube());


        runAction(new ParallelAction(
                Arrays.asList(
                        new OpenLoopDrive(0.5, 0.5, 2.5, true),
                        new SetIntaking(true, true)
                )
        ));

        runAction(new ParallelAction(
                Arrays.asList(
                        new OpenLoopDrive(-0.5, -0.5, 2.0, false),
                        new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeight, SuperstructureConstants.kStowedPositionAngle, true)
                )
        ));

        runAction(new ShootCube());
    }
}
