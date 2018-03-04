package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.subsystems.*;
import com.team254.lib.physics.DriveCharacterization;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TestIntakeThenScore extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ParallelAction(
                Arrays.asList(
//                        new DriveTrajectory(TrajectoryGenerator.getInstance().getStraightLine(), false),
                        new SeriesAction(
                                Arrays.asList(
                                        new SetIntaking(true, true),
                                        new OverrideTrajectory()
                                )
                        )
                )
        ));

    }
}
