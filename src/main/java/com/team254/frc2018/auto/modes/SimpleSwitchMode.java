package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.auto.AutoConstants;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.states.SuperstructureConstants;

import java.util.Arrays;

public class SimpleSwitchMode extends AutoModeBase {

    final boolean mGoLeft;
    private DriveTrajectory mTrajectory;

    public SimpleSwitchMode(boolean driveToLeftSwitch) {
        mGoLeft = driveToLeftSwitch;

        if(mGoLeft) {
            mTrajectory = new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().centerStartToLeftSwitch, true);
        } else {
            mTrajectory = new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().centerStartToRightSwitch, true);
        }
    }


    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Simple switch");
        runAction(new SetIntaking(false, false));

        runAction(new ParallelAction(
                Arrays.asList(
                        mTrajectory,
                        (new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeightBackwards, SuperstructureConstants.kStowedPositionAngle, true))
                )
        ));

        runAction(new ShootCube(AutoConstants.kMediumShootPower));
    }
}
