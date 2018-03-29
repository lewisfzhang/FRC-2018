package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.auto.AutoConstants;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.states.SuperstructureConstants;

import java.util.Arrays;

public class SimpleSwitchMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    final boolean mStartedLeft;
    private DriveTrajectory mStartToSwitch;

    private DriveTrajectory mSwitchToPyramidCube;
    private DriveTrajectory mSwitchToPyramidCube1;
    private DriveTrajectory mSwitchToPyramidCube2;

    private DriveTrajectory mPyramidCubeToSwitch;
    private DriveTrajectory mPyramidCube1ToSwitch;
    private DriveTrajectory mPyramidCube2ToCenterField;

    private double mPyramidCubeWaitTime, mPyramidCube1WaitTime, mStartCubeWaitTime;

    public SimpleSwitchMode(boolean driveToLeftSwitch) {
        mStartedLeft = driveToLeftSwitch;

        if(mStartedLeft) {
            mStartToSwitch = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().centerStartToLeftSwitch, true);
        } else {
            mStartToSwitch = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().centerStartToRightSwitch, true);
        }

        mSwitchToPyramidCube = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().switchToPyramidCube.get(mStartedLeft));
        mSwitchToPyramidCube1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().switchToPyramidCube1.get(mStartedLeft));
        mSwitchToPyramidCube2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().switchToPyramidCube2.get(mStartedLeft));

        mPyramidCubeToSwitch = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().pyramidCubeToSwitch.get(mStartedLeft));
        mPyramidCube1ToSwitch = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().pyramidCube1ToSwitch.get(mStartedLeft));
        mPyramidCube2ToCenterField = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().pyramidCube2ToCenterField.get(mStartedLeft));

        if(mStartedLeft) {
            mStartCubeWaitTime = mTrajectoryGenerator.getTrajectorySet().centerStartToLeftSwitch.getLastState().t() - 0.2;
        } else {
            mStartCubeWaitTime = mTrajectoryGenerator.getTrajectorySet().centerStartToRightSwitch.getLastState().t() - 0.2;
        }
        mPyramidCubeWaitTime = mTrajectoryGenerator.getTrajectorySet().pyramidCubeToSwitch.get(mStartedLeft).getLastState().t() - 0.2;
        mPyramidCube1WaitTime = mTrajectoryGenerator.getTrajectorySet().pyramidCube1ToSwitch.get(mStartedLeft).getLastState().t() - 0.2;
    }


    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Simple switch");

        //Score first cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mStartToSwitch,
                        new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeightBackwards, SuperstructureConstants.kStowedPositionAngle, true),
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(mStartCubeWaitTime),
                                        new ShootCube(AutoConstants.kMediumShootPower)
                                )
                        )
                )
        ));

        // Get second cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mSwitchToPyramidCube,
                        new SetIntaking(true, false),
                        new OpenCloseJawAction(false)
                )
        ));
        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));

        //Score second cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mPyramidCubeToSwitch,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(0.5),
                                        new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeightBackwards, SuperstructureConstants.kStowedPositionAngle, true)
                                )
                        ),
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(mPyramidCubeWaitTime),
                                        new ShootCube(AutoConstants.kMediumShootPower)
                                )
                        )
                )
        ));

        // Get third cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mSwitchToPyramidCube1,
                        new SetSuperstructurePosition(SuperstructureConstants.kIntakeSecondLevelHeight, SuperstructureConstants.kIntakePositionAngle, true),
                        new SetIntaking(false, false),
                        new OpenCloseJawAction(false)
                )
        ));
        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));

        //Score third cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mPyramidCube1ToSwitch,
                        new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeightBackwards, SuperstructureConstants.kStowedPositionAngle, true),
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(mPyramidCube1WaitTime),
                                        new ShootCube(AutoConstants.kMediumShootPower)
                                )
                        )
                )
        ));

        // Get fourth cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mSwitchToPyramidCube2,
                        new SetIntaking(true, false),
                        new OpenCloseJawAction(false)
                )
        ));
        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));


        //Drive to center field
        runAction(new ParallelAction(
                Arrays.asList(
                        mPyramidCube2ToCenterField,
                        (new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeightBackwards, SuperstructureConstants.kStowedPositionAngle, true))
                )
        ));
    }
}
