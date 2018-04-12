package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoConstants;
import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import java.util.Arrays;

public class SwitchAndScaleMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    final boolean mSwitchLeft, mScaleLeft;
    private DriveTrajectory mStartToSwitch;
    private DriveTrajectory mSwitchToPyramidCube;
    private DriveTrajectory mPyramidCubeToScale;

    private double mStartCubeWaitTime;

    public SwitchAndScaleMode(boolean isSwitchOnLeft, boolean isScaleOnLeft) {
        mSwitchLeft = isSwitchOnLeft;
        mScaleLeft = isScaleOnLeft;

        if(isSwitchOnLeft) {
            mStartToSwitch = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().centerStartToLeftSwitch, true);
            mStartCubeWaitTime = mTrajectoryGenerator.getTrajectorySet().centerStartToLeftSwitch.getLastState().t() - 0.1;
        } else {
            mStartToSwitch = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().centerStartToRightSwitch, true);
            mStartCubeWaitTime = mTrajectoryGenerator.getTrajectorySet().centerStartToRightSwitch.getLastState().t() - 0.1;
        }
        mSwitchToPyramidCube = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().switchToCenterPyramidCube.get(mSwitchLeft));
        mPyramidCubeToScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().centerPyramidCubeToScale.get(mScaleLeft));
    }


    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running scale+switch");

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
                        new OpenCloseJawAction(true)
                )
        ));
        new OpenCloseJawAction(false);
        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));

        //Score second cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mPyramidCubeToScale,
                        new SeriesAction(
                                Arrays.asList(
                                        new SetIntaking(false, true),
                                        new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeight, SuperstructureConstants.kVerticalAngle, false),
                                        new WaitUntilInsideRegion(new Translation2d(140.0, -1000.0), new Translation2d
                                                (260.0, 1000.0), mScaleLeft),

                                        (AutoConstants.kUseAutoScaleHeight ?
                                                new AutoSuperstructurePosition(0, SuperstructureConstants.kScoreBackwardsAngle,
                                                        true) :
                                                new SetSuperstructurePosition(SuperstructureConstants.kScaleNeutralHeightBackwards,
                                                        45.0, true)
                                        ),

                                        new WaitUntilInsideRegion(new Translation2d(245.0, -1000.0), new Translation2d
                                                (260, 1000), mScaleLeft),
                                        new ShootCube(AutoConstants.kFullShootPower)
                                )
                        )
                )
        ));

        //Todo: grab third cube

    }

}
