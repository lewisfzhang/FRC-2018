package com.team254.frc2018.auto.actions;

import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.subsystems.Drive;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import java.util.List;

public class DriveTrajectory implements Action {
    private static final Drive mDrive = Drive.getInstance();

    private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTrajectory;

    public DriveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory) {
        mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
    }

    @Override
    public boolean isFinished() {
        return mDrive.isDoneWithTrajectory();
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {


    }

    @Override
    public void start() {
        mDrive.setTrajectory(mTrajectory);
    }
}
