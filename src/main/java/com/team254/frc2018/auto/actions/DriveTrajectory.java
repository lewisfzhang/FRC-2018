package com.team254.frc2018.auto.actions;

import com.team254.frc2018.RobotState;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.subsystems.Drive;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;

public class DriveTrajectory implements Action {
    private static final Drive mDrive = Drive.getInstance();
    private static final RobotState mRobotState = RobotState.getInstance();

    private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTrajectory;
    private final boolean mIsReversed;

    public DriveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, boolean isReversed) {
        mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
        mIsReversed = isReversed;
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
        if(mIsReversed) {
            mRobotState.reset(Timer.getFPGATimestamp(), mTrajectory.getState().state().getPose()
                    .transformBy(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180.0))));
        } else {
            mRobotState.reset(Timer.getFPGATimestamp(), mTrajectory.getState().state().getPose());
        }
        mDrive.setTrajectory(mTrajectory, mIsReversed);
    }
}

