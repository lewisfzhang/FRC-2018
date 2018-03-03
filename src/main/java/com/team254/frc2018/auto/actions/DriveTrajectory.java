package com.team254.frc2018.auto.actions;

import com.team254.frc2018.subsystems.Drive;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.DriveSignal;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;

public class DriveTrajectory implements Action {
    private static final Drive mDrive = Drive.getInstance();

    private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTrajectory;

    public DriveTrajectory(List<Pose2d> waypoints, double max_vel, double max_accel, double max_voltage) {
        mTrajectory = mDrive.generateTrajectory(waypoints, max_vel, max_accel, max_voltage);
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
