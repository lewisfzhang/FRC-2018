package com.team254.frc2018.auto.actions;

import com.team254.frc2018.RobotState;
import com.team254.frc2018.subsystems.Drive;
import com.team254.lib.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

public class OverrideTrajectory extends RunOnceAction{
    @Override
    public void runOnce() {
        Drive.getInstance().overrideTrajectory(true);
    }
}
