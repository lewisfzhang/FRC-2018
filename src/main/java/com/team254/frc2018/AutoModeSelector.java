package com.team254.frc2018;

import com.team254.frc2018.auto.AutoModeBase;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoModeSelector {
    private static class AutoModeCreator {
        private final String mDashboardName;
        private final AutoModeBase mAutoMode;

        private AutoModeCreator(String dashboardName, AutoModeBase mode) {
            mDashboardName = dashboardName;
            mAutoMode = mode;
        }
    }

    //TODO: Fill this array with actual AutoModeCreator Objects (not null)
    private static final AutoModeCreator mDefaultMode = null;
    private static final AutoModeCreator[][] mAllModes = {
        {null, null, null},
        {null, null, null},
        {null, null, null},
        {null, null, null}
    };

    public static AutoModeBase getSelectedAutoMode() {
        int auto_alliance_confidence = (int) SmartDashboard.getNumber("Auto Alliance Confidence", -1.0);
        int auto_starting_pose = (int) SmartDashboard.getNumber("Auto Starting Pose", -1.0);

        if (auto_alliance_confidence < 0 || auto_starting_pose < 0) {
            DriverStation.reportError("NO AUTO PARAMETERS SET!!!!!", false);
            return mDefaultMode.mAutoMode;
        }

        AutoModeCreator mModeCreator = mAllModes[auto_alliance_confidence][auto_starting_pose];
        SmartDashboard.putString("Selected Auto Mode", mModeCreator.mDashboardName);

        return mModeCreator.mAutoMode;
    }
}
