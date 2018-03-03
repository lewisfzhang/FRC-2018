package com.team254.frc2018.auto;

import com.team254.frc2018.AutoFieldState;
import com.team254.frc2018.auto.modes.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoModeSelector {
    private static class AutoModeCreator {
        private final String mDashboardName;
        private final AutoModeBase mAutoMode;

        private AutoModeCreator(String mDashboardName, AutoModeBase mAutoMode) {
            this.mDashboardName = mDashboardName;
            this.mAutoMode = mAutoMode;
        }
    }

    private static final AutoModeCreator mDefaultMode = new AutoModeCreator("Stand Still Mode", new StandStillMode());
    private static final AutoModeCreator[] mLLModes = {
        //TODO
    };
    private static final AutoModeCreator[] mLRModes = {
        //TODO
    };
    private static final AutoModeCreator[] mRLModes = {
        //TODO
    };
    private static final AutoModeCreator[] mRRModes = {
        //TODO
    };

    private static final SendableChooser<AutoModeCreator> mLLChooser = new SendableChooser<>();
    private static final SendableChooser<AutoModeCreator> mLRChooser = new SendableChooser<>();
    private static final SendableChooser<AutoModeCreator> mRLChooser = new SendableChooser<>();
    private static final SendableChooser<AutoModeCreator> mRRChooser = new SendableChooser<>();
    private static final SendableChooser<AutoFieldState.StartingPose> mStartingPoseChooser = new SendableChooser<>();


    public static void initAutoModeSelector() {
        mLLChooser.addDefault(mDefaultMode.mDashboardName, mDefaultMode);
        mLRChooser.addDefault(mDefaultMode.mDashboardName, mDefaultMode);
        mRLChooser.addDefault(mDefaultMode.mDashboardName, mDefaultMode);
        mRRChooser.addDefault(mDefaultMode.mDashboardName, mDefaultMode);

        for (AutoModeCreator mMode : mLLModes) {
            mLLChooser.addObject(mMode.mDashboardName, mMode);
        }

        for (AutoModeCreator mMode : mLRModes) {
            mLRChooser.addObject(mMode.mDashboardName, mMode);
        }
        
        for (AutoModeCreator mMode : mRLModes) {
            mRLChooser.addObject(mMode.mDashboardName, mMode);
        }

        for (AutoModeCreator mMode : mRRModes) {
            mRRChooser.addObject(mMode.mDashboardName, mMode);
        }

        mStartingPoseChooser.addDefault("Left", AutoFieldState.StartingPose.LEFT); //TODO set default
        mStartingPoseChooser.addObject("Center", AutoFieldState.StartingPose.CENTER);
        mStartingPoseChooser.addObject("Right", AutoFieldState.StartingPose.RIGHT);

        SmartDashboard.putData("Switch: L/Scale: L Auto Mode", mLLChooser);
        SmartDashboard.putData("Switch: L/Scale: R Auto Mode", mLRChooser);
        SmartDashboard.putData("Switch: R/Scale: L Auto Mode", mRLChooser);
        SmartDashboard.putData("Switch: R/Scale: R Auto Mode", mRRChooser);
        SmartDashboard.putData("Starting Position", mStartingPoseChooser);
    }

    public static AutoModeBase getSelectedAutoMode() {
        AutoModeCreator mSelectedAutoMode;

        if (AutoFieldState.getAllianceSwitchSide() == AutoFieldState.Side.LEFT && AutoFieldState.getScaleSide() == AutoFieldState.Side.LEFT) {
            mSelectedAutoMode = mLLChooser.getSelected();
        } else if (AutoFieldState.getAllianceSwitchSide() == AutoFieldState.Side.LEFT && AutoFieldState.getScaleSide() == AutoFieldState.Side.RIGHT) {
            mSelectedAutoMode = mLRChooser.getSelected();
        } else if (AutoFieldState.getAllianceSwitchSide() == AutoFieldState.Side.RIGHT && AutoFieldState.getScaleSide() == AutoFieldState.Side.LEFT) {
            mSelectedAutoMode = mRLChooser.getSelected();
        } else if (AutoFieldState.getAllianceSwitchSide() == AutoFieldState.Side.RIGHT && AutoFieldState.getScaleSide() == AutoFieldState.Side.RIGHT) {
            mSelectedAutoMode = mRRChooser.getSelected();
        } else {
            mSelectedAutoMode = mDefaultMode;
        }
        
        SmartDashboard.putString("Selected Auto Mode", mSelectedAutoMode.mDashboardName);

        return mSelectedAutoMode.mAutoMode;
    }

    public static AutoFieldState.StartingPose getSelectedStartingPose() {
        return mStartingPoseChooser.getSelected();
    }
}