package com.team254.frc2018.auto.creators;

import com.team254.frc2018.AutoFieldState;
import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.modes.*;

public class SwitchAndScaleAutoModeCreator implements AutoModeCreator {

    private boolean mRobotStartedOnLeft;
    private NearScaleNearSwitchMode mEasyScaleEasySwitch;
    private NearScaleFarSwitchMode mEasyScaleHardSwitch;
    private FarScaleNearSwitchMode mHardScaleEasySwitch;
    private FarScaleFarSwitchMode mHardScaleHardSwitch;

    public SwitchAndScaleAutoModeCreator(boolean robotStartedOnLeft) {
        // Make all 4 possible auto modes, and they make their paths
        System.out.println("SwitchAndScaleAutoModeCreator, make some paths?" + robotStartedOnLeft);
        mRobotStartedOnLeft = robotStartedOnLeft;
        mEasyScaleEasySwitch = new NearScaleNearSwitchMode(robotStartedOnLeft);
        mEasyScaleHardSwitch = new NearScaleFarSwitchMode(robotStartedOnLeft);
        mHardScaleEasySwitch = new FarScaleNearSwitchMode(robotStartedOnLeft);
        mHardScaleHardSwitch = new FarScaleFarSwitchMode(robotStartedOnLeft);
    }

    @Override
    public AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState) {
        // Return which of the 4 modes is correct for this arrangement
        System.out.print("Getting SwitchAndScaleAutoModeCreator for " + mRobotStartedOnLeft + " / " + fieldState.toString());
        if (mRobotStartedOnLeft) {
            if (fieldState.getScaleSide() == AutoFieldState.Side.LEFT && fieldState.getOurSwitchSide() == AutoFieldState.Side.LEFT) {
                return mEasyScaleEasySwitch;
            }
            if (fieldState.getScaleSide() == AutoFieldState.Side.LEFT && fieldState.getOurSwitchSide() == AutoFieldState.Side.RIGHT) {
                return mEasyScaleHardSwitch;
            }
            if (fieldState.getScaleSide() == AutoFieldState.Side.RIGHT && fieldState.getOurSwitchSide() == AutoFieldState.Side.LEFT) {
                return mHardScaleEasySwitch;
            }
            if (fieldState.getScaleSide() == AutoFieldState.Side.RIGHT && fieldState.getOurSwitchSide() == AutoFieldState.Side.RIGHT) {
                return mHardScaleHardSwitch;
            }
        } else {
            if (fieldState.getScaleSide() == AutoFieldState.Side.LEFT && fieldState.getOurSwitchSide() == AutoFieldState.Side.LEFT) {
                return mHardScaleHardSwitch;
            }
            if (fieldState.getScaleSide() == AutoFieldState.Side.LEFT && fieldState.getOurSwitchSide() == AutoFieldState.Side.RIGHT) {
                return mHardScaleEasySwitch;
            }
            if (fieldState.getScaleSide() == AutoFieldState.Side.RIGHT && fieldState.getOurSwitchSide() == AutoFieldState.Side.LEFT) {
                return mEasyScaleHardSwitch;
            }
            if (fieldState.getScaleSide() == AutoFieldState.Side.RIGHT && fieldState.getOurSwitchSide() == AutoFieldState.Side.RIGHT) {
                return mEasyScaleEasySwitch;
            }
        }
        return null;
    }
}
