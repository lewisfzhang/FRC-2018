package com.team254.frc2018.auto.creators;

import com.team254.frc2018.AutoFieldState;
import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.modes.DoNothingMode;

public class ScaleOnlyAutoModeCreator implements AutoModeCreator {

    private final AutoModeBase hardScaleMode;
    private final AutoModeBase easyScaleMode;
    private boolean mRobotStartedOnLeft;

    public ScaleOnlyAutoModeCreator(boolean robotStartedOnLeft) {
        mRobotStartedOnLeft = robotStartedOnLeft;
        hardScaleMode = new DoNothingMode();
        easyScaleMode = new DoNothingMode();
    }

    @Override
    public AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState) {
        System.out.print("Getting ScaleOnlyAutoMode for " + mRobotStartedOnLeft + " / " + fieldState.toString());
        if ((mRobotStartedOnLeft && fieldState.getScaleSide() == AutoFieldState.Side.LEFT) ||
                (!mRobotStartedOnLeft && fieldState.getScaleSide() == AutoFieldState.Side.RIGHT)) {
                return easyScaleMode;
        } else {
            return hardScaleMode;
        }
    }
}
