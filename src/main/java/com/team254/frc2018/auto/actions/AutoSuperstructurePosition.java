package com.team254.frc2018.auto.actions;

import com.team254.frc2018.AutoFieldState;
import com.team254.frc2018.states.SuperstructureState;
import com.team254.frc2018.subsystems.CheesyVision2;
import com.team254.frc2018.subsystems.Superstructure;
import com.team254.lib.util.Util;

public class AutoSuperstructurePosition implements Action {
    private static final Superstructure mSuperstructure = Superstructure.getInstance();
    private static final CheesyVision2 mCheesyVision2 = CheesyVision2.getInstance();
    private static final double kHeightEpsilon = 3.0;
    private static final double kAngleEpsilon = 10.0;

    private final double mLowHeight, mNeutralHeight, mHighHeight, mDefaultHeight;
    private final double mAngle;
    private final boolean mWaitForCompletion;
    
    private double mDesiredHeight;

    public AutoSuperstructurePosition(double lowHeight, double neutralHeight, double highHeight,
                                    double defaultHeight, double angle, boolean waitForCompletion) {
        mLowHeight = lowHeight;
        mNeutralHeight = neutralHeight;
        mHighHeight = highHeight;
        mDefaultHeight = defaultHeight;
        mAngle = angle;
        mWaitForCompletion = waitForCompletion;
    }
    
    protected double getDesiredHeight() {
        if (mCheesyVision2.getError()) return mDefaultHeight;
        
        double tip = mCheesyVision2.getTip();
        AutoFieldState state = new AutoFieldState();
        state.setSides();
        if (state.getScaleSide() == AutoFieldState.Side.LEFT)
            tip = -tip;
        
        double height = mNeutralHeight;
        if (tip < 0) height = mLowHeight;
        if (tip > 0) height = mHighHeight;
        return height;
    }

    @Override
    public void start() {
        mDesiredHeight = getDesiredHeight();
        mSuperstructure.setDesiredHeight(mDesiredHeight);
        mSuperstructure.setDesiredAngle(mAngle);
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        if(mWaitForCompletion) {
            SuperstructureState state = mSuperstructure.getObservedState();
            return Util.epsilonEquals(state.height, mDesiredHeight, kHeightEpsilon) &&
                   Util.epsilonEquals(state.angle, mAngle, kAngleEpsilon);
        } else {
            return true;
        }
    }

    @Override
    public void done() {
    }
}
