package com.team254.frc2018.auto.actions;

import com.team254.frc2018.statemachines.SuperstructureStateMachine;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.states.SuperstructureState;
import com.team254.frc2018.subsystems.CheesyVision2;
import com.team254.frc2018.subsystems.Superstructure;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;

public class AutoSuperstructurePosition implements Action {
    private static final CheesyVision2 mCheesyVision2 = CheesyVision2.getInstance();
    private static final Superstructure mSuperstructure = Superstructure.getInstance();
    private static final double kHeightEpsilon = 2.0;
    private static final double kAngleEpsilon = 5.0;

    private final int mNumCubes;
    private double mHeight;
    private final double mAngle;
    private final boolean mWaitForCompletion;
    private final boolean mUseKickstand;

    public AutoSuperstructurePosition(int numCubes, double angle, boolean waitForCompletion, boolean useKickstand) {
        mNumCubes = numCubes;
        mAngle = angle;
        mWaitForCompletion = waitForCompletion;
        mUseKickstand = useKickstand;
    }

    @Override
    public void start() {
        mHeight = mCheesyVision2.getDesiredHeight((mAngle == SuperstructureConstants.kScoreBackwardsAngle), mNumCubes, mUseKickstand);
        mSuperstructure.setDesiredHeight(mHeight);
        mSuperstructure.setDesiredAngle(mAngle);
    }

    @Override
    public void update() {
        mHeight = mCheesyVision2.getDesiredHeight((mAngle == SuperstructureConstants.kScoreBackwardsAngle), mNumCubes, mUseKickstand);
        mSuperstructure.setDesiredHeight(mHeight);
    }

    @Override
    public boolean isFinished() {
        if(mWaitForCompletion) {
            SuperstructureState state = mSuperstructure.getObservedState();
            return Util.epsilonEquals(state.height, mHeight, kHeightEpsilon) &&
                    Util.epsilonEquals(state.angle, mAngle, kAngleEpsilon);
        } else {
            return true;
        }
    }

    @Override
    public void done() {
    }
}
