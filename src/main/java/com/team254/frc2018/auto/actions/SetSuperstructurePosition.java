package com.team254.frc2018.auto.actions;

import com.team254.frc2018.statemachines.SuperstructureStateMachine;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.states.SuperstructureState;
import com.team254.frc2018.subsystems.Superstructure;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;

public class SetSuperstructurePosition implements Action {
    private static final Superstructure mSuperstructure = Superstructure.getInstance();
    private static final double kHeightEpsilon = 3.0;
    private static final double kAngleEpsilon = 10.0;

    private final double mHeight;
    private final double mAngle;
    private final boolean mWaitForCompletion;

    public SetSuperstructurePosition(double height, double angle, boolean waitForCompletion) {
        mHeight = height;
        // mHeight = SuperstructureConstants.kElevatorMinHeight;
        mAngle = angle;
        mWaitForCompletion = waitForCompletion;
    }

    @Override
    public void start() {
        mSuperstructure.setDesiredHeight(mHeight);
        mSuperstructure.setDesiredAngle(mAngle);
    }

    @Override
    public void update() {
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
