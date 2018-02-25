package com.team254.frc2018.auto.actions;

import com.team254.frc2018.statemachines.SuperstructureStateMachine;
import com.team254.frc2018.subsystems.Superstructure;
import edu.wpi.first.wpilibj.Timer;

public class SetSuperstructurePosition implements Action {
    private static final Superstructure mSuperstructure = Superstructure.getInstance();

    private final double mHeight;
    private final double mAngle;
    private final boolean mWaitForCompletion;
    private double mStartTime;

    public SetSuperstructurePosition(double height, double angle, boolean waitForCompletion) {
        mHeight = height;
        mAngle = angle;
        mWaitForCompletion = waitForCompletion;
    }

    @Override
    public void start() {
        mSuperstructure.setDesiredHeight(mHeight);
        mSuperstructure.setDesiredAngle(mAngle);
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        if(mWaitForCompletion) {
            return Timer.getFPGATimestamp() - mStartTime > 0.1 && //wait for superstructure looper to update
                    mSuperstructure.getSuperStructureState() == SuperstructureStateMachine.SystemState.HOLDING_POSITION;
        } else {
            return true;
        }
    }

    @Override
    public void done() {
    }
}
