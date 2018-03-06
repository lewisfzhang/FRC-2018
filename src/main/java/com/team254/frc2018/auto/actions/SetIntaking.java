package com.team254.frc2018.auto.actions;

import com.team254.frc2018.statemachines.SuperstructureStateMachine;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.subsystems.Intake;
import com.team254.frc2018.subsystems.Superstructure;
import com.team254.lib.util.TimeDelayedBoolean;

public class SetIntaking implements Action {
    private static final Superstructure mSuperstructure = Superstructure.getInstance();
    private static final Intake mIntake = Intake.getInstance();

    private final boolean mWaitUntilHasCube;
    private final boolean mMoveToIntakingPosition;
    private final TimeDelayedBoolean mDebounce = new TimeDelayedBoolean();

    public SetIntaking(boolean moveToIntakingPosition, boolean waitUntilHasCube) {
        mWaitUntilHasCube = waitUntilHasCube;
        mMoveToIntakingPosition = moveToIntakingPosition;
    }

    @Override
    public void start() {
        mIntake.clampJaw();
        if(mMoveToIntakingPosition) {
            mSuperstructure.setDesiredAngle(SuperstructureConstants.kIntakePositionAngle);
            mSuperstructure.setDesiredHeight(SuperstructureConstants.kIntakeFloorLevelHeight);
        } else {
            mIntake.getOrKeepCube();
        }
    }

    @Override
    public void update() {
        if(mMoveToIntakingPosition && mSuperstructure.getSuperStructureState() == SuperstructureStateMachine.SystemState.HOLDING_POSITION) {
            mIntake.getOrKeepCube();
        }
    }

    @Override
    public boolean isFinished() {
        if(mWaitUntilHasCube) {
            return mDebounce.update(mIntake.hasCube(), .1);
        } else {
            return true;
        }
    }

    @Override
    public void done() {
        mIntake.clampJaw();
    }
}
