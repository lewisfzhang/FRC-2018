package com.team254.frc2018.subsystems;

import com.team254.frc2018.loops.Loop;
import com.team254.frc2018.loops.Looper;
import com.team254.frc2018.statemachines.SuperstructureStateMachine;
import com.team254.lib.util.LatchedBoolean;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Infrastructure extends Subsystem {

    private static Infrastructure mInstance = new Infrastructure();
    private Superstructure mSuperstructure;
    private Compressor mCompressor;

    private boolean mIsDuringAuto = false;

    public static Infrastructure getInstance() {
        return mInstance;
    }

    private Infrastructure() {
        mCompressor = new Compressor();
        mCompressor.start();

        mSuperstructure = Superstructure.getInstance();
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputToSmartDashboard() {
    }

    @Override
    public void stop() {
        // No-op.
    }

    @Override
    public void zeroSensors() {
        // No-op.
    }

    private void startCompressor() {
        mCompressor.start();
    }

    private void stopCompressor() {
        mCompressor.stop();
    }

    public void setIsDuringAuto(boolean isDuringAuto) {
        mIsDuringAuto = isDuringAuto;
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Infrastructure.this) {
                    boolean elevatorMoving = mSuperstructure.getSuperStructureState() ==
                            SuperstructureStateMachine.SystemState.MOVING_TO_POSITION;
                    if (elevatorMoving || mIsDuringAuto) {
                        stopCompressor();
                    } else {
                        startCompressor();
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }
}
