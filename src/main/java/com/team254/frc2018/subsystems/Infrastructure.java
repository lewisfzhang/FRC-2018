package com.team254.frc2018.subsystems;

import com.team254.frc2018.loops.Looper;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Infrastructure extends Subsystem {

    private static Infrastructure mInstance = new Infrastructure();

    public static Infrastructure getInstance() {
        return mInstance;
    }

    // protected PowerDistributionPanel mPdp;
    // protected Compressor mCompressor;

    private void Infrastructure() {
        // mPdp = new PowerDistributionPanel();
        // mCompressor = new Compressor();
        // mCompressor.start();
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputToSmartDashboard() {
        // SmartDashboard.putData(mPdp);
    }

    @Override
    public void stop() {
        // No-op.
    }

    @Override
    public void zeroSensors() {
        // No-op.
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        // No-op.
    }
}
