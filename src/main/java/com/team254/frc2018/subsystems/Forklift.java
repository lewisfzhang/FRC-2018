package com.team254.frc2018.subsystems;

import com.team254.frc2018.Constants;
import com.team254.frc2018.loops.Looper;
import edu.wpi.first.wpilibj.Solenoid;

public class Forklift extends Subsystem {
    private static Forklift mInstance = null;

    public static Forklift getInstance() {
        if (mInstance == null) {
            mInstance = new Forklift();
        }

        return mInstance;
    }

    private Solenoid mDeploySolenoid;

    private Forklift() {
        mDeploySolenoid = new Solenoid(Constants.kForkliftDeploySolenoid);
    }

    public synchronized void deploy() {
        mDeploySolenoid.set(true);
    }

    public synchronized void retract() {
        mDeploySolenoid.set(false);
    }

    @Override
    public boolean checkSystem() { return true; }

    @Override
    public void outputToSmartDashboard(){}

    @Override
    public void stop() {}

    @Override
    public void zeroSensors() {}

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {}
}