package com.team254.frc2018.auto.actions;

import com.team254.frc2018.subsystems.Drive;
import com.team254.lib.util.DriveSignal;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;

public class CollectDriveData implements Action {
    private Drive mDrive = Drive.getInstance();
    private double mAccel;
    private List<double[]> mVelocityData;
    private double mStartTime = 0.0;
    private boolean isFinished = false;
    private boolean mTurn;

    /**
     * @param data reference to the list where data points should be stored
     * @param accel acceleration to use in percent power/sec
     * @param turn if true turn, if false drive straight
     */
    public CollectDriveData(List<double[]> data, double accel, boolean turn) {
        mVelocityData = data;
        mAccel = accel;
        mTurn = turn;
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
        double percentPower = mAccel * (Timer.getFPGATimestamp() - mStartTime);
        if(percentPower > 1.0) {
            isFinished = true;
            return;
        }
        mDrive.setOpenLoop(new DriveSignal(percentPower, (mTurn ? -1.0 : 1.0) * percentPower));
        mVelocityData.add(new double[]{
                (Math.abs(mDrive.getLeftVelocityNativeUnits()) + Math.abs(mDrive.getRightVelocityNativeUnits())) / 4096.0 * Math.PI * 10, //convert to radians per second
                percentPower * 12.0 //convert to volts
        });
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void done() {
        mDrive.setOpenLoop(DriveSignal.BRAKE);
    }
}
