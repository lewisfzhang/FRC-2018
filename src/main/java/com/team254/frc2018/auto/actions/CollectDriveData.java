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
    private boolean mReverse;
    private double mMaxPower;
    private boolean mHighGear;

    /**
     * @param data reference to the list where data points should be stored
     * @param rampRate power ramp rate in percent power/sec
     * @param maxPower max power to ramp up to before finishing
     * @param highGear use high gear or low
     * @param reverse if true drive in reverse, if false drive normally
     * @param turn if true turn, if false drive straight
     *
     */
    public CollectDriveData(List<double[]> data, double rampRate, double maxPower, boolean highGear, boolean reverse, boolean turn) {
        mVelocityData = data;
        mAccel = rampRate;
        mMaxPower = maxPower;
        mHighGear = highGear;
        mReverse = reverse;
        mTurn = turn;
    }

    @Override
    public void start() {
        mDrive.setHighGear(mHighGear);
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
        double percentPower = mAccel * (Timer.getFPGATimestamp() - mStartTime);
        if(percentPower > mMaxPower) {
            isFinished = true;
            return;
        }
        mDrive.setOpenLoop(new DriveSignal((mReverse ? -1.0 : 1.0) * percentPower, (mReverse ? -1.0 : 1.0) * (mTurn ? -1.0 : 1.0) * percentPower));
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
