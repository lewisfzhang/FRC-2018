package com.team254.frc2018.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends Subsystem {
    private static final double vpw = 2.0 * Math.tan(54.0 / 2.0);
    private static final double vph = 2.0 * Math.tan(41.0 / 2.0);

    private static Limelight mInstance = new Limelight();

    private NetworkTable mTable = NetworkTableInstance.getDefault().getTable("limelight");

    private Limelight() {

    }

    public static Limelight getInstance() {
        return mInstance;
    }

    public synchronized void setLedOn() {
        mTable.getEntry("ledMode").setNumber(0);
    }

    public synchronized void setLedOff() {
        mTable.getEntry("ledMode").setNumber(1);
    }

    public synchronized void setLedBlink() {
        mTable.getEntry("ledMode").setNumber(2);
    }

    /**
     * @return true if the limelight sees any valid targets, false if not
     */
    public boolean seesTarget() {
        return mTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /**
     * @return An array containing 3 TargetInfo objects from the Limelight
     */
    public TargetInfo[] getTargetInfo() {
        TargetInfo[] targets = new TargetInfo[3];
        for(int i = 0; i < 3; ++i) {
            targets[i] = new TargetInfo();

            double nx = mTable.getEntry("tx" + i).getDouble(0.0);
            double ny = mTable.getEntry("ty" + i).getDouble(0.0);

            targets[i].angleX = normalizedXtoDegrees(nx);
            targets[i].angleY = normalizedYtoDegrees(ny);
            targets[i].skew = mTable.getEntry("ts" + i).getDouble(0.0);
            targets[i].area = mTable.getEntry("ta" + i).getDouble(0.0);
        }
        return targets;
    }

    /**
     * Converts a normalized x pixel coordinate (-1 to 1) to an angle to the target
     * @param nx normalized x pixel corrdinate
     * @return horizontal angle to target
     */
    private double normalizedXtoDegrees(double nx) {
        double x = vpw/2.0 * nx;
        return Math.atan2(1, x);
    }

    /**
     * Converts a normalized y pixel coordinate (-1 to 1) to an angle to the target
     * @param ny normalized y pixel corrdinate
     * @return vertical angle to target
     */
    private double normalizedYtoDegrees(double ny) {
        double y = vph/2.0 * ny;
        return Math.atan2(1, y);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
    }

    @Override
    public void stop() {
        setLedOff();
    }

    public class TargetInfo {
        double angleX;
        double angleY;
        double skew;
        double area;
    }
}

