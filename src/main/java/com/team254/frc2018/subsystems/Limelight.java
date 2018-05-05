package com.team254.frc2018.subsystems;

import com.team254.frc2018.RobotState;
import com.team254.frc2018.loops.ILooper;
import com.team254.frc2018.loops.Loop;
import com.team254.lib.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.List;

public class Limelight extends Subsystem {
    private static final double vpw = 2.0 * Math.tan(Math.toRadians(54.0 / 2.0));
    private static final double vph = 2.0 * Math.tan(Math.toRadians(41.0 / 2.0));
    private static final double kImageCaptureLatency = 11.0 / 1000.0;

    private static Limelight mInstance = new Limelight();
    private static RobotState mRobotState = RobotState.getInstance();

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

    public synchronized double getLatency() {
        double pipelineLatency = mTable.getEntry("tl").getDouble(0.0) / 1000.0;
        return pipelineLatency + kImageCaptureLatency;
    }

    public synchronized void setStream(int id) {
        mTable.getEntry("stream").setNumber(id);
    }

    public synchronized void setCamMode(int id) {
        mTable.getEntry("camMode").setNumber(id);
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
    public List<TargetInfo> getRawTargetInfo() {
        List<TargetInfo> targets = new ArrayList<>();
        for (int i = 0; i < 2; ++i) {
            TargetInfo target = new TargetInfo();

            double nx = mTable.getEntry("tx" + i).getDouble(0.0);
            double ny = mTable.getEntry("ty" + i).getDouble(0.0);

            target.horizontalAngle = Rotation2d.fromRadians(normalizedYtoDegrees(ny));
            target.verticalAngle = Rotation2d.fromRadians(normalizedXtoDegrees(nx));
            target.skew = mTable.getEntry("ts" + i).getDouble(0.0);
            target.area = mTable.getEntry("ta" + i).getDouble(0.0);

            targets.add(target);
        }
        return targets;
    }

    public TargetInfo getTargetInfo() {
        TargetInfo target = new TargetInfo();

        double x = mTable.getEntry("ty").getDouble(0.0);
        double y = mTable.getEntry("tx").getDouble(0.0);

        target.horizontalAngle = Rotation2d.fromDegrees(x);
        target.verticalAngle = Rotation2d.fromDegrees(y);
        target.skew = mTable.getEntry("ts").getDouble(0.0);
        target.area = mTable.getEntry("ta").getDouble(0.0);

        return target;
    }

    /**
     * Converts a normalized x pixel coordinate (-1 to 1) to an angle to the target
     *
     * @param nx normalized x pixel corrdinate
     * @return horizontal angle to target
     */
    private double normalizedXtoDegrees(double nx) {
        double x = vpw / 2.0 * nx;
        return Math.atan2(x, 1);
    }

    /**
     * Converts a normalized y pixel coordinate (-1 to 1) to an angle to the target
     *
     * @param ny normalized y pixel corrdinate
     * @return vertical angle to target
     */
    private double normalizedYtoDegrees(double ny) {
        double y = vph / 2.0 * ny;
        return Math.atan2(y, 1);
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
        public Rotation2d horizontalAngle;
        public Rotation2d verticalAngle;
        public double skew;
        public double area;

        @Override
        public String toString() {
            return "x: " + horizontalAngle + ", y: " + verticalAngle;
        }
    }

    @Override
    public void registerEnabledLoops(ILooper looper) {
        looper.register(new EnabledLoop());
    }

    private class EnabledLoop implements Loop {
        @Override
        public synchronized void onStart(double timestamp) {
        }

        @Override
        public synchronized void onLoop(double timestamp) {
            double latency = getLatency();
            mRobotState.addVisionUpdate(timestamp - latency, getTargetInfo());
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    }
}

