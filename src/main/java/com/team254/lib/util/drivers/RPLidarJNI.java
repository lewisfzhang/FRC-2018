package com.team254.lib.util.drivers;

public class RPLidarJNI {
    static {
        System.loadLibrary("");
    }

    public static class DataPoint {
        public double distance, angle;
    }

    public native void init();

    public native boolean checkHealth();

    public native void startMotor();

    public native void startScan();

    public native void grabScanData(double[] data);

    public native void stop();
}
