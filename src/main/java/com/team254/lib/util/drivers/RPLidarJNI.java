package com.team254.lib.util.drivers;

public class RPLidarJNI {
    static {
        try {

            System.loadLibrary("rplidarjni");

        } catch (UnsatisfiedLinkError e) {
            System.err.println("Native code library failed to load.\n" + e);
        }
    }

    public static class DataPoint {
        public double distance, angle;
    }

    public native void init();

    public native boolean checkHealth();

    public native void startMotor();

    public native void startScan();

    public native String grabScanData();

    public native void stop();

    public void parseScanData(String scanData) {
        //TODO Use REGEX to store scanData in a DataPoint array
    }

    public void outputToNetworkTables() {
        //TODO Parse DataPoint array, transform to xy coordinates, and write to NetworkTables using SmartDashboard
    }
}
