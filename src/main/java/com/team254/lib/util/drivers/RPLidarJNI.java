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
        public DataPoint(double dist, double a, long ts) {
            distance = dist;
            angle = a;
            timestamp = ts;
        }
        public double distance, angle;
        public long timestamp;
    }

    public native void init();

    public native boolean checkHealth();

    public native void startMotor();

    public native void startScan();

    /// arrays to hold scan data from the native code
    public static final int DATA_BUFFER_LENGTH = 360*2;
    private double[] distances  = new double[DATA_BUFFER_LENGTH];
    private double[] angles     = new double[DATA_BUFFER_LENGTH];
    private long[]   timestamps = new long[DATA_BUFFER_LENGTH];

    /**
     * Native method to get one revolution's worth of scan data
     * via the RPLIDAR SDK. Returns the number of data points
     * retrieved, or -1 on failure; the data itself is put into
     * the three arrays passed as parameters. The measurements
     * are sorted in ascending order by angle.
     * 
     * This method may block for a short period of time to wait
     * for a full revolution of data to be available.
     */
    private native int grabRawScanData(double[] distances, double[] angles, long[] timestamps);

    /**
     * Gets the next batch of data with grabRawScanData() and
     * returns it as an array of DataPoints.
     */
    public DataPoint[] getScanData() {
        int count = grabRawScanData(distances, angles, timestamps);
        DataPoint[] array = new DataPoint[count];
        for (int i = 0; i < count; i++) {
            array[i] = new DataPoint(distances[i], angles[i], timestamps[i]);
        }
        return array;
    }

    public native void stop();

    public void outputToNetworkTables() {
        //TODO Parse DataPoint array, transform to xy coordinates, and write to NetworkTables using SmartDashboard
    }
}
