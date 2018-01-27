package com.team254.frc2018.lidar;

import com.team254.frc2018.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.LinkedList;

/**
 * Stores a set amount of lidar scans.  All interfacing with the lidar should be done through this class.
 */
public class LidarInterface {
    private static LidarInterface mInstance = null;

    private LinkedList<LidarScan> mScans = new LinkedList<>();
    private double prevAngle = 0.0;

    public static LidarInterface getInstance() {
        if(mInstance == null) {
            mInstance = new LidarInterface();
        }
        return mInstance;
    }

    public LidarInterface() {
        mScans.add(new LidarScan());
    }

    public void addPoint(LidarPoint point) {
        if(point.angle < prevAngle) { //crosses the 360-0 threshold.  start a new scan
            SmartDashboard.putString("lidarScan", mScans.getLast().toJsonString()); //output to lidar visualizer
            mScans.add(new LidarScan());
            if(mScans.size() > Constants.kNumScansToStore) {
                mScans.removeFirst();
            }
        }
        mScans.getLast().addPoint(point);
        prevAngle = point.angle;
    }

    public LinkedList<LidarScan> getAllScans() {
        return mScans;
    }

    public LidarScan getCurrentScan() {
        return mScans.getLast();
    }

    public LidarScan getLatestCompleteScan() {
        return mScans.get(mScans.size() > 1 ? mScans.size() - 2 : 0);
    }

}
