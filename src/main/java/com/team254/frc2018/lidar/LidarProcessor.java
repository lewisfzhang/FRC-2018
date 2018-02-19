package com.team254.frc2018.lidar;

import com.team254.frc2018.Constants;
import com.team254.frc2018.RobotState;
import com.team254.frc2018.lidar.icp.ICP;
import com.team254.frc2018.lidar.icp.Point;
import com.team254.frc2018.lidar.icp.ReferenceModel;
import com.team254.frc2018.lidar.icp.Transform;
import com.team254.frc2018.loops.Loop;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.Iterator;
import java.util.LinkedList;

/**
 * Stores a set amount of lidar scans. All interfacing with the lidar should be
 * done through this class.
 */
public class LidarProcessor implements Loop {
    private static LidarProcessor mInstance = null;

    public static LidarProcessor getInstance() {
        if (mInstance == null) {
            mInstance = new LidarProcessor();
        }
        return mInstance;
    }

    private LidarServer mLidarServer = LidarServer.getInstance();
    private RobotState mRobotState = RobotState.getInstance();

    private LinkedList<LidarScan> mScans = new LinkedList<>();
    private double prev_timestamp = Double.MAX_VALUE;

    private ICP icp = new ICP(ReferenceModel.TOWER, 100);

    public LidarProcessor() {
        mScans.add(new LidarScan());
    }

    int count = 0;

    public void addPoint(LidarPoint point, boolean newScan) {
        if (newScan) { // crosses the 360-0 threshold. start a new scan
            prev_timestamp = Timer.getFPGATimestamp();
            count++;
            if (count > 10) {
                count = 0;
                // SmartDashboard.putString("lidarScan", mScans.getLast().toJsonString());
                // //output to lidar visualizer
            }

            mScans.add(new LidarScan());
            if (mScans.size() > Constants.kChezyLidarNumScansToStore) {
                mScans.removeFirst();
            }
        }
        
        Translation2d cartesian = point.toCartesian();
        if (cartesian != null) {
            getCurrentScan().addPoint(new Point(cartesian), point.timestamp);
        }
    }

    public LinkedList<LidarScan> getAllScans() {
        return mScans;
    }

    public LidarScan getCurrentScan() {
        return mScans.getLast();
    }

    public LidarScan getLatestCompleteScan() {
        return mScans.size() > 1? mScans.get(mScans.size() - 2) : null;
    }

    public Iterable<Point> getAllPoints() {
        return () -> {
            return new Iterator<Point>() {
                Iterator<LidarScan> scanIt = mScans.iterator();
                Iterator<Point> pointIt = null;
                
                @Override
                public Point next() {
                    while (pointIt == null || !pointIt.hasNext()) {
                        pointIt = scanIt.next().getPoints().iterator();
                    }
                    return pointIt.next();
                }
                
                @Override
                public boolean hasNext() {
                    return scanIt.hasNext() || pointIt.hasNext();
                }
            };
        };
    }

    public Pose2d doICP() {
        Pose2d guess = mRobotState.getFieldToLidar(getCurrentScan().getTimestamp());
        return icp.doICP(getAllPoints(), new Transform(guess)).toPose2d();
    }

    @Override
    public void onStart(double timestamp) {
        prev_timestamp = Double.MAX_VALUE;
    }

    @Override
    public void onLoop(double timestamp) {
        if (Timer.getFPGATimestamp() - prev_timestamp > Constants.kChezyLidarRestartTime) {
            if (mLidarServer.isRunning()) {
                System.out.println("Lidar timed out. Restarting");
                mLidarServer.stop();
            } else {
                if (!mLidarServer.isEnding()) {
                    if (mLidarServer.start()) {
                        prev_timestamp = Timer.getFPGATimestamp();
                    }
                }
            }
        }
    }

    @Override
    public void onStop(double timestamp) {
    }
}