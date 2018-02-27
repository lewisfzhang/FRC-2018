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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

/**
 * Receives LIDAR points from the {@link LidarServer}, stores a set number
 * of scans/revolutions, and provides methods for processing the data.
 * <p>
 * All interfacing with the LIDAR should be done through this class.
 * 
 * @see Constants.kChezyLidarNumScansToStore
 * @see doICP()
 * @see getTowerPosition()
 */
public class LidarProcessor implements Loop {
    private static LidarProcessor mInstance = null;

    public static LidarProcessor getInstance() {
        if (mInstance == null) {
            mInstance = new LidarProcessor();
        }
        return mInstance;
    }

    private RobotState mRobotState = RobotState.getInstance();

    private LinkedList<LidarScan> mScans = new LinkedList<>();
    private double prev_timestamp;

    private ICP icp = new ICP(ReferenceModel.TOWER, 100);

    private PrintWriter dataLogFile;

    private final ReadWriteLock lock = new ReentrantReadWriteLock();

    private LidarProcessor() {
        mScans.add(new LidarScan());
        try {
            dataLogFile = new PrintWriter(Constants.kLidarLogPath);
        } catch (FileNotFoundException e) {
            System.err.println("Failed to open lidar log file:");
            e.printStackTrace();
        }
    }

    public void addPoint(LidarPoint point, boolean newScan) {
        SmartDashboard.putNumber("LIDAR last_angle", point.angle);
        
        Translation2d cartesian = point.toCartesian();
        dataLogFile.println(point.angle+" "+point.distance+" "+cartesian.x()+" "+cartesian.y());
        
        lock.writeLock().lock();
        try {
            if (newScan) { // crosses the 360-0 threshold. start a new scan
                prev_timestamp = Timer.getFPGATimestamp();
                
                long start = System.nanoTime();
                Translation2d towerPos = getTowerPosition();
                long end = System.nanoTime();
                SmartDashboard.putNumber("towerPos_ms", (end-start)/1000000);
                SmartDashboard.putNumber("towerPosX", towerPos.x());
                SmartDashboard.putNumber("towerPosY", towerPos.y());

                mScans.add(new LidarScan());
                if (mScans.size() > Constants.kChezyLidarNumScansToStore) {
                    mScans.removeFirst();
                }
            }
            
            if (!excludePoint(cartesian.x(), cartesian.y())) {
                getCurrentScan().addPoint(new Point(cartesian), point.timestamp);
            }
        } finally {
            lock.writeLock().unlock();
        }
    }
    
    private static final double FIELD_WIDTH = 27*12, FIELD_HEIGHT = 54*12;
    private static final double RECT_RX = FIELD_WIDTH/5, RECT_RY = FIELD_HEIGHT/2;
    private static final double FIELD_CX = FIELD_WIDTH/2, FIELD_CY = FIELD_HEIGHT/2;
    private static final double RECT_X_MIN = FIELD_CX-RECT_RX, RECT_X_MAX = FIELD_CX+RECT_RX,
                                RECT_Y_MIN = FIELD_CY-RECT_RY, RECT_Y_MAX = FIELD_CY+RECT_RY;
    
    private static boolean excludePoint(double x, double y) {
        return x < RECT_X_MIN || x > RECT_X_MAX ||
               y < RECT_Y_MIN || y > RECT_Y_MAX;
    }

    private LidarScan getCurrentScan() {
        return mScans.getLast();
    }
    
    private final Iterable<Point> allPoints = () -> {
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
    
    private Point getAveragePoint() {
        double sumX = 0, sumY = 0;
        int n = 0;
        for (Point p : allPoints) {
            sumX += p.x;
            sumY += p.y;
            n++;
        }
        return new Point(sumX/n, sumY/n);
    }

    public Pose2d doICP() {
        lock.readLock().lock();
        try {
            Pose2d guess = mRobotState.getFieldToLidar(getCurrentScan().getTimestamp());
            return icp.doICP(allPoints, new Transform(guess).inverse()).inverse().toPose2d();
        } finally {
            lock.readLock().unlock();
        }
    }

    public Translation2d getTowerPosition() {
        lock.readLock().lock();
        try {
            Point avg = getAveragePoint();
            Transform trans = icp.doICP(allPoints, new Transform(0, avg.x, avg.y));
            return trans.apply(icp.reference).getMidpoint().toTranslation2d();
        } finally {
            lock.readLock().unlock();
        }
    }

    public void setPrevTimestamp(double time) {
        lock.writeLock().lock();
        try {
            prev_timestamp = time;
        } finally {
            lock.writeLock().unlock();
        }
    }

    public double getPrevTimestamp() {
        lock.readLock().lock();
        try {
            return prev_timestamp;
        } finally {
            lock.readLock().unlock();
        }
    }

    @Override
    public void onStart(double timestamp) {
        setPrevTimestamp(Double.NEGATIVE_INFINITY);
    }

    @Override
    public void onLoop(double timestamp) {
        LidarServer lidarServer = LidarServer.getInstance();
        if (timestamp - getPrevTimestamp() > Constants.kChezyLidarRestartTime) {
            if (lidarServer.isRunning()) {
                System.err.println("Lidar timed out. Restarting");
                lidarServer.stop();
            } else if (!lidarServer.isEnding() && lidarServer.start()) {
                setPrevTimestamp(timestamp);
            }
        }
    }

    @Override
    public void onStop(double timestamp) {
    }
}