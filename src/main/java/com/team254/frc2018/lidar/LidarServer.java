package com.team254.frc2018.lidar;

import com.team254.frc2018.Constants;
import edu.wpi.first.wpilibj.Timer;

import java.io.BufferedReader;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

public class LidarServer {
    private static LidarServer mInstance = null;
    private LidarInterface mLidarInterface = LidarInterface.getInstance();
    private UdbBridge mBridge;
    private static BufferedReader mBufferedReader;
    private boolean mRunning = false;
    private Thread mThread;

    public static LidarServer getInstance() {
        if (mInstance == null) {
            mInstance = new LidarServer();
        }
        return mInstance;
    }

    public LidarServer() {
        mBridge = new UdbBridge();
        mThread = new Thread(new SocketThread());
    }

    public boolean start() {
        if(!mRunning) {
            if((mBufferedReader = mBridge.start()) != null) {
                mRunning = true;
                mThread.start();
                return true;
            }
        } else {
            System.out.println("Error: Server already running");
        }
        return false;
    }

    public boolean stop() {
        if(mRunning) {
            mRunning = false;
            try {
                System.out.println("About to join");
                mThread.join();
            } catch (Exception e) {
                e.printStackTrace();
            }
            return mBridge.stop();
        } else {
            System.out.println("Error: Server not running");
        }
        return false;
    }

    private void handleUpdate(String packet) {
        String[] lines = packet.split("-");
        long curSystemTime = System.currentTimeMillis();
        double curFPGATime = Timer.getFPGATimestamp();

        for (String line : lines) {
            String[] parts = line.split(",");
            if (parts.length == 3) {
                try {
                    long ts = Long.parseLong(parts[0]);
                    long ms_ago = curSystemTime - ts;
                    double normalizedTs = curFPGATime - (ms_ago / 1000.0f);
                    double angle = Double.parseDouble(parts[1]);
                    double distance = Double.parseDouble(parts[2]);
                    mLidarInterface.addPoint(new LidarPoint(normalizedTs, angle, distance));
                } catch (java.lang.NumberFormatException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    private class SocketThread implements Runnable {
        @Override
        public void run() {
            while(mRunning) {
                String line;
                try {
                    while ((line = mBufferedReader.readLine()) != null) {
                        handleUpdate(line);
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            System.out.println("Thread ending");
        }
    }



}
