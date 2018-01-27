package com.team254.frc2018.lidar;

import com.team254.frc2018.Constants;
import edu.wpi.first.wpilibj.Timer;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

public class LidarServer {
    private static LidarServer mInstance = null;
    private LidarInterface mLidarInterface = LidarInterface.getInstance();
    private UdbBridge mBridge = new UdbBridge();
    private DatagramSocket mSocketServer;
    private boolean mRunning = false;
    private Thread mThread;

    public static LidarServer getInstance() {
        if (mInstance == null) {
            mInstance = new LidarServer(Constants.kLidarUDPPort);
        }
        return mInstance;
    }

    public LidarServer(int port) {
        try {
            mSocketServer = new DatagramSocket(port);
            mThread = new Thread(new SocketThread());
        } catch (SocketException e) {
            e.printStackTrace();
        }
    }

    public boolean start() {
        if(!mRunning) {
            if(mBridge.start()) {
                mRunning = true;
                mThread.run();
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
            return mBridge.stop();
        } else {
            System.out.println("Error: Server not running");
        }
        return false;
    }

    private void handleUpdate(byte[] update) {
        String packet = new String(update);
        String[] lines = packet.split("\n");

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
            byte[] receiveData = new byte[200000];
            
            while(mRunning) {
                DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
                System.out.println(receivePacket.getLength());

                try {
                	// gets stuck on the next line after 42 loops (2162 or 2163 messages)
                    mSocketServer.receive(receivePacket);
                } catch (IOException e) {
                    e.printStackTrace();
                }

                handleUpdate(receivePacket.getData());
            }
        }
    }



}
