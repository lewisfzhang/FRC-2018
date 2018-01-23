package com.team254.frc2018.lidar;

import edu.wpi.first.wpilibj.Timer;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

public class UDPLidar {
    DatagramSocket mSocketServer;
    boolean mRunning = true;
    Thread mThread;
    LidarScan mLidarScan = new LidarScan();

    private class SocketThread implements Runnable {

        @Override
        public void run() {
            byte[] receiveData = new byte[20000];
            while(mRunning)
            {
                DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
                try {
                    mSocketServer.receive(receivePacket);
                } catch (IOException e) {
                    e.printStackTrace();
                }

                long curSystemTime = System.currentTimeMillis();
                double curFPGATime = Timer.getFPGATimestamp();
                String packet = new String( receivePacket.getData());
                String[] lines = packet.split("\n");
                for (String line : lines) {
                    String[] parts = line.split(",");
                    if (parts.length == 3) {
                        try {
                            long ts = Long.parseLong(parts[0]);
                            long ms_ago = curSystemTime - ts;
                            double normalizedTs = curFPGATime - (ms_ago / 1000.0f);
                            double angle = Double.parseDouble(parts[1]);
                            double distance = Double.parseDouble(parts[2]);
                            mLidarScan.timestamp = normalizedTs;
                            mLidarScan.angle = angle;
                            mLidarScan.distance = distance;

                            // TODO - Do Something with mLidarScan (dont print :P)
                        } catch (java.lang.NumberFormatException e) {
                            e.printStackTrace();
                        }
                    }
                }

            }
        }
    }

    public UDPLidar() {
        try {
            mSocketServer = new DatagramSocket(9254);
            mThread = new Thread(new SocketThread());
            mThread.run();
        } catch (SocketException e) {
            e.printStackTrace();
        }
    }

}
