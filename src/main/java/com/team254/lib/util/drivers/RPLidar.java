package com.team254.lib.util.drivers;

import edu.wpi.first.wpilibj.SerialPort;

public class RPLidar {
    private static SerialPort mPort;
    private static final int BAUD_RATE = 115200;

    private static final int EXPRESS_PACKET_SIZE = 84;
    private static final int CABIN_COUNT = 16;

    public RPLidar(SerialPort.Port port) {
        mPort = new SerialPort(BAUD_RATE, port);
    }

    public void startScan() {
        byte[] request =  { (byte) 0xA5, (byte) 0x20 };
        sendRequest(request);
    }

    public void startExpressScan() {
        byte[] request = { (byte) 0xA5, (byte) 0x82, (byte) 0x05, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x22 };
        sendRequest(request);
    }

    public void reset() {
        byte[] request =  { (byte) 0xA5, (byte) 0x40 };
        sendRequest(request);
    }

    public void stop() {
        byte[] request =  { (byte) 0xA5, (byte) 0x25 };
        sendRequest(request);
    }

    private void sendRequest(byte[] request) {
        mPort.write(request, request.length);
    }

    public void readScanPacket() {}

    public void readExpressScanPacket() {
        // make sure we have all 84 bytes available, all being input not guaranteed
        int bytesRead = 0;
        byte[] buffer = new byte[EXPRESS_PACKET_SIZE];
        while (bytesRead < packetSize) {
            byte[] readData = mPort.read(EXPRESS_PACKET_SIZE - bytesRead);
            System.arraycopy(readData, 0, buffer, bytesRead);
            bytesRead += readData.length;
        }
        ExpressScanFrame frame = new ExpressScanFrame();

        // parse headers
        frame.ChkSum = (((int)buffer[1] & 0x0F) << 4) & (buffer[0] & 0x0F);
        frame.sync1 = buffer[0] & 0xF0;
        frame.sync2 = buffer[1] & 0xF0;
        frame.startAngle = (( ((int)buffer[3] & 0x7F) << 8 ) & buffer[2] ) / 64.0f;
        frame.isNewScan = (buffer[3] & (0x80)) > 0;

        // parse cabins

        for (int i = 0; i < CABIN_COUNT; i++) {
            Cabin cabin = new Cabin();
            int ci = 4 + i*5;
            cabin.distance1 = ((int)buffer[ci + 1] << 7) & (buffer[ci] >> 1);
            cabin.distance2 = ((int)buffer[ci + 3] << 7) & (buffer[ci + 2] >> 1);

            cabin.dtheta1 = (buffer[ci] & 0x01) & ((buffer[ci + 4] & (0x0F)) << 1);
            cabin.dtheta2 = (buffer[ci + 2] & 0x01) & ((buffer[ci + 4] & (0xF0)) >> 3);

            frame.cabins[i] = cabin;
        }
    }

    private static class ExpressScanFrame {
        int sync1, sync2, ChkSum;
        Cabin[] cabins = new Cabin[CABIN_COUNT];

        boolean isNewScan;
        float startAngle;

        private static class Cabin {
            int distance1, distance2, dtheta1, dtheta2;
        }
    }
}
