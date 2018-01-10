package com.team254.lib.util.drivers;

import edu.wpi.first.wpilibj.SerialPort;

public class RPLidar {
    private static SerialPort mPort;
    private static int BAUD_RATE = 115200;

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
        byte[] buffer = mPort.read(84);
        ExpressScanFrame frame = new ExpressScanFrame();
        frame.ChkSum1 = buffer[0] & 0x0F;
        frame.sync1 = buffer[0] & 0xF0;
    }
    
    private static class ExpressScanFrame {
        int sync1, sync2, ChkSum1, ChkSum2;
        Cabin[] cabins;
        
        private static class Cabin {
            int distance1, distance2, dtheta1, dtheta2;
        }
    }
}