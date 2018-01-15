package com.team254.frc2018;

import com.team254.lib.util.drivers.RPLidar;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SerialPort;

public class Robot extends IterativeRobot {
    private RPLidar mRPLidar = new RPLidar(SerialPort.Port.kUSB);

    @Override
    public void robotInit() {
        System.out.println("Robot Init");
    }

    @Override
    public void disabledInit() { }

    @Override
    public void autonomousInit() { }

    @Override
    public void teleopInit() {
        System.out.println("Teleop Init");

//        mRPLidar.startExpressScan();
        System.out.println("Initial health: " + mRPLidar.checkHealth());
        mRPLidar.startMotor();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        System.out.println("Start Monitor Health: " + mRPLidar.checkHealth());
//        mRPLidar.forceScan();
        mRPLidar.startScan();
    }

    @Override
    public void testInit() { }


    @Override
    public void disabledPeriodic() { }

    @Override
    public void autonomousPeriodic() { }

    @Override
    public void teleopPeriodic() {
        System.out.println("Teleop Periodic");
//        try {
//            Thread.sleep(1000);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }

//         RPLidar.ExpressScanFrame frame = mRPLidar.readExpressScanPacket();
//         System.out.println(frame);


        byte[] data = mRPLidar.dataDump();
        System.out.println("Read " + data.length + " units of data");

        for (int i = 0; i < data.length; i++) {
            System.out.print(Integer.toHexString(Byte.toUnsignedInt(data[i])) + " ");
        }

        System.out.println("\nHealth: " + mRPLidar.checkHealth());
    }

    @Override
    public void testPeriodic() { }
}