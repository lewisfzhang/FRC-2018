package com.team254.frc2018;

import com.team254.lib.util.drivers.RPLidar;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SerialPort;

public class Robot extends IterativeRobot {
    private RPLidar mRPLidar;

    @Override
    public void robotInit() {
        mRPLidar = new RPLidar(SerialPort.Port.kUSB);
    }

    @Override
    public void disabledInit() { }

    @Override
    public void autonomousInit() { }

    @Override
    public void teleopInit() {
        mRPLidar.startExpressScan();
    }

    @Override
    public void testInit() { }


    @Override
    public void disabledPeriodic() { }

    @Override
    public void autonomousPeriodic() { }

    @Override
    public void teleopPeriodic() {
        RPLidar.ExpressScanFrame frame = mRPLidar.readExpressScanPacket();
        System.out.println(frame);
    }

    @Override
    public void testPeriodic() { }
}