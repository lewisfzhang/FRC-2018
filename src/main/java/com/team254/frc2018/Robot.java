package com.team254.frc2018;

import com.team254.frc2018.subsystems.ProtoIntake;
import com.team254.frc2018.subsystems.Subsystem;
import edu.wpi.first.wpilibj.IterativeRobot;
import com.team254.frc2018.loops.*;

import java.util.Arrays;
import java.util.List;

public class Robot extends IterativeRobot {

    private Looper mEnabledLooper = new Looper();
    private SubsystemManager mSubsystemManager = new SubsystemManager(Arrays.asList(ProtoIntake.getInstance()));

    @Override
    public void robotInit() {
        mSubsystemManager.registerEnabledLoops(mEnabledLooper);
    }

    @Override
    public void disabledInit() {
        mEnabledLooper.stop();
    }

    @Override
    public void autonomousInit() { }

    @Override
    public void teleopInit() {
        mEnabledLooper.start();
    }

    @Override
    public void testInit() { }


    @Override
    public void disabledPeriodic() { }
    
    @Override
    public void autonomousPeriodic() { }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testPeriodic() { }
}