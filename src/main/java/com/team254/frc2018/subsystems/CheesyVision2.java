package com.team254.frc2018.subsystems;

import com.team254.frc2018.loops.ILooper;
import com.team254.frc2018.loops.Loop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Subsystem abstract class, which serves as a basic framework for all robot subsystems. Each subsystem outputs
 * commands to SmartDashboard, has a stop routine (for after each match), and a routine to zero all sensors, which helps
 * with calibration.
 * <p>
 * All Subsystems only have one instance (after all, one robot does not have two drivetrains), and functions get the
 * instance of the drivetrain and act accordingly. Subsystems are also a state machine with a desired state and actual
 * state; the robot code will try to match the two states with actions. Each Subsystem also is responsible for
 * instantializing all member components at the start of the match.
 */
public class CheesyVision2 extends Subsystem {
    
    private static CheesyVision2 mInstance;
    
    public synchronized static CheesyVision2 getInstance() {
        if (mInstance == null) {
            mInstance = new CheesyVision2();
        }
        return mInstance;
    }
    
    private CheesyVision2() {}
    
    
    private double angle = 0, tip = 0;
    
    /**
     * @return the current angle of the scale, in degrees, as seen
     *         from the driver station (positive = right side raised;
     *         negative = left side raised)
     */
    public double getAngle() {
        return angle;
    }
    
    /**
     * @return the current thresholded "height" of the <em>right</em> scale
     *         plate, as seen from the driver station (-1, 0, or +1)
     */
    public double getTip() {
        return tip;
    }
    
    
    @Override
    public boolean checkSystem() {
        return false;
    }
    
    @Override
    public void outputTelemetry() {}
    
    @Override
    public void stop() {}
    
    @Override
    public void registerEnabledLoops(ILooper looper) {
        looper.register(new EnabledLoop());
    }
    
    private class EnabledLoop implements Loop {
        @Override
        public synchronized void onStart(double timestamp) {
        }
        
        @Override
        public synchronized void onLoop(double timestamp) {
            angle = SmartDashboard.getNumber("scaleAngle", Double.NaN);
            tip = SmartDashboard.getNumber("scaleTip", Double.NaN);
            SmartDashboard.putNumber("scaleAngleEcho", angle);
            SmartDashboard.putNumber("scaleTipEcho", tip);
        }
        
        @Override
        public void onStop(double timestamp) {
            stop();
        }
    }
}