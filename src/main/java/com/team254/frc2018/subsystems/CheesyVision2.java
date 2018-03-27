package com.team254.frc2018.subsystems;

import com.team254.frc2018.AutoFieldState;
import com.team254.frc2018.Constants;
import com.team254.frc2018.loops.ILooper;
import com.team254.frc2018.loops.Loop;
import com.team254.frc2018.states.SuperstructureConstants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Handles things related to scale angle detection.
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
    private boolean error = true;
    private double lastHeartbeatValue = -1, lastHeartbeatTime = Double.NEGATIVE_INFINITY;
    
    /**
     * @return true if the robot is receiving data from the scale tracker
     */
    public boolean isConnected() {
        return Timer.getFPGATimestamp() < lastHeartbeatTime + Constants.kScaleTrackerTimeout;
    }
    
    /**
     * @return true if the system doesn't have a good reading of the scale
     */
    public boolean getError() {
        return error;
    }
    
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
    
    
    public static final double[] DEFAULT_HEIGHT = new double[] {SuperstructureConstants.kScaleNeutralHeight, SuperstructureConstants.kScaleNeutralHeightBackwards};
    public static final double[] LOW_HEIGHT     = new double[] {SuperstructureConstants.kScaleLowHeight, SuperstructureConstants.kScaleLowHeightBackwards};
    public static final double[] NEUTRAL_HEIGHT = new double[] {SuperstructureConstants.kScaleNeutralHeight, SuperstructureConstants.kScaleNeutralHeightBackwards};
    public static final double[] HIGH_HEIGHT    = new double[] {SuperstructureConstants.kScaleHighHeight, SuperstructureConstants.kScaleHighHeightBackwards};
    
    public double getDesiredHeight(boolean backwards) {
        int i = backwards? 1 : 0;
        if (getError()) return DEFAULT_HEIGHT[i];
        
        double tip = getTip();
        AutoFieldState state = new AutoFieldState();
        state.setSides();
        if (state.getScaleSide() == AutoFieldState.Side.LEFT)
            tip = -tip;
        
        double[] height = NEUTRAL_HEIGHT;
        if (tip < 0) height = LOW_HEIGHT;
        if (tip > 0) height = HIGH_HEIGHT;
        return height[i];
    }
    
    public double getDesiredHeight(boolean backwards, int cubeNum) {
        return getDesiredHeight(backwards) + 10.0*cubeNum;
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
            error = SmartDashboard.getBoolean("scaleError", true);
            double heartbeat = SmartDashboard.getNumber("scaleHeartbeat", -2);
            if (heartbeat > lastHeartbeatValue) {
                lastHeartbeatValue = heartbeat;
                lastHeartbeatTime = timestamp;
            }
            
            SmartDashboard.putNumber("scaleAngleEcho", angle);
            SmartDashboard.putNumber("scaleTipEcho", tip);
        }
        
        @Override
        public void onStop(double timestamp) {
            stop();
        }
    }
}