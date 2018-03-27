package com.team254.frc2018.subsystems;

import com.team254.frc2018.AutoFieldState;
import com.team254.frc2018.Constants;
import com.team254.frc2018.AutoFieldState.Side;
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
    
    /**
     * Enum that represents the current height of the scale, based on data sent by the CheesyVision2 python app over Network Tables
     */
    public static enum ScaleHeight {
        HIGH(SuperstructureConstants.kScaleHighHeight, SuperstructureConstants.kScaleHighHeightBackwards),
        NEUTRAL(SuperstructureConstants.kScaleNeutralHeight, SuperstructureConstants.kScaleNeutralHeightBackwards),
        LOW(SuperstructureConstants.kScaleLowHeight, SuperstructureConstants.kScaleLowHeightBackwards),
        UNKNOWN(SuperstructureConstants.kScaleHighHeight, SuperstructureConstants.kScaleHighHeightBackwards); // Worst case: go to highest preset

        private final double mForwardsHeight, mBackwardsHeight;

        ScaleHeight(double forwardsHeight, double backwardsHeight) {
            this.mForwardsHeight = forwardsHeight;
            this.mBackwardsHeight = backwardsHeight;
        }
    
        public double getHeight(boolean backwards) {
            return backwards ? mBackwardsHeight : mForwardsHeight;
        }
    }

    public double getDesiredHeight(AutoFieldState fieldState, boolean backwards, int cubeNum) {
        ScaleHeight state = fieldState.getScaleSide() == Side.RIGHT ? getRightScaleHeight() : getLeftScaleHeight();
        return state.getHeight(backwards) + 10.0*cubeNum;
    }
    
    /**
     * @return the current ScaleHeight of the <em>right</em> scale
     *         plate, as seen from the driver station
     */
    public ScaleHeight getRightScaleHeight() {    
        if (getError() || !isConnected()) {
            return ScaleHeight.UNKNOWN;
        } else if (getTip() > 0.5) {
            return ScaleHeight.HIGH;
        } else if (getTip() < -0.5) {
            return ScaleHeight.LOW;
        } else {
            return ScaleHeight.NEUTRAL;
        }
    }
    /**
     * @return the current ScaleHeight of the <em>left</em> scale
     *         plate, as seen from the driver station
     */
    public ScaleHeight getLeftScaleHeight() {
        ScaleHeight right = getRightScaleHeight();
        switch (right) {
            case HIGH:
                return ScaleHeight.LOW;
            case NEUTRAL:
                return ScaleHeight.NEUTRAL;
            case LOW:
                return ScaleHeight.HIGH;
            case UNKNOWN:
            default:
                return ScaleHeight.UNKNOWN;
        }
    }
    
    @Override
    public boolean checkSystem() {
        return false;
    }
    
    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Connected to CheesyVision2", isConnected());
        SmartDashboard.putString("Right Scale", getRightScaleHeight().toString());
        SmartDashboard.putString("Left Scale", getLeftScaleHeight().toString());
    }
    
    @Override
    public void stop() {}
    
    @Override
    public void registerEnabledLoops(ILooper looper) {
        Loop loop = new Loop() {
            @Override
            public void onStart(double timestamp) {}
            
            @Override
            public void onLoop(double timestamp) {
                angle = SmartDashboard.getNumber("scaleAngle", Double.NaN);
                tip = SmartDashboard.getNumber("scaleTip", Double.NaN);
                error = SmartDashboard.getBoolean("scaleError", true);
                double heartbeat = SmartDashboard.getNumber("scaleHeartbeat", -2);
                if (heartbeat > lastHeartbeatValue) {
                    lastHeartbeatValue = heartbeat;
                    lastHeartbeatTime = timestamp;
                }
            }
            
            @Override
            public void onStop(double timestamp) {
                stop();
            }
        };

        looper.register(loop);
    }
}