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

    public synchronized double getDesiredHeight(boolean backwards, int cubeNum) {
        double correctedTip = tip * (AutoFieldState.getInstance().getScaleSide() == Side.RIGHT ? 1.0 : -1.0);

        double baseHeight;

        if(correctedTip > 0.5) {
            baseHeight = backwards ? SuperstructureConstants.kScaleHighHeightBackwards : SuperstructureConstants.kScaleHighHeight;
        } else if(correctedTip < -0.5) {
            baseHeight = backwards ? SuperstructureConstants.kScaleLowHeightBackwards : SuperstructureConstants.kScaleLowHeight;
        } else {
            baseHeight = backwards ? SuperstructureConstants.kScaleNeutralHeightBackwards : SuperstructureConstants.kScaleNeutralHeight;
        }

        //assume we are losing the scale if we don't have a reading
        if(error || !isConnected()) {
            baseHeight = backwards ? SuperstructureConstants.kScaleHighHeightBackwards : SuperstructureConstants.kScaleHighHeight;
        }

        baseHeight += cubeNum * SuperstructureConstants.kCubeOffset;

        return baseHeight;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }
    
    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Connected to CheesyVision2", isConnected());
        SmartDashboard.putNumber("Desired Height (0 cubes)", getDesiredHeight(false, 0));
        SmartDashboard.putNumber("Desired Height (1 cube)", getDesiredHeight(false, 1));
        SmartDashboard.putNumber("Desired Height (2 cubes)", getDesiredHeight(false, 2));
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