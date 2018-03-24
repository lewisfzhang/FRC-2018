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
    private boolean mConnectedToNetworkTables = false;

    /**
     * @return the current angle of the scale, in degrees, as seen
     *         from the driver station (positive = right side raised;
     *         negative = left side raised)
     */
    public double getAngle() {
        return angle;
    }
    
    /**
     * Enum that represents the current height of the scale, based on data sent by the CheesyVision2 python app over Network Tables
     */
    public enum ScaleHeight {
        HIGH(SuperstructureConstants.kScaleHighHeight),
        NEUTRAL(SuperstructureConstants.kScaleNeutralHeight),
        LOW(SuperstructureConstants.kScaleLowHeight),
        UNKNOWN(SuperstructureConstants.kScaleHighHeight); // Worst case: go to highest preset

        private double mHeight;

        ScaleHeight(double height) {
            this.mHeight = height;
        }

        public double getHeight() {
            return this.mHeight;
        }
    }

    /**
     * @return the current ScaleHeight of the <em>right</em> scale
     *         plate, as seen from the driver station
     */
    public ScaleHeight getRightScaleHeight() {
        if ((int) tip > 2) {
            mConnectedToNetworkTables = false;
            return ScaleHeight.UNKNOWN;
        }

        mConnectedToNetworkTables = true;

        if (tip > 0.9) {
            return ScaleHeight.HIGH;
        } else if (tip < -0.9) {
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

        switch(right) {
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
        SmartDashboard.putBoolean("Connected to Cheesy Vision 2", mConnectedToNetworkTables);
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
                angle = SmartDashboard.getNumber("scaleAngle", 1000); // Set default value out of [-1, 1] range
                tip = SmartDashboard.getNumber("scaleTip", 1000); // Set default value out of [-1, 1] range
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        };

        looper.register(loop);
    }
}