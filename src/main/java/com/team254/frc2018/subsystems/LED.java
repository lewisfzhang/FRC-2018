package com.team254.frc2018.subsystems;

import com.team254.frc2018.loops.ILooper;
import com.team254.frc2018.loops.Loop;
import com.team254.frc2018.states.LEDState;

public class LED extends Subsystem {
    private static final double kHangingBlinkDuration = 0.5; // In sec
    private static final double kFaultBlinkDuration = 0.25; // In sec

    private static LED mInstance;

    private CarriageCanifier mCarriageCanifier;
    private SystemState mSystemState = SystemState.DISPLAYING_INTAKE;
    private WantedAction mWantedAction = WantedAction.DISPLAY_INTAKE;

    private Wrist mWrist;
    private Elevator mElevator;

    private boolean mFaultsEnabled = false;

    private LEDState mDesiredLEDState = new LEDState(0.0, 0.0, 0.0);
    private LEDState mIntakeLEDState = new LEDState(0.0, 0.0, 0.0);

    public synchronized static LED getInstance() {
        if (mInstance == null) {
            mInstance = new LED();
        }
        return mInstance;
    }

    private LED() {
       mCarriageCanifier = CarriageCanifier.getInstance();
    }

    public synchronized void setIntakeLEDState(LEDState intakeLEDState) {
        mIntakeLEDState.copyFrom(intakeLEDState);
    }

    public synchronized void setWantedAction(WantedAction wantedAction) {
        mWantedAction = wantedAction;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            double stateStartTime;
            @Override
            public void onStart(double timestamp) {
                stateStartTime = timestamp;
                mWrist = Wrist.getInstance();
                mElevator = Elevator.getInstance();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (LED.this) {
                    SystemState newState = getStateTransition();

                    if (mSystemState != newState) {
                        System.out.println(timestamp + ": LED changed state: " + mSystemState + " -> " + newState);
                        mSystemState = newState;
                        stateStartTime = timestamp;
                    }

                    double timeInState = timestamp - stateStartTime;

                    switch (mSystemState) {
                        case DISPLAYING_INTAKE:
                            setIntakeLEDCommand();
                            break;
                        case DISPLAYING_FAULT:
                            setFaultLEDCommand(timeInState);
                            break;
                        case DISPLAYING_HANG:
                            setHangLEDCommand(timeInState);
                            break;
                        default:
                            System.out.println("Fell through on LED commands: " + mSystemState);
                            break;
                    }
                    mCarriageCanifier.setLEDColor(mDesiredLEDState.red, mDesiredLEDState.green,
                            mDesiredLEDState.blue);
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    private void setIntakeLEDCommand() {
        mDesiredLEDState.copyFrom(mIntakeLEDState);
    }
    private void setFaultLEDCommand(double timeInState) {
         // Blink red.
        if ((int)(timeInState / kFaultBlinkDuration) % 2 == 0) {
            mDesiredLEDState.copyFrom(LEDState.kFault);
        } else {
            mDesiredLEDState.copyFrom(LEDState.kOff);
        }
    }
    private void setHangLEDCommand(double timeInState) {
        // Blink orange.
        if ((int)(timeInState / kHangingBlinkDuration) % 2 == 0) {
            mDesiredLEDState.copyFrom(LEDState.kHanging);
        } else {
            mDesiredLEDState.copyFrom(LEDState.kOff);
        }
    }

    private SystemState getStateTransition() {
        if (mFaultsEnabled && (!mWrist.hasBeenZeroed() || !mElevator.hasBeenZeroed())) {
            return SystemState.DISPLAYING_FAULT;
        }
        switch (mWantedAction) {
            case DISPLAY_HANG:
                return SystemState.DISPLAYING_HANG;
            case DISPLAY_INTAKE:
                return SystemState.DISPLAYING_INTAKE;
            default:
                System.out.println("Fell through on LED wanted action check: " + mWantedAction);
                return SystemState.DISPLAYING_INTAKE;
        }
    }

    public synchronized void setEnableFaults(boolean enable) {
        mFaultsEnabled = enable;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}

    public enum WantedAction {
        DISPLAY_HANG,
        DISPLAY_INTAKE,
    }

    private enum SystemState {
        DISPLAYING_FAULT,
        DISPLAYING_INTAKE,
        DISPLAYING_HANG,
    }
}
