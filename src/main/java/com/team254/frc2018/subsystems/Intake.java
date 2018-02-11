package com.team254.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2018.Constants;
import com.team254.frc2018.loops.Loop;
import com.team254.frc2018.loops.Looper;
import com.team254.lib.drivers.TalonSRXFactory;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class Intake extends Subsystem {
    private final static boolean kClosed = true;
    private final static boolean kClamped = true;
    private final static double kActuationTime = 0.25;
    private final static double kShootSetpoint = 1.0;
    private final static double kIntakeSetpoint = -1.0;
    private final static double kHoldSetpoint = -0.05;
    private final static double kShootTime = 0.25;

    private static Intake mInstance;

    public static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    public enum WantedState {
        IDLE,
        INTAKE,
        SHOOT,
        PLACE
    }

    private enum SystemState {
        IDLE,
        INTAKING,
        CLAMPING,
        HOLDING,
        SHOOTING,
        PLACING
    }

    public enum JawState {
        OPEN,
        CLOSED,
        CLAMPED
    }

    private final Solenoid mCloseSolenoid, mClampSolenoid;//open->false, false; close->true, false; clamp->true, true;
    private final TalonSRX mLeftMaster, mRightMaster;
    private final DigitalInput mLeftBanner, mRightBanner;

    private WantedState mWantedState;
    private SystemState mSystemState;
    private JawState mJawState;

    private Intake() {
        mCloseSolenoid = Constants.makeSolenoidForId(Constants.kIntakeCloseSolenoid);
        mClampSolenoid = Constants.makeSolenoidForId(Constants.kIntakeClampSolenoid);

        mLeftMaster = TalonSRXFactory.createDefaultTalon(Constants.kIntakeLeftMasterId);
        mLeftMaster.set(ControlMode.PercentOutput, 0);
        mLeftMaster.setInverted(false);

        mRightMaster = TalonSRXFactory.createDefaultTalon(Constants.kIntakeRightMasterId);
        mRightMaster.set(ControlMode.PercentOutput, 0);
        mRightMaster.setInverted(true);

        mLeftBanner = new DigitalInput(Constants.kIntakeLeftBannerId);
        mRightBanner = new DigitalInput(Constants.kIntakeRightBannerId);
    }

    @Override
    public void outputToSmartDashboard() {
    }

    @Override
    public void stop() {
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        Loop loop = new Loop() {
            private double mCurrentStateStartTime;

            @Override
            public void onStart(double timestamp) {
                synchronized (Intake.this) {
                    mSystemState = SystemState.IDLE;
                    mWantedState = WantedState.IDLE;
                    mJawState = JawState.CLOSED;
                }
                mCurrentStateStartTime = Timer.getFPGATimestamp();
            }

            @Override
            public void onLoop(double timestamp) {

                synchronized (Intake.this) {
                    SystemState newState;
                    double timeInState = Timer.getFPGATimestamp() - mCurrentStateStartTime;
                    switch (mSystemState) {
                        case IDLE:
                            newState = handleIdle();
                            break;
                        case INTAKING:
                            newState = handleIntaking();
                            break;
                        case CLAMPING:
                            newState = handleClamping(timeInState);
                            break;
                        case HOLDING:
                            newState = handleHolding();
                            break;
                        case SHOOTING:
                            newState = handleShooting(timeInState);
                            break;
                        case PLACING:
                            newState = handlePlacing(timeInState);
                            break;
                        default:
                            System.out.println("Unexpected intake system state: " + mSystemState);
                            newState = mSystemState;
                            break;
                    }

                    if (newState != mSystemState) {
                        System.out.println(timestamp + ": Changed state: " + mSystemState + " -> " + newState);
                        mSystemState = newState;
                        mCurrentStateStartTime = Timer.getFPGATimestamp();
                    }
                }

            }

            @Override
            public void onStop(double timestamp) {
                mWantedState = WantedState.IDLE;
                mSystemState = SystemState.IDLE;
                // Set the states to what the robot falls into when disabled.
                stop();
            }
        };
        enabledLooper.register(loop);
    }

    private synchronized SystemState handleIdle() {
        setPower(0);
        setJaw(JawState.CLOSED);

        switch (mWantedState) {
            case INTAKE:
                return SystemState.INTAKING;
            default:
                return SystemState.IDLE;
        }
    }

    private synchronized SystemState handleIntaking() {
        setPower(kIntakeSetpoint);
        setJaw(JawState.CLOSED);

        if(seesCube()) {
            return SystemState.CLAMPING;
        }

        switch (mWantedState) {
            case IDLE:
                return SystemState.IDLE;
            default:
                return SystemState.INTAKING;
        }
    }

    private synchronized SystemState handleClamping(double timeInState) {
        setPower(kIntakeSetpoint);
        setJaw(JawState.CLAMPED);

        if(seesCube()) {
            if(timeInState > kActuationTime) {
                return SystemState.HOLDING;
            } else {
                return SystemState.CLAMPING;
            }
        } else {
            return SystemState.IDLE;
        }
    }

    private synchronized SystemState handleHolding() {
        setPower(kHoldSetpoint);
        setJaw(JawState.CLAMPED);

        if(!seesCube()) {
            return SystemState.IDLE;
        }

        switch (mWantedState) {
            case SHOOT:
                return SystemState.SHOOTING;
            case PLACE:
                return SystemState.PLACING;
            default:
                return SystemState.HOLDING;
        }
    }

    private synchronized SystemState handleShooting(double timeInState) {
        setPower(kShootSetpoint);
        setJaw(JawState.CLAMPED);

        if(timeInState > kShootTime) {
            return SystemState.IDLE;
        } else {
            return SystemState.SHOOTING;
        }
    }

    private synchronized SystemState handlePlacing(double timeInState) {
        setPower(0);
        setJaw(JawState.OPEN);

        if(timeInState > kActuationTime) {
            return SystemState.IDLE;
        } else {
            return SystemState.PLACING;
        }
    }

    private void setPower(double output) {
        mLeftMaster.set(ControlMode.PercentOutput, output);
        mRightMaster.set(ControlMode.PercentOutput, output);
    }

    private void setJaw(JawState state) {
        mJawState = state;
        switch (mJawState) {
            case OPEN:
                mCloseSolenoid.set(!kClosed);
                mClampSolenoid.set(!kClamped);
                break;
            case CLOSED:
                mCloseSolenoid.set(kClosed);
                mClampSolenoid.set(!kClamped);
                break;
            case CLAMPED:
                mCloseSolenoid.set(kClosed);
                mClampSolenoid.set(kClamped);
                break;
        }
    }

    private boolean seesCube() {
        return mLeftBanner.get() && mRightBanner.get();
    }

    public boolean hasCube() {
        return mSystemState == SystemState.HOLDING;
    }

    public JawState getJawState() {
        return mJawState;
    }

    public void setState(WantedState wantedState) {
        mWantedState = wantedState;
    }

    @Override
    public boolean checkSystem() {
        return true;
    }
}

