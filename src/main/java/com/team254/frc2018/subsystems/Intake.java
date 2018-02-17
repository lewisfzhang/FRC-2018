package com.team254.frc2018.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2018.Constants;
import com.team254.frc2018.loops.Loop;
import com.team254.frc2018.loops.Looper;
import com.team254.frc2018.statemachines.IntakeStateMachine;
import com.team254.frc2018.states.IntakeState;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.lib.drivers.TalonSRXFactory;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class Intake extends Subsystem {
    private final static boolean kClosed = false;
    private final static boolean kClamped = false;

    private static Intake mInstance;

    public synchronized static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    private final Solenoid mCloseSolenoid, mClampSolenoid; //open->false, false; close->true, false; clamp->true, true;
    private final TalonSRX mLeftMaster, mRightMaster;
    private final DigitalInput mLeftBanner, mRightBanner;

    public static CANifier canifier = new CANifier(0);

    private IntakeStateMachine.WantedAction mWantedAction;
    private IntakeState.JawState mJawState;

    private IntakeState mCurrentState = new IntakeState();
    private IntakeStateMachine mStateMachine = new IntakeStateMachine();

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

    public IntakeState getCurrentState() {
        mCurrentState.leftCubeSensorTriggered = false; // TODO: replace with banner sensor
        mCurrentState.rightCubeSensorTriggered = false; // TODO: replace with banner sensor
        mCurrentState.wristAngle = Wrist.getInstance().getAngle(); // this is a hack
        return mCurrentState;
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        Loop loop = new Loop() {


            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {

                synchronized (Intake.this) {
                    IntakeState newState = mStateMachine.update(Timer.getFPGATimestamp(), mWantedAction, getCurrentState());
                    updateActuatorFromState(newState);
                }

            }

            @Override
            public void onStop(double timestamp) {
                mWantedAction = IntakeStateMachine.WantedAction.IDLE;
                // Set the states to what the robot falls into when disabled.
                stop();
            }
        };
        enabledLooper.register(loop);
    }

    public void setPower(double output) {
        mLeftMaster.set(ControlMode.PercentOutput, output);
        mRightMaster.set(ControlMode.PercentOutput, output);
    }

    public void setJaw(IntakeState.JawState state) {
        if (mJawState == state) {
            return;
        }
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

    private synchronized void updateActuatorFromState(IntakeState state) {
        mLeftMaster.set(ControlMode.PercentOutput, state.leftMotor);
        mRightMaster.set(ControlMode.PercentOutput, state.rightMotor);
        setJaw(state.jawState);
    }

    public boolean getLeftBannerSensor() {
        return canifier.getGeneralInput(CANifier.GeneralPin.LIMF);
    }

    public boolean getRightBannerSensor() {
        return canifier.getGeneralInput(CANifier.GeneralPin.LIMF);
    }

    public boolean hasCube() {
        return mStateMachine.hasCubeClamped();
    }

    public IntakeState.JawState getJawState() {
        return mJawState;
    }

    public void setState(IntakeStateMachine.WantedAction mWantedAction) {
        mWantedAction = mWantedAction;
    }

    @Override
    public boolean checkSystem() {
        return true;
    }
}

