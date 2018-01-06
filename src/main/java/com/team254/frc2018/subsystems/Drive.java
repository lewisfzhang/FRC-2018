package com.team254.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;

import com.team254.frc2018.Constants;
import com.team254.frc2018.loops.Loop;
import com.team254.frc2018.loops.Looper;
import com.team254.lib.util.DriveSignal;

/**
 * This subsystem consists of the robot's drivetrain: 4 CIM motors, 4 talons, one solenoid and 2 pistons to shift gears,
 * and a navX board. The Drive subsystem has several control methods including open loop, velocity control, and position
 * control. The Drive subsystem also has several methods that handle automatic aiming, autonomous path driving, and
 * manual control.
 * 
 * @see Subsystem
 */
public class Drive extends Subsystem {

    private static Drive mInstance = new Drive();

    private static final int kLowGearPositionControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;

    public static Drive getInstance() {
        return mInstance;
    }

    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        VELOCITY_SETPOINT, // velocity PID control
    }

    /**
     * Check if the drive talons are configured for velocity control
     */
    protected static boolean usesTalonVelocityControl(DriveControlState state) {
        if (state == DriveControlState.VELOCITY_SETPOINT) {
            return true;
        }
        return false;
    }

    // Control states
    private DriveControlState mDriveControlState;

    // Hardware
    private final TalonSRX mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;
    private final Solenoid mShifter;

    // Hardware states
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
                setOpenLoop(DriveSignal.NEUTRAL);
                setBrakeMode(false);
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                switch (mDriveControlState) {
                case OPEN_LOOP:
                    return;
                case VELOCITY_SETPOINT:
                    return;
                default:
                    System.out.println("Unexpected drive control state: " + mDriveControlState);
                    break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    private Drive() {
        // Start all Talons in open loop mode.
        mLeftMaster = new TalonSRX(Constants.kLeftDriveSlaveId);
        mLeftMaster.set(ControlMode.PercentOutput, 0);
        mLeftMaster.setInverted(false);
        ErrorCode leftSensorPresent = mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
        if (leftSensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect left encoder: " + leftSensorPresent, false);
        }
        mLeftMaster.setSensorPhase(true);



        mLeftSlave = new TalonSRX(Constants.kLeftDriveSlaveId);
        mLeftSlave.set(ControlMode.Follower, Constants.kLeftDriveMasterId);
        mLeftSlave.setInverted(false);
        ErrorCode leftTalonFramePeriod = mLeftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        if (leftTalonFramePeriod != ErrorCode.OK) {
            DriverStation.reportError("Could not set left frame period: " + leftTalonFramePeriod, false);
        }


        mRightMaster = new TalonSRX(Constants.kRightDriveMasterId);
        mRightMaster.set(ControlMode.PercentOutput, 0);
        mRightMaster.setInverted(false);
        ErrorCode rightSensorPresent = mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
        if (leftSensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect left encoder: " + leftSensorPresent, false);
        }
        mRightMaster.setSensorPhase(true);



        mRightSlave = new TalonSRX(Constants.kRightDriverSlaveId);
        mRightSlave.set(ControlMode.Follower, Constants.kRightDriveMasterId);
        mRightSlave.setInverted(false);
        ErrorCode rightTalonFramePeriod = mRightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        if (leftTalonFramePeriod != ErrorCode.OK) {
            DriverStation.reportError("Could not set left frame period: " + leftTalonFramePeriod, false);
        }

        mShifter = Constants.makeSolenoidForId(Constants.kShifterSolenoidId);

        reloadGains();

        mIsHighGear = false;
        setHighGear(true);
        setOpenLoop(DriveSignal.NEUTRAL);

        // Force a CAN message across.
        mIsBrakeMode = true;
        setBrakeMode(false);
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            mLeftMaster.configNominalOutputForward(0.0, 0);
            mLeftMaster.configNominalOutputReverse(0.0, 0);
            mRightMaster.configNominalOutputForward(0.0, 0);
            mRightMaster.configNominalOutputReverse(0.0, 0);
            mDriveControlState = DriveControlState.OPEN_LOOP;
            setBrakeMode(false);
        }
        // Right side is reversed, but reverseOutput doesn't invert PercentVBus.
        // So set negative on the right master.
        mLeftMaster.set (ControlMode.PercentOutput, -signal.getRight());
        mRightMaster.set(ControlMode.PercentOutput, signal.getLeft());
    }

    public boolean isHighGear() {
        return mIsHighGear;
    }

    public synchronized void setHighGear(boolean wantsHighGear) {
        if (wantsHighGear != mIsHighGear) {
            mIsHighGear = wantsHighGear;
            mShifter.set(!wantsHighGear);
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            mRightMaster.setNeutralMode(mode);
            mRightSlave.setNeutralMode(mode);
            mLeftMaster.setNeutralMode(mode);
            mLeftSlave.setNeutralMode(mode);
        }
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputToSmartDashboard() {
    }

    public synchronized void resetEncoders() {
        mLeftMaster.getSensorCollection().setQuadraturePosition(0, 0);
        mLeftMaster.setSelectedSensorPosition(0, 0, 0);
        mRightMaster.getSensorCollection().setQuadraturePosition(0, 0);
        mRightMaster.setSelectedSensorPosition(0, 0, 0);
        mLeftSlave.setSelectedSensorPosition(0, 0, 0);
        mRightSlave.setSelectedSensorPosition(0, 0, 0);
    }

    @Override
    public void zeroSensors() {
        resetEncoders();
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    public double getLeftDistanceInches() {
        return rotationsToInches(mLeftMaster.getSelectedSensorPosition(0));
    }

    public double getRightDistanceInches() {
        return rotationsToInches(mRightMaster.getSelectedSensorPosition(0));
    }

    public double getLeftVelocityInchesPerSec() {
        return rpmToInchesPerSecond(mLeftMaster.getSelectedSensorVelocity(0));
    }

    public double getRightVelocityInchesPerSec() {
        return rpmToInchesPerSecond(mRightMaster.getSelectedSensorVelocity(0));
    }

    public synchronized void reloadGains() {

    }

    @Override
    public void writeToLog() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }
}
