package com.team254.frc2018.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team254.frc2018.Constants;
import com.team254.frc2018.loops.Loop;
import com.team254.frc2018.loops.Looper;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.drivers.TalonSRXUtil;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.drivers.TalonSRXChecker;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Drive extends Subsystem {

    private static Drive mInstance = new Drive();

    private static final int kLowGearVelocityControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;

    private static final double DRIVE_ENCODER_PPR = 4096.;

    public static Drive getInstance() {
        return mInstance;
    }

    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
    }

    public enum ShifterState {
        FORCE_LOW_GEAR,
        FORCE_HIGH_GEAR,
        AUTO_SHIFT
    }

    // Control states
    private DriveControlState mDriveControlState;

    // Hardware
    private final TalonSRX mLeftMaster, mRightMaster, mLeftSlaveA, mRightSlaveA, mLeftSlaveB, mRightSlaveB;
    private final Solenoid mShifter;
    private PigeonIMU mPigeon;

    // Hardware states
    private boolean mAutoShift;
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
                zeroSensors();
                setOpenLoop(DriveSignal.NEUTRAL);
                setBrakeMode(false);
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                switch (mDriveControlState) {
                    case OPEN_LOOP:
                        break;
                    case PATH_FOLLOWING:
                        updatePathFollower();
                        break;
                    default:
                        System.out.println("Unexpected drive control state: " + mDriveControlState);
                        break;
                }
                if (mAutoShift && false) { // TODO: fix this (tom)
                    handleAutoShift();
                } else {
                    setHighGear(false);
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
        mLeftMaster = TalonSRXFactory.createDefaultTalon(Constants.kLeftDriveMasterId);
        final ErrorCode leftSensorPresent = mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice
                .CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
        if (leftSensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect left encoder: " + leftSensorPresent, false);
        }
        mLeftMaster.setInverted(false);
        mLeftMaster.setSensorPhase(true);
        mLeftMaster.enableVoltageCompensation(true);
        mLeftMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);

        mLeftSlaveA = TalonSRXFactory.createPermanentSlaveTalon(Constants.kLeftDriveSlaveAId,
                Constants.kLeftDriveMasterId);
        mLeftSlaveA.setInverted(false);

        mLeftSlaveB = TalonSRXFactory.createPermanentSlaveTalon(Constants.kLeftDriveSlaveBId,
                Constants.kLeftDriveMasterId);
        mLeftSlaveB.setInverted(false);

        mRightMaster = TalonSRXFactory.createDefaultTalon(Constants.kRightDriveMasterId);
        final ErrorCode rightSensorPresent = mRightMaster.configSelectedFeedbackSensor(FeedbackDevice
                .CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
        if (rightSensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect left encoder: " + rightSensorPresent, false);
        }
        mRightMaster.setInverted(true);
        mRightMaster.setSensorPhase(true);
        mRightMaster.enableVoltageCompensation(true);
        mRightMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);

        mRightSlaveA = TalonSRXFactory.createPermanentSlaveTalon(Constants.kRightDriveSlaveAId,
                Constants.kRightDriveMasterId);
        mRightSlaveA.setInverted(true);

        mRightSlaveB = TalonSRXFactory.createPermanentSlaveTalon(Constants.kRightDriveSlaveBId,
                Constants.kRightDriveMasterId);
        mRightSlaveB.setInverted(true);

        mShifter = Constants.makeSolenoidForId(Constants.kShifterSolenoidId);

        reloadGains();

        mPigeon = new PigeonIMU(mLeftSlaveB);

        // Force a solenoid message.
        mIsHighGear = true;
        setHighGear(false);

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
            setBrakeMode(false);
            mAutoShift = true;

            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
        mLeftMaster.set(ControlMode.PercentOutput, signal.getRight());
        mRightMaster.set(ControlMode.PercentOutput, signal.getLeft());
    }

    /**
     * Configures talons for velocity control
     */
    private synchronized void setVelocity(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            // We entered a velocity control state.
            setBrakeMode(true);
            mAutoShift = false;
            mLeftMaster.selectProfileSlot(kHighGearVelocityControlSlot, 0);
            mRightMaster.selectProfileSlot(kHighGearVelocityControlSlot, 0);

            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }

        mLeftMaster.set(ControlMode.Velocity, signal.getLeft());
        mRightMaster.set(ControlMode.Velocity, signal.getRight());
    }

    public boolean isHighGear() {
        return mIsHighGear;
    }

    public synchronized void setHighGear(boolean wantsHighGear) {
        if (wantsHighGear != mIsHighGear) {
            mIsHighGear = wantsHighGear;
            mShifter.set(wantsHighGear);
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(mPigeon.getFusedHeading());
    }

    public void setHeading(Rotation2d heading) {
        mPigeon.setFusedHeading(heading.getDegrees(), 10);
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            mRightMaster.setNeutralMode(mode);
            mRightSlaveA.setNeutralMode(mode);
            mRightSlaveB.setNeutralMode(mode);

            mLeftMaster.setNeutralMode(mode);
            mLeftSlaveA.setNeutralMode(mode);
            mLeftSlaveB.setNeutralMode(mode);
        }
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Right Drive Distance", getRightEncoderDistance());
        SmartDashboard.putNumber("Left Drive Distance", getLeftEncoderDistance());
        SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());
        SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
    }

    public synchronized void resetEncoders() {
        mLeftMaster.getSensorCollection().setQuadraturePosition(0, 0);
        mLeftMaster.setSelectedSensorPosition(0, 0, 0);
        mRightMaster.getSensorCollection().setQuadraturePosition(0, 0);
        mRightMaster.setSelectedSensorPosition(0, 0, 0);
        mLeftSlaveA.setSelectedSensorPosition(0, 0, 0);
        mRightSlaveA.setSelectedSensorPosition(0, 0, 0);
    }

    @Override
    public void zeroSensors() {
        setHeading(new Rotation2d());
        resetEncoders();
        mAutoShift = true;
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

    public double getLeftEncoderRotations() {
        return mLeftMaster.getSelectedSensorPosition(0) / DRIVE_ENCODER_PPR;
    }

    public double getRightEncoderRotations() {
        return mRightMaster.getSelectedSensorPosition(0) / DRIVE_ENCODER_PPR;
    }

    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    public double getRightVelocityNativeUnits() {
        return mRightMaster.getSelectedSensorVelocity(0);
    }

    public double getLeftVelocityNativeUnits() {
        return mLeftMaster.getSelectedSensorVelocity(0) ;
    }
    public double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() *  10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
    }

    private void updatePathFollower() {
        DriveSignal signal = new DriveSignal(0, 0); // TODO get this from path follower
        setVelocity(signal);
    }

    private void handleAutoShift() {
        final double linear_velocity = Math.abs(getLinearVelocity());
        final double angular_velocity = Math.abs(getAngularVelocity());
        if (mIsHighGear && linear_velocity < Constants.kDriveDownShiftVelocity && angular_velocity < Constants.kDriveDownShiftAngularVelocity) {
            setHighGear(false);
        } else if (!mIsHighGear && linear_velocity > Constants.kDriveUpShiftVelocity) {
            setHighGear(true);
        }
    }

    public synchronized void reloadGains() {
        //todo: implement this
    }

    @Override
    public void writeToLog() {
    }

    @Override
    public boolean checkSystem() {
        boolean leftSide = TalonSRXChecker.CheckTalons(this,
                new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
                    {
                        add(new TalonSRXChecker.TalonSRXConfig("left_master", mLeftMaster));
                        add(new TalonSRXChecker.TalonSRXConfig("left_slave", mLeftSlaveA));
                        add(new TalonSRXChecker.TalonSRXConfig("left_slave1", mLeftSlaveB));
                    }
                }, new TalonSRXChecker.CheckerConfig() {
                    {
                        mCurrentFloor = 2;
                        mRPMFloor = 1500;
                        mCurrentEpsilon = 2.0;
                        mRPMEpsilon = 250;
                        mRPMSupplier = () -> mLeftMaster.getSelectedSensorVelocity(0);
                    }
                });
        boolean rightSide = TalonSRXChecker.CheckTalons(this,
                new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
                    {
                        add(new TalonSRXChecker.TalonSRXConfig("right_master", mRightMaster));
                        add(new TalonSRXChecker.TalonSRXConfig("right_slave", mRightSlaveA));
                        add(new TalonSRXChecker.TalonSRXConfig("right_slave1", mRightSlaveB));
                    }
                }, new TalonSRXChecker.CheckerConfig() {
                    {
                        mCurrentFloor = 2;
                        mRPMFloor = 1500;
                        mCurrentEpsilon = 2.0;
                        mRPMEpsilon = 250;
                        mRPMSupplier = () -> mRightMaster.getSelectedSensorVelocity(0);
                    }
                });
        return leftSide && rightSide;
    }
}
