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
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.drivers.TalonSRXChecker;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

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

    private static final int kLowGearVelocityControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;

    public static Drive getInstance() {
        return mInstance;
    }

    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
    }

    // Control states
    private DriveControlState mDriveControlState;

    // Hardware
    private final TalonSRX mLeftMaster, mRightMaster, mLeftSlaveA, mRightSlaveA, mLeftSlaveB, mRightSlaveB;
    private final Solenoid mShifter;
    private PigeonIMU mPigeon;

    // Hardware states
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
                        return;
                    case PATH_FOLLOWING:
                        updatePathFollower();
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
        mLeftMaster = TalonSRXFactory.createDefaultTalon(Constants.kLeftDriveMasterId);
        ErrorCode leftSensorPresent = mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice
                .CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
        if (leftSensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect left encoder: " + leftSensorPresent, false);
        }
        mLeftMaster.setInverted(false);
        mLeftMaster.setSensorPhase(true);

        mLeftSlaveA = TalonSRXFactory.createPermanentSlaveTalon(Constants.kLeftDriveSlaveAId,
                Constants.kLeftDriveMasterId);
        mLeftSlaveA.setInverted(false);

        mLeftSlaveB = TalonSRXFactory.createPermanentSlaveTalon(Constants.kLeftDriveSlaveBId,
                Constants.kLeftDriveMasterId);
        mLeftSlaveB.setInverted(false);

        mRightMaster = TalonSRXFactory.createDefaultTalon(Constants.kRightDriveMasterId);
        ErrorCode rightSensorPresent = mRightMaster.configSelectedFeedbackSensor(FeedbackDevice
                .CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
        if (rightSensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect left encoder: " + rightSensorPresent, false);
        }
        mRightMaster.setInverted(true);
        mRightMaster.setSensorPhase(true);

        mRightSlaveA = TalonSRXFactory.createPermanentSlaveTalon(Constants.kRightDriveSlaveAId,
                Constants.kRightDriveMasterId);
        mRightSlaveA.setInverted(true);

        mRightSlaveB = TalonSRXFactory.createPermanentSlaveTalon(Constants.kRightDriveSlaveBId,
                Constants.kRightDriveMasterId);
        mRightSlaveB.setInverted(true);

        mShifter = Constants.makeSolenoidForId(Constants.kShifterSolenoidId);

        reloadGains();

        mPigeon = new PigeonIMU(mLeftSlaveB);

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
            setBrakeMode(false);

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
            mLeftMaster.enableVoltageCompensation(true);
            mLeftMaster.configVoltageCompSaturation(12.0, 0);
            mLeftMaster.selectProfileSlot(kHighGearVelocityControlSlot, 0);

            mRightMaster.enableVoltageCompensation(true);
            mRightMaster.configVoltageCompSaturation(12.0, 0);
            mRightMaster.selectProfileSlot(kHighGearVelocityControlSlot, 0);
            setBrakeMode(true);

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
            mShifter.set(!wantsHighGear);
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(mPigeon.getFusedHeading());
    }

    public void setHeading(Rotation2d heading) {
        mPigeon.setFusedHeading(0, 0);
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
        SmartDashboard.putNumber("Left speed: ", mLeftMaster.getSelectedSensorVelocity(0));
        SmartDashboard.putNumber("Right speed: ", mRightMaster.getSelectedSensorVelocity(0));
        SmartDashboard.putNumber("Heading: ", getHeading().getDegrees());
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

    private void updatePathFollower() {
        DriveSignal signal = new DriveSignal(0, 0); //get this from path follower
        setVelocity(signal);
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
