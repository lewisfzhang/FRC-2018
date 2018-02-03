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
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.DriveSignal;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    private final TalonSRX mLeftMaster, mRightMaster, mLeftSlave, mRightSlave, mLeftSlave1, mRightSlave1;
    private final Solenoid mShifter;
    private PigeonIMU mPigeon;

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
        mLeftMaster = new TalonSRX(Constants.kLeftDriveMasterId);
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
//        ErrorCode leftTalonFramePeriod = mLeftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
//        if (leftTalonFramePeriod != ErrorCode.OK) {
//            DriverStation.reportError("Could not set left frame period: " + leftTalonFramePeriod, false);
//        }

        mLeftSlave1 = new TalonSRX(Constants.kLeftDriveSlave1Id);
        mLeftSlave1.set(ControlMode.Follower, Constants.kLeftDriveMasterId);
        mLeftSlave1.setInverted(false);
//        ErrorCode leftTalonFramePeriod1 = mLeftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
//        if (leftTalonFramePeriod1 != ErrorCode.OK) {
//            DriverStation.reportError("Could not set left frame period: " + leftTalonFramePeriod1, false);
//        }

        mRightMaster = new TalonSRX(Constants.kRightDriveMasterId);//new TalonSRX(Constants.kRightDriveMasterId);
        mRightMaster.set(ControlMode.PercentOutput, 0);
        mRightMaster.setInverted(true);
        ErrorCode rightSensorPresent = mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
        if (rightSensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect left encoder: " + rightSensorPresent, false);
        }
        mRightMaster.setSensorPhase(true);

        mRightSlave = new TalonSRX(Constants.kRightDriveSlaveId);
        mRightSlave.set(ControlMode.Follower, Constants.kRightDriveMasterId);
        mRightSlave.setInverted(true);
//        ErrorCode rightTalonFramePeriod = mRightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
//        if (rightTalonFramePeriod != ErrorCode.OK) {
//            DriverStation.reportError("Could not set left frame period: " + rightTalonFramePeriod, false);
//        }

        mRightSlave1 = new TalonSRX(Constants.kRightDriveSlave1Id);
        mRightSlave1.set(ControlMode.Follower, Constants.kRightDriveMasterId);
        mRightSlave1.setInverted(true);
//        ErrorCode rightTalonFramePeriod1 = mRightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
//        if (rightTalonFramePeriod1 != ErrorCode.OK) {
//            DriverStation.reportError("Could not set left frame period: " + rightTalonFramePeriod1, false);
//        }

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
        mLeftMaster.set(ControlMode.PercentOutput, signal.getRight());
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

    public Rotation2d getHeading() {
        double[] ypr = new double[3];
        mPigeon.getYawPitchRoll(ypr);
        return Rotation2d.fromDegrees(ypr[0]);
    }

    public void setHeading(Rotation2d heading) {
        mPigeon.setYaw(heading.getDegrees(), 0);
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            mRightMaster.setNeutralMode(mode);
            mRightSlave.setNeutralMode(mode);
            mRightSlave1.setNeutralMode(mode);

            mLeftMaster.setNeutralMode(mode);
            mLeftSlave.setNeutralMode(mode);
            mLeftSlave1.setNeutralMode(mode);
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

    /**
     * Follower wheel getters
     **/
    public double getLeftFollowerDistance() {
        return rotationsToInches(0); // todo: figure out where encoders are plugged in
    }

    public double getRightFollowerDistance() {
        return rotationsToInches(0); // todo: figure out where encoders are plugged in
    }

    public double getBackFollowerDistance() {
        return rotationsToInches(0); // todo: figure out where encoders are plugged in
    }

    public double getLeftFollowerVelocity() {
        return rpmToInchesPerSecond(0); // todo: figure out where encoders are plugged in
    }

    public double getRightFollowerVelocity() {
        return rpmToInchesPerSecond(0); // todo: figure out where encoders are plugged in
    }

    public double getBackFollowerVelocity() {
        return rpmToInchesPerSecond(0); // todo: figure out where encoders are plugged in
    }

    public synchronized void reloadGains() {
        //todo: implement this
    }

    @Override
    public void writeToLog() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }
}
