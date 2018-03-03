package com.team254.frc2018.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team254.frc2018.Constants;
import com.team254.frc2018.loops.ILooper;
import com.team254.frc2018.loops.Loop;
import com.team254.frc2018.planners.DriveMotionPlanner;
import com.team254.lib.drivers.TalonSRXChecker;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

public class Drive extends Subsystem {

    private static final int kLowGearVelocityControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;
    private static final double DRIVE_ENCODER_PPR = 4096.;
    private static Drive mInstance = new Drive();
    // Hardware
    private final TalonSRX mLeftMaster, mRightMaster, mLeftSlaveA, mRightSlaveA, mLeftSlaveB, mRightSlaveB;
    private final Solenoid mShifter;
    // Control states
    private DriveControlState mDriveControlState;
    private PigeonIMU mPigeon;
    // Hardware states
    private PeriodicInputs mPeriodicInputs;
    private PeriodicOutputs mPeriodicOutputs;
    private boolean mAutoShift;
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;
    private ReflectingCSVWriter<PeriodicInputs> mCSVWriter = null;
    private DriveMotionPlanner mMotionPlanner;

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
                /*
                // TODO: fix this (tom)
                if (mAutoShift) {
                    handleAutoShift();
                } else */
                {
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
        mPeriodicInputs = new PeriodicInputs();
        mPeriodicOutputs = new PeriodicOutputs();

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
        mLeftSlaveB.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 10, 10);

        // Force a solenoid message.
        mIsHighGear = true;
        setHighGear(false);

        setOpenLoop(DriveSignal.NEUTRAL);

        // Force a CAN message across.
        mIsBrakeMode = true;
        setBrakeMode(false);

        mMotionPlanner = new DriveMotionPlanner();
    }

    public TrajectoryIterator<TimedState<Pose2dWithCurvature>> generateTrajectory(
            final List<Pose2d> waypoints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        Trajectory<TimedState<Pose2dWithCurvature>> trajectory = mMotionPlanner.generateTrajectory(waypoints, max_vel, max_accel, max_voltage);
        TrajectoryIterator<TimedState<Pose2dWithCurvature>> iterator = new TrajectoryIterator<>(new TimedView<>(trajectory));
        return iterator;
    }

    public static Drive getInstance() {
        return mInstance;
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

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return (rad_s / Math.PI * 2.0) * 4096.0 / 10.0;
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
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
        mPeriodicOutputs.left_output_ = signal.getLeft();
        mPeriodicOutputs.right_output_ = signal.getRight();
        mPeriodicOutputs.left_feedforward_ = 0.0;
        mPeriodicOutputs.right_feedforward_ = 0.0;
    }

    /**
     * Configures talons for velocity control
     */
    private synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            // We entered a velocity control state.
            setBrakeMode(true);
            mAutoShift = false;
            mLeftMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            mRightMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);

            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
        mPeriodicOutputs.left_output_ = signal.getLeft();
        mPeriodicOutputs.right_output_ = signal.getRight();
        // TODO reenable after tuning!
        // mPeriodicOutputs.left_feedforward_ = feedforward.getLeft();
        // mPeriodicOutputs.right_feedforward_ = feedforward.getRight();
    }

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        if(mMotionPlanner != null) {
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
    }

    public boolean isDoneWithTrajectory() {
        if(mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            return false;
        }
        return mMotionPlanner.isDone();
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

    public Rotation2d getHeading() {
        return mPeriodicInputs.gyro_heading_;
    }

    public void setHeading(Rotation2d heading) {
        mPigeon.setFusedHeading(heading.getDegrees(), 10);
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Right Drive Distance", mPeriodicInputs.right_distance_);
        SmartDashboard.putNumber("Right Drive Ticks", mPeriodicInputs.right_position_ticks_);
        SmartDashboard.putNumber("Left Drive Ticks", mPeriodicInputs.left_position_ticks_);
        SmartDashboard.putNumber("Left Drive Distance", mPeriodicInputs.left_distance_);
        SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());
        if(getHeading() != null) {
            SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
        }
        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    public synchronized void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition(0, 0, 0);
        mRightMaster.setSelectedSensorPosition(0, 0, 0);
        mPeriodicInputs = new PeriodicInputs();
    }

    @Override
    public void zeroSensors() {
        setHeading(Rotation2d.identity());
        resetEncoders();
        mAutoShift = true;
    }

    public double getLeftEncoderRotations() {
        return mPeriodicInputs.left_position_ticks_ / DRIVE_ENCODER_PPR;
    }

    public double getRightEncoderRotations() {
        return mPeriodicInputs.right_position_ticks_ / DRIVE_ENCODER_PPR;
    }

    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    public double getRightVelocityNativeUnits() {
        return mPeriodicInputs.right_velocity_ticks_per_100ms_;
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLeftVelocityNativeUnits() {
        return mPeriodicInputs.left_velocity_ticks_per_100ms_;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
    }

    private void updatePathFollower() {
        if(mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            DriveMotionPlanner.Output output = mMotionPlanner.update(Timer.getFPGATimestamp());
            // DriveSignal signal = new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0);

            setVelocity(new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity), radiansPerSecondToTicksPer100ms(output.right_velocity)),
                    new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0));
        } else {
            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    private void handleAutoShift() {
        final double linear_velocity = Math.abs(getLinearVelocity());
        final double angular_velocity = Math.abs(getAngularVelocity());
        if (mIsHighGear && linear_velocity < Constants.kDriveDownShiftVelocity && angular_velocity < Constants
                .kDriveDownShiftAngularVelocity) {
            setHighGear(false);
        } else if (!mIsHighGear && linear_velocity > Constants.kDriveUpShiftVelocity) {
            setHighGear(true);
        }
    }

    public synchronized void reloadGains() {
        mLeftMaster.config_kP(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKp, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kI(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKi, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kD(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKd, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kF(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKf, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_IntegralZone(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);

        mRightMaster.config_kP(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKp, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kI(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKi, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kD(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKd, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kF(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKf, Constants.kLongCANTimeoutMs);
        mRightMaster.config_IntegralZone(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);
    }

    @Override
    public void writeToLog() {
    }

    @Override
    public synchronized void readPeriodicInputs() {
        double prevLeftTicks = mPeriodicInputs.left_position_ticks_;
        double prevRightTicks = mPeriodicInputs.right_position_ticks_;
        mPeriodicInputs.left_position_ticks_ = mLeftMaster.getSelectedSensorPosition(0);
        mPeriodicInputs.right_position_ticks_ = mRightMaster.getSelectedSensorPosition(0);
        mPeriodicInputs.left_velocity_ticks_per_100ms_ = mLeftMaster.getSelectedSensorVelocity(0);
        mPeriodicInputs.right_velocity_ticks_per_100ms_ = mRightMaster.getSelectedSensorVelocity(0);
        mPeriodicInputs.gyro_heading_ = Rotation2d.fromDegrees(mPigeon.getFusedHeading());

        double deltaLeftTicks = ((mPeriodicInputs.left_position_ticks_ - prevLeftTicks) / 4096.0) * Math.PI;
        if (deltaLeftTicks > 0.0) {
            mPeriodicInputs.left_distance_ += deltaLeftTicks * Constants.kDriveWheelDiameterInchesForwards;
        } else {
            mPeriodicInputs.left_distance_ += deltaLeftTicks * Constants.kDriveWheelDiameterInchesReverse;
        }

        double deltaRightTicks = ((mPeriodicInputs.right_position_ticks_ - prevRightTicks) / 4096.0) * Math.PI;
        if (deltaRightTicks > 0.0) {
            mPeriodicInputs.right_distance_ += deltaRightTicks * Constants.kDriveWheelDiameterInchesForwards;
        } else {
            mPeriodicInputs.right_distance_ += deltaRightTicks * Constants.kDriveWheelDiameterInchesReverse;
        }

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicInputs);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mLeftMaster.set(mDriveControlState == DriveControlState.OPEN_LOOP ? ControlMode.PercentOutput :
                ControlMode.Velocity, mPeriodicOutputs.left_output_, DemandType.ArbitraryFeedForward, mPeriodicOutputs.left_feedforward_);
        mRightMaster.set(mDriveControlState == DriveControlState.OPEN_LOOP ? ControlMode.PercentOutput :
                ControlMode.Velocity, mPeriodicOutputs.right_output_, DemandType.ArbitraryFeedForward, mPeriodicOutputs.right_feedforward_);
        mPeriodicInputs.left_velocity_setpoint_ = mPeriodicOutputs.left_output_;
        mPeriodicInputs.right_velocity_setpoint_ = mPeriodicOutputs.right_output_;

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

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicInputs.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
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

    public static class PeriodicInputs {
        public int left_position_ticks_;
        public int right_position_ticks_;
        public double left_distance_;
        public double right_distance_;
        public int left_velocity_ticks_per_100ms_;
        public int right_velocity_ticks_per_100ms_;
        public Rotation2d gyro_heading_;

        // TODO this is a hack
        public double left_velocity_setpoint_;
        public double right_velocity_setpoint_;
    }

    private static class PeriodicOutputs {
        public double left_output_;
        public double right_output_;
        public double left_feedforward_;
        public double right_feedforward_;
    }
}
