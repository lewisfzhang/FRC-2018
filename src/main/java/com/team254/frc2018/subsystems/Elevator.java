package com.team254.frc2018.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2018.Constants;
import com.team254.lib.drivers.TalonSRXChecker;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.drivers.TalonSRXUtil;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

// Top Soft limit : -151288
public class Elevator extends Subsystem {
    public static final double kHomePositionInches = 5.0;
    private static final int kHighGearSlot = 0;
    private static final int kLowGearSlot = 1;
    private static final int kReverseSoftLimit = -99000; // Encoder ticks (used to be -151000)
    private static final int kForwardSoftLimit = 500; // Encoder ticks.  TODO set to ~0 once skipping is fixed.
    private static final double kEncoderTicksPerInch = -1271.0;
    private static Elevator mInstance = null;
    private Intake mIntake = Intake.getInstance();
    private final TalonSRX mMaster, mRightSlave, mLeftSlaveA, mLeftSlaveB;
    private final Solenoid mShifter;
    private PeriodicInputs mPeriodicInputs = new PeriodicInputs();
    private PeriodicOutputs mPeriodicOutputs = new PeriodicOutputs();
    private ElevatorControlState mElevatorControlState = ElevatorControlState.OPEN_LOOP;

    private Elevator() {
        mMaster = TalonSRXFactory.createDefaultTalon(Constants.kElevatorMasterId);

        TalonSRXUtil.checkError(
                mMaster.configSelectedFeedbackSensor(
                        FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100),
                "Could not detect elevator encoder: ");

        TalonSRXUtil.checkError(
                mMaster.configForwardLimitSwitchSource(
                        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
                        Constants.kLongCANTimeoutMs),
                "Could not set forward (down) limit switch elevator: ");

        TalonSRXUtil.checkError(
                mMaster.configForwardSoftLimitThreshold(
                        kForwardSoftLimit, Constants.kLongCANTimeoutMs),
                "Could not set forward (down) soft limit switch elevator: ");

        TalonSRXUtil.checkError(
                mMaster.configForwardSoftLimitEnable(true, Constants.kLongCANTimeoutMs),
                "Could not enable forward (down) soft limit switch elevator: ");

        TalonSRXUtil.checkError(
                mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs),
                "Could not set voltage compensation saturation elevator: ");

        TalonSRXUtil.checkError(
                mMaster.configReverseSoftLimitThreshold(
                        kReverseSoftLimit, Constants.kLongCANTimeoutMs),
                "Could not set reverse (up) soft limit switch elevator: ");

        TalonSRXUtil.checkError(
                mMaster.configReverseSoftLimitEnable(
                        true, Constants.kLongCANTimeoutMs),
                "Could not enable reverse (up) soft limit switch elevator: ");

        //configure magic motion
        TalonSRXUtil.checkError(
                mMaster.config_kP(
                        kHighGearSlot, Constants.kElevatorHighGearKp, Constants.kLongCANTimeoutMs),
                "Could not set elevator kp: ");

        TalonSRXUtil.checkError(
                mMaster.config_kI(
                        kHighGearSlot, Constants.kElevatorHighGearKi, Constants.kLongCANTimeoutMs),
                "Could not set elevator ki: ");

        TalonSRXUtil.checkError(
                mMaster.config_kD(
                        kHighGearSlot, Constants.kElevatorHighGearKd, Constants.kLongCANTimeoutMs),
                "Could not set elevator kd: ");

        TalonSRXUtil.checkError(
                mMaster.config_kF(
                        kHighGearSlot, Constants.kElevatorHighGearKf, Constants.kLongCANTimeoutMs),
                "Could not set elevator kf: ");

        TalonSRXUtil.checkError(
                mMaster.configMaxIntegralAccumulator(
                        kHighGearSlot, Constants.kElevatorHighGearMaxIntegralAccumulator, Constants.kLongCANTimeoutMs),
                "Could not set elevator max integral: ");

        TalonSRXUtil.checkError(
                mMaster.config_IntegralZone(
                        kHighGearSlot, Constants.kElevatorHighGearIZone, Constants.kLongCANTimeoutMs),
                "Could not set elevator i zone: ");

        TalonSRXUtil.checkError(
                mMaster.configAllowableClosedloopError(
                        kHighGearSlot, Constants.kElevatorHighGearDeadband, Constants.kLongCANTimeoutMs),
                "Could not set elevator deadband: ");

        TalonSRXUtil.checkError(
                mMaster.configMotionAcceleration(
                        Constants.kElevatorHighGearAcceleration, Constants.kLongCANTimeoutMs),
                "Could not set elevator acceleration: ");

        TalonSRXUtil.checkError(
                mMaster.configMotionCruiseVelocity(
                        Constants.kElevatorHighGearCruiseVelocity, Constants.kLongCANTimeoutMs),
                "Could not set elevator cruise velocity: ");

        TalonSRXUtil.checkError(
                mMaster.configClosedloopRamp(
                        Constants.kElevatorRampRate, Constants.kLongCANTimeoutMs),
                "Could not set elevator voltage ramp rate: ");

        mMaster.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, 0);
        mMaster.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);

        // TODO add low gear gains

        mMaster.selectProfileSlot(0, 0);

        mMaster.overrideLimitSwitchesEnable(true);
        mMaster.overrideSoftLimitsEnable(true);

        mMaster.enableVoltageCompensation(true);

        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 10, 20);

        mMaster.setInverted(true);
        mMaster.setSensorPhase(true);

        mRightSlave = TalonSRXFactory.createPermanentSlaveTalon(Constants.kElevatorRightSlaveId,
                Constants.kElevatorMasterId);
        mRightSlave.setInverted(true);

        mLeftSlaveA = TalonSRXFactory.createPermanentSlaveTalon(Constants.kElevatorLeftSlaveAId,
                Constants.kElevatorMasterId);
        mLeftSlaveA.setInverted(false);

        mLeftSlaveB = TalonSRXFactory.createPermanentSlaveTalon(Constants.kElevatorLeftSlaveBId,
                Constants.kElevatorMasterId);
        mLeftSlaveB.setInverted(false);

        mShifter = Constants.makeSolenoidForId(Constants.kElevatorShifterSolenoidId);
        mShifter.set(true);

        // Start with zero power.
        mMaster.set(ControlMode.PercentOutput, 0);
        setNeutralMode(NeutralMode.Brake);
    }

    public synchronized static Elevator getInstance() {
        if (mInstance == null) {
            mInstance = new Elevator();
        }
        return mInstance;
    }

    public synchronized void setOpenLoop(double percentage) {
        mElevatorControlState = ElevatorControlState.OPEN_LOOP;
        mPeriodicOutputs.output_ = percentage;
    }

    public synchronized void setClosedLoopPosition(double positionInchesOffGround) {
        double positionInchesFromHome = positionInchesOffGround - kHomePositionInches;
        double encoderPosition = positionInchesFromHome * kEncoderTicksPerInch;
        setClosedLoopRawPosition(encoderPosition);
    }

    private synchronized void setClosedLoopRawPosition(double encoderPosition) {
        mElevatorControlState = ElevatorControlState.MOTION_MAGIC;
        mPeriodicOutputs.output_ = encoderPosition;
    }

    public synchronized boolean hasFinishedTrajectory() {
        return mElevatorControlState == ElevatorControlState.MOTION_MAGIC &&
                Util.epsilonEquals(mPeriodicInputs.active_trajectory_position_, mPeriodicOutputs.output_, 5);
    }

    public synchronized void setHangMode(boolean hang_mode) {
        mShifter.set(!hang_mode);
    }

    public synchronized double getRPM() {
        // We are using a CTRE mag encoder which is 4096 native units per revolution.
        return mPeriodicInputs.velocity_ticks_per_100ms_ * 10.0 / 4096.0 * 60.0;
    }

    public synchronized double getInchesOffGround() {
        return (mPeriodicInputs.position_ticks_ / kEncoderTicksPerInch) + kHomePositionInches;
    }

    public synchronized double getSetpoint() {
        return mElevatorControlState == ElevatorControlState.MOTION_MAGIC ?
                mPeriodicOutputs.output_ / kEncoderTicksPerInch + kHomePositionInches : Double.NaN;
    }

    public synchronized double getActiveTrajectoryAccelG() {
        return mPeriodicInputs.active_trajectory_accel_g_;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Elevator Output %", mPeriodicInputs.output_percent_);
        SmartDashboard.putNumber("Elevator RPM", getRPM());
        // SmartDashboard.putNumber("Elevator Error", mMaster.getClosedLoopError(0) / kEncoderTicksPerInch);
        SmartDashboard.putNumber("Elevator Height", getInchesOffGround());
        SmartDashboard.putBoolean("Elevator Limit", mPeriodicInputs.limit_switch_);
        SmartDashboard.putNumber("Elevator Sensor Height", mPeriodicInputs.position_ticks_);


        SmartDashboard.putNumber("Elevator Last Expected Trajectory", getSetpoint());
        SmartDashboard.putNumber("Elevator Current Trajectory Point", mPeriodicInputs.active_trajectory_position_);
        SmartDashboard.putNumber("Elevator Traj Vel", mPeriodicInputs.active_trajectory_velocity_);
        SmartDashboard.putNumber("Elevator Traj Accel", mPeriodicInputs.active_trajectory_accel_g_);
        SmartDashboard.putBoolean("Elevator Has Sent Trajectory", hasFinishedTrajectory());
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    @Override
    public void zeroSensors() {
        mMaster.setSelectedSensorPosition(0, 0, 10);
    }

    public synchronized void resetIfAtLimit() {
        if (mPeriodicInputs.limit_switch_) {
            zeroSensors();
        }
    }

    private void setNeutralMode(NeutralMode neutralMode) {
        mLeftSlaveA.setNeutralMode(neutralMode);
        mLeftSlaveB.setNeutralMode(neutralMode);
        mMaster.setNeutralMode(neutralMode);
        mRightSlave.setNeutralMode(neutralMode);
    }

    @Override
    public synchronized void readPeriodicInputs() {
        final double t = Timer.getFPGATimestamp();
        mPeriodicInputs.position_ticks_ = mMaster.getSelectedSensorPosition(0);
        mPeriodicInputs.velocity_ticks_per_100ms_ = mMaster.getSelectedSensorVelocity(0);
        if (mMaster.getControlMode() == ControlMode.MotionMagic) {
            mPeriodicInputs.active_trajectory_position_ = mMaster.getActiveTrajectoryPosition();
            final int newVel = mMaster.getActiveTrajectoryVelocity();
            // TODO check sign of elevator accel
            if (Util.epsilonEquals(newVel, Constants.kElevatorHighGearCruiseVelocity, 5) ||
                    Util.epsilonEquals(newVel, mPeriodicInputs.active_trajectory_velocity_, 5)) {
                // Elevator is ~constant velocity.
                mPeriodicInputs.active_trajectory_accel_g_ = 0.0;
            } else if (newVel > mPeriodicInputs.active_trajectory_velocity_) {
                // Elevator is accelerating downwards.
                mPeriodicInputs.active_trajectory_accel_g_ = -Constants.kElevatorHighGearAcceleration * 10.0 /
                        (kEncoderTicksPerInch * 386.09);
            } else {
                // Elevator is accelerating upwards.
                mPeriodicInputs.active_trajectory_accel_g_ = Constants.kElevatorHighGearAcceleration * 10.0 /
                        (kEncoderTicksPerInch * 386.09);
            }
            mPeriodicInputs.active_trajectory_velocity_ = newVel;
        } else {
            mPeriodicInputs.active_trajectory_position_ = Integer.MIN_VALUE;
            mPeriodicInputs.active_trajectory_velocity_ = 0;
            mPeriodicInputs.active_trajectory_accel_g_ = 0.0;
        }
        mPeriodicInputs.output_percent_ = mMaster.getMotorOutputPercent();
        mPeriodicInputs.limit_switch_ = mMaster.getSensorCollection().isFwdLimitSwitchClosed();
        mPeriodicInputs.t_ = t;

        if (getInchesOffGround() > Constants.kElevatorEpsilon && mShifter.get()) {
            mPeriodicInputs.feedforward_ = mIntake.hasCube() ? Constants.kElevatorFeedforwardWithCube : Constants
                    .kElevatorFeedforwardNoCube;
        } else {
            mPeriodicInputs.feedforward_ = 0.0;
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mMaster.set(mElevatorControlState == ElevatorControlState.MOTION_MAGIC ? ControlMode.MotionMagic :
                ControlMode.PercentOutput, mPeriodicOutputs.output_, DemandType.ArbitraryFeedForward, mPeriodicInputs
                .feedforward_);
    }

    @Override
    public boolean checkSystem() {
        setNeutralMode(NeutralMode.Coast);
        setHangMode(true);

        boolean leftSide =
                TalonSRXChecker.CheckTalons(this,
                        new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
                            {
                                add(new TalonSRXChecker.TalonSRXConfig("left_slave_a",
                                        mLeftSlaveA));
                                add(new TalonSRXChecker.TalonSRXConfig("left_slave_b",
                                        mLeftSlaveB));
                            }
                        }, new TalonSRXChecker.CheckerConfig() {
                            {
                                mCurrentFloor = 2;
                                mRPMFloor = 200;
                                mCurrentEpsilon = 2.0;
                                mRPMEpsilon = 250;
                                mRunTimeSec = 0.4;
                                mRunOutputPercentage = -0.4;
                                mRPMSupplier = () -> -mMaster.getSelectedSensorVelocity(0);
                            }
                        });
        boolean rightSide =
                TalonSRXChecker.CheckTalons(this,
                        new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
                            {
                                add(new TalonSRXChecker.TalonSRXConfig("master", mMaster));
                                add(new TalonSRXChecker.TalonSRXConfig("right_slave", mRightSlave));
                            }
                        }, new TalonSRXChecker.CheckerConfig() {
                            {
                                mCurrentFloor = 2;
                                mRPMFloor = 200;
                                mCurrentEpsilon = 2.0;
                                mRPMEpsilon = 250;
                                mRunTimeSec = 0.4;
                                mRunOutputPercentage = -0.4;
                                mRPMSupplier = () -> -mMaster.getSelectedSensorVelocity(0);
                            }
                        });

        setHangMode(false);
        setNeutralMode(NeutralMode.Brake);
        return leftSide && rightSide;
    }

    private enum ElevatorControlState {
        OPEN_LOOP,
        MOTION_MAGIC
    }

    private static class PeriodicInputs {
        public int position_ticks_;
        public int velocity_ticks_per_100ms_;
        public double active_trajectory_accel_g_;
        public int active_trajectory_velocity_;
        public int active_trajectory_position_;
        public double output_percent_;
        public boolean limit_switch_;
        public double feedforward_;
        public double t_;
    }

    private static class PeriodicOutputs {
        public double output_;
    }

}
