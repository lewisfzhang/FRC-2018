package com.team254.frc2018.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2018.Constants;
import com.team254.frc2018.loops.ILooper;
import com.team254.frc2018.loops.Loop;
import com.team254.lib.drivers.TalonSRXChecker;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.drivers.TalonSRXUtil;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Wrist extends Subsystem {
    private static final int kMagicMotionSlot = 0;
    private static final int kForwardSoftLimit = 2100;  // Encoder ticks.
    private static final int kReverseSoftLimit = -500;  // Encoder ticks.  TODO make ~0 once skipping is fixed.

    private static final double kHomingOutput = -0.25;

    private static Wrist mInstance;
    private final TalonSRX mMaster;
    private PeriodicInputs mPeriodicInputs = new PeriodicInputs();
    private PeriodicOutputs mPeriodicOutputs = new PeriodicOutputs();
    private double mZeroPosition = Double.NaN;
    private SystemState mSystemState = SystemState.HOMING;
    private SystemState mDesiredState = SystemState.CLOSED_LOOP;

    private Wrist() {
        mMaster = TalonSRXFactory.createDefaultTalon(Constants.KWristMasterId);
        ErrorCode errorCode;

        //configure talon
        errorCode = mMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, Constants
                .kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not detect wrist encoder: " + errorCode, false);
        errorCode = mMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal
                .NormallyOpen, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set forward limit switch wrist: " + errorCode, false);
        errorCode = mMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal
                .NormallyOpen, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set reverse limit switch wrist: " + errorCode, false);

        errorCode = mMaster.configForwardSoftLimitThreshold(kForwardSoftLimit, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set forward soft limit switch wrist: " + errorCode, false);
        errorCode = mMaster.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not enable forward soft limit switch wrist: " + errorCode, false);
        errorCode = mMaster.configReverseSoftLimitThreshold(kReverseSoftLimit, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set reverse soft limit switch wrist: " + errorCode, false);
        errorCode = mMaster.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not enable reverse soft limit switch wrist: " + errorCode, false);

        errorCode = mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist voltage compensation: " + errorCode, false);

        //configure magic motion
        errorCode = mMaster.config_kP(kMagicMotionSlot, Constants.kWristKp, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist kp: " + errorCode, false);

        errorCode = mMaster.config_kI(kMagicMotionSlot, Constants.kWristKi, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist ki: " + errorCode, false);

        errorCode = mMaster.config_kD(kMagicMotionSlot, Constants.kWristKd, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist kd: " + errorCode, false);

        errorCode = mMaster.config_kF(kMagicMotionSlot, Constants.kWristKf, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist kf: " + errorCode, false);

        errorCode = mMaster.configMaxIntegralAccumulator(kMagicMotionSlot, Constants.kWristMaxIntegralAccumulator,
                Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist max integral: " + errorCode, false);

        errorCode = mMaster.config_IntegralZone(kMagicMotionSlot, Constants.kWristIZone, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist i zone: " + errorCode, false);

        errorCode = mMaster.configAllowableClosedloopError(kMagicMotionSlot, Constants.kWristDeadband, Constants
                .kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist deadband: " + errorCode, false);

        errorCode = mMaster.configMotionAcceleration(Constants.kWristAcceleration, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist acceleration: " + errorCode, false);

        errorCode = mMaster.configMotionCruiseVelocity(Constants.kWristCruiseVelocity, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist cruise velocity: " + errorCode, false);

        TalonSRXUtil.checkError(
                mMaster.configContinuousCurrentLimit(20, Constants.kLongCANTimeoutMs),
                "Could not set wrist continuous current limit.");

        TalonSRXUtil.checkError(
                mMaster.configPeakCurrentLimit(40, Constants.kLongCANTimeoutMs),
                "Could not set wrist peak current limit.");

        TalonSRXUtil.checkError(
                mMaster.configPeakCurrentDuration(200, Constants.kLongCANTimeoutMs),
                "Could not set wrist peak current duration.");

        TalonSRXUtil.checkError(
                mMaster.configClosedloopRamp(
                        Constants.kWristRampRate, Constants.kLongCANTimeoutMs),
                "Could not set wrist voltage ramp rate: ");

        mMaster.enableCurrentLimit(true);

        mMaster.selectProfileSlot(0, 0);

        mMaster.setInverted(false);
        mMaster.setSensorPhase(true);
        mMaster.setNeutralMode(NeutralMode.Brake);
        mMaster.overrideLimitSwitchesEnable(true);
        mMaster.overrideSoftLimitsEnable(true);

        mMaster.enableVoltageCompensation(true);
        mMaster.set(ControlMode.PercentOutput, 0);

        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 10, 20);

        // DO NOT reset encoder positions on limit switch
        mMaster.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, 0);
        mMaster.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);
    }

    public synchronized static Wrist getInstance() {
        if (mInstance == null) {
            mInstance = new Wrist();
        }
        return mInstance;
    }

    // Converts sensor units from [0, 4096 / 2] to a position based on zeroed position.
    private double relativeToAbsolute(double position) {
        return position + mZeroPosition;
    }

    // Inverse of above.
    private double absoluteToRelative(double position) {
        return position - mZeroPosition;
    }

    @Override
    public synchronized void outputToSmartDashboard() {
        if (!Double.isNaN(mZeroPosition)) {
            SmartDashboard.putNumber("Wrist Angle", getAngle());
            SmartDashboard.putNumber("Wrist Position", getPosition());
        }
        SmartDashboard.putNumber("Wrist RPM", getRPM());
        SmartDashboard.putNumber("Wrist Power %", mPeriodicInputs.output_percent_);
        // SmartDashboard.putNumber("Wrist Error", sensorUnitsToDegrees(mMaster.getClosedLoopError(0)));
        SmartDashboard.putBoolean("Wrist Limit Switch", mPeriodicInputs.limit_switch_);
        SmartDashboard.putNumber("Wrist Last Expected Trajectory", getSetpoint());
        SmartDashboard.putNumber("Wrist Current Trajectory Point", mPeriodicInputs.active_trajectory_position_);
        SmartDashboard.putBoolean("Wrist Has Sent Trajectory", hasFinishedTrajectory());
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    @Override
    public synchronized void zeroSensors() {
        mZeroPosition = mPeriodicInputs.position_ticks_;

        // TODO is this needed?
        mMaster.configForwardSoftLimitThreshold((int) relativeToAbsolute(kForwardSoftLimit), 0);
        mMaster.configForwardSoftLimitEnable(false, 0);

        mMaster.configReverseSoftLimitThreshold((int) relativeToAbsolute(kReverseSoftLimit), 0);
        mMaster.configReverseSoftLimitEnable(false, 0);
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            double mHomingStartTime;
            boolean mFoundHome;

            @Override
            public void onStart(double timestamp) {
                mHomingStartTime = timestamp;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Wrist.this) {
                    if (!Double.isNaN(mZeroPosition) && mDesiredState != mSystemState) {
                        System.out.println(timestamp + ": Wrist changed states: " + mSystemState + " -> " +
                                mDesiredState);
                        mSystemState = mDesiredState;
                    }

                    switch (mSystemState) {
                        case OPEN_LOOP:
                            // mMaster.set(ControlMode.PercentOutput, mOutputValue);
                            break;
                        case CLOSED_LOOP:
                            // mMaster.set(ControlMode.MotionMagic, mOutputValue);
                            break;
                        case HOMING:
                            if (Double.isNaN(mZeroPosition)) {
                                mPeriodicOutputs.output_ = resetIfAtLimit() ? 0.0 : kHomingOutput;
                            }
                            break;
                        default:
                            System.out.println("Fell through on Wrist states!");
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicOutputs.output_ = percentage;
        mDesiredState = SystemState.OPEN_LOOP;
    }

    public synchronized boolean resetIfAtLimit() {
        if (mMaster.getSensorCollection().isRevLimitSwitchClosed()) {
            zeroSensors();
            return true;
        }
        return false;
    }

    /**
     * @param position the target position of the wrist in sensor units
     */
    public void setClosedLoop(int position) {
        mPeriodicOutputs.output_ = relativeToAbsolute(position);
        mDesiredState = SystemState.CLOSED_LOOP;
    }

    /**
     * @param angle the target position of the wrist in degrees.  0 is full back, 180 is facing forwards
     */
    public synchronized void setClosedLoopAngle(double angle) {
        mPeriodicOutputs.output_ = relativeToAbsolute(degreesToSensorUnits(angle));
        mDesiredState = SystemState.CLOSED_LOOP;
    }

    /**
     * @return current position of the wrist in sensor units
     */
    public synchronized double getPosition() { //returns angle of wrist in degrees
        return absoluteToRelative(mPeriodicInputs.position_ticks_);
    }

    /**
     * @return current angle of the wrist in degrees
     */
    public synchronized double getAngle() { //returns angle of wrist in degrees
        return sensorUnitsToDegrees(absoluteToRelative(mPeriodicInputs.position_ticks_));
    }

    /**
     * @return current velocity in rpm
     */
    public double getRPM() {
        return sensorUnitsToDegrees(mPeriodicInputs.velocity_ticks_per_100ms_) * 600.0 / 360.0;
    }

    /**
     * @return current velocity in degrees per second
     */
    public double getDegreesPerSecond() {
        return sensorUnitsToDegrees(mPeriodicInputs.velocity_ticks_per_100ms_) * 10.0;
    }

    public synchronized boolean hasFinishedTrajectory() {
        if (Util.epsilonEquals(mPeriodicInputs.active_trajectory_position_,
                getSetpoint(), 2)) {
            return true;
        }
        return false;
    }

    public synchronized double getSetpoint() {
        return mDesiredState == SystemState.CLOSED_LOOP ? sensorUnitsToDegrees(absoluteToRelative(mPeriodicOutputs
                .output_)) : Double.NaN;
    }

    private double sensorUnitsToDegrees(double units) {
        return units / 4096.0 * 360.0;
    }

    private double degreesToSensorUnits(double degrees) {
        return degrees * 4096.0 / 360.0;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        if (mMaster.hasResetOccurred()) {
            DriverStation.reportError("Wrist Talon Reset! ",false);
        }
        StickyFaults faults = new StickyFaults();
        mMaster.getStickyFaults(faults);
        if (faults.hasAnyFault()) {
            DriverStation.reportError("Wrist Talon Fault! " + faults.toString(),false);
        }
        if (mMaster.getControlMode() == ControlMode.MotionMagic) {
            mPeriodicInputs.active_trajectory_position_ = mMaster.getActiveTrajectoryPosition();

            if (mPeriodicInputs.active_trajectory_position_ < kReverseSoftLimit) {
                DriverStation.reportError("Active trajectory past reverse soft limit!", false);
            } else if (mPeriodicInputs.active_trajectory_position_ > kForwardSoftLimit) {
                DriverStation.reportError("Active trajectory past forward soft limit!", false);
            }
        } else {
            mPeriodicInputs.active_trajectory_position_ = Integer.MIN_VALUE;
        }
        mPeriodicInputs.limit_switch_ = mMaster.getSensorCollection().isRevLimitSwitchClosed();
        mPeriodicInputs.output_percent_ = mMaster.getMotorOutputPercent();
        mPeriodicInputs.position_ticks_ = mMaster.getSelectedSensorPosition(0);
        mPeriodicInputs.velocity_ticks_per_100ms_ = mMaster.getSelectedSensorVelocity(0);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mMaster.set(mDesiredState == SystemState.CLOSED_LOOP ? ControlMode.MotionMagic : ControlMode.PercentOutput,
                mPeriodicOutputs.output_);
    }

    @Override
    public boolean checkSystem() {
        return TalonSRXChecker.CheckTalons(this,
                new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
                    {
                        add(new TalonSRXChecker.TalonSRXConfig("wrist_master", mMaster));
                    }
                }, new TalonSRXChecker.CheckerConfig() {
                    {
                        mRunTimeSec = 0.5;
                        mRunOutputPercentage = 0.25;

                        mRPMFloor = 50.0;
                        mCurrentFloor = 2.0;

                        mRPMSupplier = () -> mMaster.getSelectedSensorVelocity(0);
                    }
                });
    }

    public enum SystemState {
        HOMING,
        CLOSED_LOOP,
        OPEN_LOOP,
    }

    private static class PeriodicInputs {
        public int position_ticks_;
        public int velocity_ticks_per_100ms_;
        public int active_trajectory_position_;
        public double output_percent_;
        public boolean limit_switch_;
    }

    private static class PeriodicOutputs {
        public double output_;
    }
}
