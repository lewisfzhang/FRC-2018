package com.team254.frc2018.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2018.Constants;
import com.team254.frc2018.loops.Loop;
import com.team254.frc2018.loops.Looper;
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

    private static Wrist mInstance;

    public synchronized static Wrist getInstance() {
        if (mInstance == null) {
            mInstance = new Wrist();
        }
        return mInstance;
    }

    private final TalonSRX mMaster;
    private double mLastTrajectoryPoint = Double.NaN;

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
        errorCode = mMaster.configForwardSoftLimitEnable(true, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not enable forward soft limit switch wrist: " + errorCode, false);
        errorCode = mMaster.configReverseSoftLimitThreshold(kReverseSoftLimit, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set reverse soft limit switch wrist: " + errorCode, false);
        errorCode = mMaster.configReverseSoftLimitEnable(true, Constants.kLongCANTimeoutMs);
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

        mMaster.enableCurrentLimit(true);

        mMaster.selectProfileSlot(0, 0);

        mMaster.setInverted(false);
        mMaster.setSensorPhase(true);
        mMaster.setNeutralMode(NeutralMode.Brake);
        mMaster.overrideLimitSwitchesEnable(true);
        mMaster.overrideSoftLimitsEnable(true);

        mMaster.enableVoltageCompensation(true);
        mMaster.set(ControlMode.PercentOutput, 0);

        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,10, 20);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer,10, 20);

        // DO NOT reset encoder positions on limit switch
        mMaster.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, 0);
        mMaster.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);
    }

    @Override
    public synchronized void outputToSmartDashboard() {
        SmartDashboard.putNumber("Wrist Angle", getAngle());
        SmartDashboard.putNumber("Wrist Position", getPosition());
        SmartDashboard.putNumber("Wrist RPM", getRPM());
        SmartDashboard.putNumber("Wrist Power %", mMaster.getMotorOutputPercent());
        SmartDashboard.putNumber("Wrist Error", sensorUnitsToDegrees(mMaster.getClosedLoopError(0)));
        SmartDashboard.putBoolean("Wrist Limit Switch", mMaster.getSensorCollection().isRevLimitSwitchClosed());
        SmartDashboard.putNumber("Wrist Last Expected Trajectory", mLastTrajectoryPoint);
        SmartDashboard.putNumber("Wrist Current Trajectory Point", mMaster.getActiveTrajectoryPosition());
        SmartDashboard.putBoolean("Wrist Has Sent Trajectory", hasFinishedTrajectory());
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    @Override
    public synchronized void zeroSensors() {
        mMaster.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
    }

    public synchronized void setOpenLoop(double percentage) {
        mMaster.set(ControlMode.PercentOutput, percentage);
    }

    public synchronized void resetIfAtLimit() {
        if (mMaster.getSensorCollection().isRevLimitSwitchClosed()) {
            zeroSensors();
        }
    }

    /**
     * @param position the target position of the wrist in sensor units
     */
    public void setClosedLoop(double position) {
        mLastTrajectoryPoint = position;
        mMaster.set(ControlMode.MotionMagic, position);
    }

    /**
     * @param angle the target position of the wrist in degrees.  0 is full back, 180 is facing forwards
     */
    public synchronized void setClosedLoopAngle(double angle) {
        mLastTrajectoryPoint = degreesToSensorUnits(angle);
        mMaster.set(ControlMode.MotionMagic, mLastTrajectoryPoint);
    }

    /**
     * @return current position of the wrist in sensor units
     */
    public synchronized double getPosition() { //returns angle of wrist in degrees
        return mMaster.getSelectedSensorPosition(0);
    }

    /**
     * @return current angle of the wrist in degrees
     */
    public synchronized double getAngle() { //returns angle of wrist in degrees
        return sensorUnitsToDegrees(mMaster.getSelectedSensorPosition(0));
    }

    /**
     * @return current velocity in rpm
     */
    public double getRPM() {
        return sensorUnitsToDegrees(mMaster.getSelectedSensorVelocity(0)) * 600.0 / 360.0;
    }

    /**
     * @return current velocity in degrees per second
     */
    public double getDegreesPerSecond() {
        return sensorUnitsToDegrees(mMaster.getSelectedSensorVelocity(0)) * 10.0;
    }

    public synchronized boolean hasFinishedTrajectory() {
        if (Util.epsilonEquals(mMaster.getActiveTrajectoryPosition(),
                mLastTrajectoryPoint, 2)) {
            return true;
        }
        return false;
    }

    public synchronized double getSetpoint() {
        return sensorUnitsToDegrees(mLastTrajectoryPoint);
    }

    private double sensorUnitsToDegrees(double units) {
        return units / 4096.0 * 360.0;
    }

    private double degreesToSensorUnits(double degrees) {
        return degrees * 4096.0 / 360.0;
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
}
