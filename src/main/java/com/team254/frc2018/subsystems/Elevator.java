package com.team254.frc2018.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2018.Constants;
import com.team254.frc2018.loops.Looper;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.drivers.TalonSRXChecker;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

// Top Soft limit : -151288
public class Elevator extends Subsystem {
    private static final int kHighGearSlot = 0;
    private static final int kLowGearSlot = 1;
    private final int kReverseSoftLimit = -96000; // Encoder ticks (used to be -151000)
    private final int kForwardSoftLimit = 500; // Encoder ticks.  TODO set to ~0 once skipping is fixed.
    private final double kEncoderTicksPerInch = -1271.0;
    public static final double kHomePositionInches = 5.0;

    private static Elevator mInstance = null;

    public synchronized static Elevator getInstance() {
        if (mInstance == null) {
            mInstance = new Elevator();
        }
        return mInstance;
    }

    private final TalonSRX mMaster, mRightSlave, mLeftSlaveA, mLeftSlaveB;

    private double mLastTrajectoryPoint = Double.NaN;

    private Elevator() {
        mMaster = TalonSRXFactory.createDefaultTalon(Constants.kElevatorMasterId);
        ErrorCode errorCode =
                mMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                        0, 100);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not detect elevator encoder: " + errorCode, false);
        errorCode = mMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal
                .NormallyOpen, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set forward (down) limit switch elevator: " + errorCode, false);
        errorCode = mMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal
                .NormallyOpen, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set reverse (up) limit switch elevator: " + errorCode, false);
        errorCode = mMaster.configForwardSoftLimitThreshold(kForwardSoftLimit, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set forward (down) soft limit switch elevator: " + errorCode, false);
        errorCode = mMaster.configForwardSoftLimitEnable(true, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not enable forward (down) soft limit switch elevator: " + errorCode, false);
        errorCode = mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        errorCode = mMaster.configReverseSoftLimitThreshold(kReverseSoftLimit, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set reverse (up) soft limit switch elevator: " + errorCode, false);
        errorCode = mMaster.configReverseSoftLimitEnable(true, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not enable reverse (up) soft limit switch elevator: " + errorCode, false);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set elevator voltage compensation: " + errorCode, false);

        //configure magic motion
        errorCode = mMaster.config_kP(kHighGearSlot, Constants.kElevatorHighGearKp, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set elevator kp: " + errorCode, false);

        errorCode = mMaster.config_kI(kHighGearSlot, Constants.kElevatorHighGearKi, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set elevator ki: " + errorCode, false);

        errorCode = mMaster.config_kD(kHighGearSlot, Constants.kElevatorHighGearKd, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set elevator kd: " + errorCode, false);

        errorCode = mMaster.config_kF(kHighGearSlot, Constants.kElevatorHighGearKf, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set elevator kf: " + errorCode, false);

        errorCode = mMaster.configMaxIntegralAccumulator(kHighGearSlot, Constants.kElevatorHighGearMaxIntegralAccumulator,
                Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set elevator max integral: " + errorCode, false);

        errorCode = mMaster.config_IntegralZone(kHighGearSlot, Constants.kElevatorHighGearIZone, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set elevator i zone: " + errorCode, false);

        errorCode = mMaster.configAllowableClosedloopError(kHighGearSlot, Constants.kElevatorHighGearDeadband, Constants
                .kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set elevator deadband: " + errorCode, false);

        errorCode = mMaster.configMotionAcceleration(Constants.kElevatorHighGearAcceleration, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set elevator acceleration: " + errorCode, false);

        errorCode = mMaster.configMotionCruiseVelocity(Constants.kElevatorHighGearCruiseVelocity, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set elevator cruise velocity: " + errorCode, false);

        errorCode = mMaster.configSetParameter(ParamEnum.eClearPositionOnLimitF, 1, 0, 0, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set elevator reset on limit f: " + errorCode, false);

        // TODO add low gear gains

        mMaster.selectProfileSlot(0, 0);

        mMaster.setInverted(false);
        mMaster.setSensorPhase(true);
        mMaster.setNeutralMode(NeutralMode.Brake);
        mMaster.overrideLimitSwitchesEnable(true);
        mMaster.overrideSoftLimitsEnable(true);

        mMaster.enableVoltageCompensation(true);
        mMaster.set(ControlMode.PercentOutput, 0);

        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,10, 10);
        mMaster.setInverted(false);
        mMaster.setSensorPhase(true);
        mMaster.setNeutralMode(NeutralMode.Brake);

        // Re-zero on limit
        mMaster.configSetParameter(
                ParamEnum.eClearPositionOnLimitF, 1, 0, 0, 0);

        mRightSlave = TalonSRXFactory.createPermanentSlaveTalon(Constants.kElevatorRightSlaveId,
                Constants.kElevatorMasterId);
        mRightSlave.setInverted(false);
        mRightSlave.setNeutralMode(NeutralMode.Brake);

        mLeftSlaveA = TalonSRXFactory.createPermanentSlaveTalon(Constants.kElevatorLeftSlaveAId,
                Constants.kElevatorMasterId);
        mLeftSlaveA.setInverted(true);
        mLeftSlaveA.setNeutralMode(NeutralMode.Brake);

        mLeftSlaveB = TalonSRXFactory.createPermanentSlaveTalon(Constants.kElevatorLeftSlaveBId,
                Constants.kElevatorMasterId);
        mLeftSlaveB.setInverted(true);
        mLeftSlaveB.setNeutralMode(NeutralMode.Brake);
    }

    public synchronized void setOpenLoop(double percentage) {
        mMaster.set(ControlMode.PercentOutput, percentage);
    }

    public synchronized void setClosedLoopPosition(double positionInchesOffGround) {
        double positionInchesFromHome = positionInchesOffGround - kHomePositionInches;
        double encoderPosition = positionInchesFromHome * kEncoderTicksPerInch;
        setClosedLoopRawPosition(encoderPosition);
    }

    private synchronized void setClosedLoopRawPosition(double encoderPosition) {
        mMaster.set(ControlMode.MotionMagic, encoderPosition);
        mLastTrajectoryPoint = encoderPosition;
    }

    public synchronized boolean hasFinishedTrajectory() {
        if (Util.epsilonEquals(mMaster.getActiveTrajectoryPosition(), mLastTrajectoryPoint, 5)) {
            return true;
        }
        return false;
    }

    public synchronized double getRPM() {
        // We are using a CTRE mag encoder which is 4096 native units per revolution.
        // GetVelocity is in native units per 100ms.
        return mMaster.getSelectedSensorVelocity(0) * 10.0 / 4096.0 * 60.0;
    }

    public synchronized double getInchesOffGround() {
        return (mMaster.getSelectedSensorPosition(0) / kEncoderTicksPerInch) + kHomePositionInches;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Elevator Master Output", mMaster.getMotorOutputPercent());
        SmartDashboard.putNumber("Elevator Master RPM", getRPM());
        SmartDashboard.putNumber("Elevator error", mMaster.getClosedLoopError(0));
        SmartDashboard.putNumber("Elevator Master Position",
                mMaster.getSelectedSensorPosition(0));
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    @Override
    public void zeroSensors() {
        mMaster.setSelectedSensorPosition(0,0, 10);
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
    }

    @Override
    public boolean checkSystem() {
        boolean rightSide =
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
                                mRPMFloor = 1500;
                                mCurrentEpsilon = 2.0;
                                mRPMEpsilon = 250;
                                mRunTimeSec = 1.0;
                                mRunOutputPercentage = 0.25;
                                mRPMSupplier = () -> mMaster.getSelectedSensorVelocity(0);
                            }
                        });
        boolean leftSide =
                TalonSRXChecker.CheckTalons(this,
                        new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
                            {
                                add(new TalonSRXChecker.TalonSRXConfig("master", mMaster));
                                add(new TalonSRXChecker.TalonSRXConfig("right_slave", mRightSlave));
                            }
                        }, new TalonSRXChecker.CheckerConfig() {
                            {
                                mCurrentFloor = 2;
                                mRPMFloor = 1500;
                                mCurrentEpsilon = 2.0;
                                mRPMEpsilon = 250;
                                mRunTimeSec = 1.0;
                                mRunOutputPercentage = 0.25;
                                mRPMSupplier = () -> mMaster.getSelectedSensorVelocity(0);
                            }
                        });
        return leftSide && rightSide;
    }

}
