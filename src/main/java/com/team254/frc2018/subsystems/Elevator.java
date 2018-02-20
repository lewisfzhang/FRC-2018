package com.team254.frc2018.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2018.Constants;
import com.team254.frc2018.loops.Looper;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.drivers.TalonSRXChecker;
import com.team254.lib.drivers.TalonSRXUtil;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
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

    private final Solenoid mShifter;

    private double mLastTrajectoryPoint = Double.NaN;

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

        mMaster.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, 0);
        mMaster.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);

        // TODO add low gear gains

        mMaster.selectProfileSlot(0, 0);

        mMaster.overrideLimitSwitchesEnable(true);
        mMaster.overrideSoftLimitsEnable(true);

        mMaster.enableVoltageCompensation(true);

        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,10, 20);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer,10, 20);

        mMaster.setInverted(true);
        mMaster.setSensorPhase(true);
        mMaster.setNeutralMode(NeutralMode.Brake);

        mRightSlave = TalonSRXFactory.createPermanentSlaveTalon(Constants.kElevatorRightSlaveId,
                Constants.kElevatorMasterId);
        mRightSlave.setInverted(true);
        mRightSlave.setNeutralMode(NeutralMode.Brake);

        mLeftSlaveA = TalonSRXFactory.createPermanentSlaveTalon(Constants.kElevatorLeftSlaveAId,
                Constants.kElevatorMasterId);
        mLeftSlaveA.setInverted(false);
        mLeftSlaveA.setNeutralMode(NeutralMode.Brake);

        mLeftSlaveB = TalonSRXFactory.createPermanentSlaveTalon(Constants.kElevatorLeftSlaveBId,
                Constants.kElevatorMasterId);
        mLeftSlaveB.setInverted(false);
        mLeftSlaveB.setNeutralMode(NeutralMode.Brake);

        mShifter = Constants.makeSolenoidForId(Constants.kElevatorShifterSolenoidId);

        // Start with zero power.
        mMaster.set(ControlMode.PercentOutput, 0);
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

    public synchronized void setHangMode(boolean hang_mode) {
        mShifter.set(hang_mode);
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
        SmartDashboard.putNumber("Elevator Output %", mMaster.getMotorOutputPercent());
        SmartDashboard.putNumber("Elevator RPM", getRPM());
        SmartDashboard.putNumber("Elevator Error", mMaster.getClosedLoopError(0) / kEncoderTicksPerInch);
        SmartDashboard.putNumber("Elevator Height", getInchesOffGround());
        SmartDashboard.putBoolean("Elevator Limit", mMaster.getSensorCollection().isFwdLimitSwitchClosed());
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    @Override
    public void zeroSensors() {
        mMaster.setSelectedSensorPosition(0,0, 10);
    }

    public synchronized void resetIfAtLimit() {
        if (mMaster.getSensorCollection().isFwdLimitSwitchClosed()) {
            zeroSensors();
        }
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
