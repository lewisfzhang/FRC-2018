package com.team254.frc2018.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2018.Constants;
import com.team254.frc2018.loops.Looper;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.drivers.TalonSRXChecker;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

// Top Soft limit : -151288
public class Elevator extends Subsystem {
    private static Elevator mInstance = null;

    private int kNegativeSoftLimit = -130000; // -151000
    private double kPositiveSoftLimit = 0;

    public synchronized static Elevator getInstance() {
        if (mInstance == null) {
            mInstance = new Elevator();
        }
        return mInstance;
    }

    private final TalonSRX mMaster, mRightSlave, mLeftSlaveA, mLeftSlaveB;

    private Elevator() {
        mMaster = TalonSRXFactory.createDefaultTalon(Constants.kElevatorMasterId);
        ErrorCode sensorPresent =
                mMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                        0, 100);
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect elevator encoder: " +
                    sensorPresent, false);
        }
        mMaster.setInverted(false);
        mMaster.setSensorPhase(true);
        mMaster.setNeutralMode(NeutralMode.Brake);

        // SET UP BOTTOM LIMIT SWITCH
        ErrorCode errorCode = mMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal
                .NormallyOpen, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not detect forward limit switch wrist: " + errorCode, false);

        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,10, 10);

        // Re-zero on limit
        // TODO: determine if we want this on?
        // mMaster.configSetParameter(ParamEnum.eClearPositionOnLimitF, 1, 0, 0, 0);

        // Soft limits
        mMaster.configReverseSoftLimitThreshold(kNegativeSoftLimit, 10);
        mMaster.configReverseSoftLimitEnable(true, 10);

        // Enable limts
        mMaster.overrideLimitSwitchesEnable(true);
        mMaster.overrideSoftLimitsEnable(true);

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

        // Set up gains
        mMaster.selectProfileSlot(0, 0);
        mMaster.config_kP(0, Constants.kElevatorHighGearKp, 100);
        mMaster.config_kI(0, Constants.kElevatorHighGearKi, 100);
        mMaster.config_kD(0, Constants.kElevatorHighGearKd, 100);
        mMaster.config_kF(0, Constants.kElevatorHighGearKf, 100);

        mMaster.configMotionCruiseVelocity(11250, 100); // TODO: tune
        mMaster.configMotionAcceleration(40000, 100); //todo: tune

        mMaster.config_IntegralZone(0, Constants.kElevatorHighGearIZone, 100);

    }

    public synchronized void setOpenLoop(double percentage) {
        mMaster.set(ControlMode.PercentOutput, percentage);
    }

    public synchronized void setClosedLoopPosition(double position) {
        setClosedLoopRawPosition(position);
    }

    private synchronized void setClosedLoopRawPosition(double position) {
        System.out.println("Going for: " + position);
        mMaster.set(ControlMode.MotionMagic, position);
    }

    public double getRPM() {
        // We are using a CTRE mag encoder which is 4096 native units per revolution.
        // GetVelocity is in native units per 100ms.
        // TODO: make this rpm again
        return mMaster.getSelectedSensorVelocity(0); // * 10.0 / 4096.0 * 60.0;
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
