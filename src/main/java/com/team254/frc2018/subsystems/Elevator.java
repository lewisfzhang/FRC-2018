package com.team254.frc2018.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2018.Constants;
import com.team254.frc2018.loops.Looper;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.util.TalonSRXChecker;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Elevator extends Subsystem {
    private static Elevator mInstance = null;

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
        mMaster.setSensorPhase(false);

        mRightSlave = TalonSRXFactory.createPermanentSlaveTalon(Constants.kElevatorRightSlaveId,
                Constants.kElevatorMasterId);
        mRightSlave.setInverted(false);

        mLeftSlaveA = TalonSRXFactory.createPermanentSlaveTalon(Constants.kElevatorLeftSlaveAId,
                Constants.kElevatorMasterId);
        mLeftSlaveA.setInverted(true);

        mLeftSlaveB = TalonSRXFactory.createPermanentSlaveTalon(Constants.kElevatorLeftSlaveBId,
                Constants.kElevatorMasterId);
        mLeftSlaveB.setInverted(true);
    }

    public synchronized void setOpenLoop(double percentage) {
        mMaster.set(ControlMode.PercentOutput, percentage);
    }

    public synchronized void setClosedLoop(double position) {
        mMaster.set(ControlMode.MotionMagic, position);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Elevator Master Output", mMaster.getMotorOutputPercent()) ;
        SmartDashboard.putNumber("Elevator Master Velocity", mMaster.getSelectedSensorVelocity(0));
        SmartDashboard.putNumber("Elevator Master Position" ,
                mMaster.getSelectedSensorPosition(0))     ;
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    @Override
    public void zeroSensors() {}

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {}

    @Override
    public boolean checkSystem() {
        boolean rightSide =
                TalonSRXChecker.CheckTalons(this,
                        new ArrayList<TalonSRXChecker.TalonSRXConfig>(){
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
        boolean leftSide =  TalonSRXChecker.CheckTalons(this,
                new ArrayList<TalonSRXChecker.TalonSRXConfig>(){
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
