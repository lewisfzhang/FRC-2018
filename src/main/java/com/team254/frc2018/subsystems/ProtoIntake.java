package com.team254.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2018.ControlBoard;
import com.team254.frc2018.ControlBoardInterface;
import com.team254.frc2018.loops.Loop;
import com.team254.frc2018.loops.Looper;

public class ProtoIntake extends Subsystem {

    private static ProtoIntake mInstance = new ProtoIntake();
    private ControlBoardInterface mControlBoard = ControlBoard.getInstance();

    public static ProtoIntake getInstance() {
        return mInstance;
    }

    private final TalonSRX leftFront, leftBack, rightFront, rightBack;

    private double voltage = 0.2;
    private int LF, LB, RF, RB;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (ProtoIntake.this) {

            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (ProtoIntake.this) {
                if(mControlBoard.getIncreaseVoltage()) {
                    voltage += 0.1;
                    System.out.println("Power (out of 1) = " + voltage);
                }

                if(mControlBoard.getDecreaseVoltage()) {
                    voltage -= 0.1;
                    System.out.println("Power (out of 1) = " + voltage);
                }

                if(mControlBoard.getLeftFrontIntake()) {
                    LF++;
                    System.out.println((LF%3 - 1));
                }
                if(mControlBoard.getLeftBackIntake()) LB++;
                if(mControlBoard.getRightFrontIntake()) RF++;
                if(mControlBoard.getRightBackIntake()) RB++;

                if(mControlBoard.getOverride()) {
                    leftFront.set(ControlMode.PercentOutput, 0);
                    leftBack.set(ControlMode.PercentOutput, 0);
                    rightFront.set(ControlMode.PercentOutput, 0);
                    rightBack.set(ControlMode.PercentOutput, 0);
                } else {
                    leftFront.set(ControlMode.PercentOutput, voltage * (LF % 3 - 1));
                    leftBack.set(ControlMode.PercentOutput, voltage * (LB % 3 - 1));
                    rightFront.set(ControlMode.PercentOutput, voltage * (RF % 3 - 1));
                    rightBack.set(ControlMode.PercentOutput, voltage * (RB % 3 - 1));
                }
            }
        }

        @Override
        public void onStop(double timestamp) {

        }
    };

    private ProtoIntake() {
        LF = LB = RF = RB = 1;
        leftFront = new TalonSRX(4);
        leftBack = new TalonSRX(3);

        rightFront = new TalonSRX(11);
        rightFront.setInverted(true);
        rightBack = new TalonSRX(12);
        rightBack.setInverted(true);
    }

    public void stop() {}

    public void zeroSensors() {}

    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    public void outputToSmartDashboard() {}
}
