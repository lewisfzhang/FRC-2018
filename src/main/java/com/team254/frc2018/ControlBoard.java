package com.team254.frc2018;

import edu.wpi.first.wpilibj.Joystick;

public class ControlBoard implements IControlBoard {
    private static ControlBoard mInstance = null;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }
        return mInstance;
    }

    private final Joystick mThrottleStick;
    private final Joystick mTurnStick;
    private final Joystick mButtonBoard;

    private ControlBoard() {
        mThrottleStick = new Joystick(0);
        mTurnStick = new Joystick(1);
        mButtonBoard = new Joystick(2);
    }


    @Override
    public double getThrottle() {
        return -mThrottleStick.getRawAxis(0);
    }

    @Override
    public double getTurn() {
        return mTurnStick.getRawAxis(1);
    }

    @Override
    public boolean getQuickTurn() {
        return mTurnStick.getRawButton(1);
    }

    @Override
    public boolean getLowGear() {
        return !mThrottleStick.getRawButton(1);
    }

    @Override
    public boolean getJogWristStow() {
        return mButtonBoard.getRawButton(11);
    }

    @Override
    public boolean getJogWristExtend() {
        return mButtonBoard.getRawButton(12);
    }

    @Override
    public boolean getScore() {
        return mThrottleStick.getRawButton(2);
    }

    @Override
    public boolean getFarScore() {
        return mTurnStick.getRawButton(2);
    }

    @Override
    public boolean getIntake() {
        return mButtonBoard.getRawButton(10);
    }

    @Override
    public boolean getBackwardsModifier() {
        return mButtonBoard.getRawButton(9);
    }

    @Override
    public boolean getHighScale() {
        return mButtonBoard.getRawButton(3);
    }

    @Override
    public boolean getSwitch() {
        return mButtonBoard.getRawAxis(2) < -0.1;
    }

    @Override
    public boolean getNeutralScale() {
        return mButtonBoard.getRawButton(5);
    }

    @Override
    public boolean getStow() {
        return mButtonBoard.getRawAxis(0) < -0.1;
    }

    @Override
    public boolean getArmIn() {
        return mButtonBoard.getRawButton(6);
    }

    @Override
    public boolean getJogElevatorUp() {
        return mButtonBoard.getRawButton(1);
    }

    @Override
    public boolean getExchange() {
        return mButtonBoard.getRawAxis(2) < -0.1;
    }

    @Override
    public boolean getLowScale() {
        return mButtonBoard.getRawAxis(1) < -0.1;
    }

    @Override
    public boolean getJogElevatorDown() {
        return mButtonBoard.getRawButton(2);
    }

    @Override
    public boolean getHangMode() { return mButtonBoard.getRawButton(4); }

    @Override
    public double getHangThrottle() { return mTurnStick.getRawAxis(0); }
}
