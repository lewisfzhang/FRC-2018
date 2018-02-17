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
    public final Joystick mButtonBoard;

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
        return mThrottleStick.getRawButton(2);
    }

    @Override
    public boolean getIntakeTest() { return mButtonBoard.getRawButton(11); }

    @Override
    public boolean getReverseIntakeTest() { return mButtonBoard.getRawButton(12); }

    public boolean getTestWristNegative() {
        return mButtonBoard.getRawButton(7);
    }

    public boolean getTestWristPositive() {
        return mButtonBoard.getRawButton(8);
    }

    @Override
    public boolean getJogElevatorUp() {
        return mButtonBoard.getRawButton(1);
    }

    @Override
    public boolean getJogElevatorDown() {
        return mButtonBoard.getRawButton(2);
    }
}
