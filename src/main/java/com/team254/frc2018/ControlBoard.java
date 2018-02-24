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
    public boolean getJogWristBack() {
        return mButtonBoard.getRawButton(11);
    }

    @Override
    public boolean getJogWristForward() {
        return mButtonBoard.getRawButton(12);
    }

    @Override
    public boolean getOpenJaw() {
        return mThrottleStick.getRawButton(2);
    }

    @Override
    public boolean getShoot() {
        return mTurnStick.getRawButton(2);
    }

    @Override
    public boolean getRunIntake() {
        return mButtonBoard.getRawButton(9);
    }

    @Override
    public boolean getGoToStowHeight() {
        return mButtonBoard.getRawButton(10);
    }

    @Override
    public boolean getGoToSwitchHeight() {
        return mButtonBoard.getRawButton(3);
    }

    @Override
    public boolean getGoToLowScaleHeight() {
        return mButtonBoard.getRawButton(4);
    }

    @Override
    public boolean getGoToNeutralScaleHeight() {
        return mButtonBoard.getRawButton(5);
    }

    @Override
    public boolean getGoToHighScaleHeight() {
        return mButtonBoard.getRawButton(6);
    }

    @Override
    public boolean goToIntakingWrist() {
        return mButtonBoard.getRawAxis(0) < 0.1;
    }

    @Override
    public boolean goToScoringWrist() {
        return mButtonBoard.getRawAxis(2) < 0.1;
    }

    @Override
    public boolean goToVerticalWrist() {
        return mButtonBoard.getRawAxis(1) < 0.1;
    }

    @Override
    public boolean goToStowWrist() {
        return false;
    }

    @Override
   public boolean getJogElevatorUp() {
        return mButtonBoard.getRawButton(1);
    }


    @Override
    public boolean getJogElevatorDown() {
        return mButtonBoard.getRawButton(2);
    }

    @Override
    public boolean getHangMode() {
        return false;
    }

    @Override
    public double getHangThrottle() {
        return 0.0;
    }
}
