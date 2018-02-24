package com.team254.frc2018;

import com.team254.frc2018.controlboard.*;

public class ControlBoard implements IControlBoard {
    private static ControlBoard mInstance = null;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }
        return mInstance;
    }

    private IDriveControlBoard mDriveControlBoard;
    private IButtonControlBoard mButtonControlBoard;

    private ControlBoard() {
        if (Constants.kUseGamepadForDriving) {
            mDriveControlBoard = GamepadDriveControlBoard.getInstance();
        } else {
            mDriveControlBoard = MainDriveControlBoard.getInstance();
        }

        if (Constants.kUseGamepadForButtons) {
            mButtonControlBoard = GamepadButtonControlBoard.getInstance();
        } else {
            mButtonControlBoard = MainButtonBoard.getInstance();
        }
    }

    @Override
    public double getThrottle() {
        return mDriveControlBoard.getThrottle();
    }

    @Override
    public double getTurn() {
        return mDriveControlBoard.getTurn();
    }

    @Override
    public boolean getQuickTurn() {
        return mDriveControlBoard.getQuickTurn();
    }

    @Override
    public boolean getOpenJaw() {
        return mDriveControlBoard.getOpenJaw();
    }

    @Override
    public boolean getShoot() {
        return mDriveControlBoard.getShoot();
    }

    @Override
    public boolean goToIntakingWrist() {
        return mButtonControlBoard.goToIntakingWrist();
    }

    @Override
    public boolean goToScoringWrist() {
        return mButtonControlBoard.goToScoringWrist();
    }

    @Override
    public boolean goToVerticalWrist() {
        return mButtonControlBoard.goToVerticalWrist();
    }

    @Override
    public boolean goToStowWrist() {
        return mButtonControlBoard.goToStowWrist();
    }

    @Override
    public boolean getGoToHighScaleHeight() {
        return mButtonControlBoard.getGoToHighScaleHeight();
    }

    @Override
    public boolean getGoToNeutralScaleHeight() {
        return mButtonControlBoard.getGoToNeutralScaleHeight();
    }

    @Override
    public boolean getGoToLowScaleHeight() {
        return mButtonControlBoard.getGoToLowScaleHeight();
    }

    @Override
    public boolean getGoToSwitchHeight() {
        return mButtonControlBoard.getGoToSwitchHeight();
    }

    @Override
    public boolean getGoToStowHeight() {
        return mButtonControlBoard.getGoToStowHeight();
    }

    @Override
    public boolean getJogElevatorUp() {
        return mButtonControlBoard.getJogElevatorUp();
    }

    @Override
    public boolean getJogElevatorDown() {
        return mButtonControlBoard.getJogElevatorDown();
    }

    @Override
    public boolean getJogWristBack() {
        return mButtonControlBoard.getJogWristBack();
    }

    @Override
    public boolean getJogWristForward() {
        return mButtonControlBoard.getJogWristForward();
    }

    @Override
    public boolean getIntakePosition() {
        return mButtonControlBoard.getIntakePosition();
    }

    @Override
    public boolean getRunIntake() {
        return mButtonControlBoard.getRunIntake();
    }

    @Override
    public boolean getHangMode() {
        return false;
    }

    @Override
    public double getHangThrottle() {
        return 0.0;
    }

    public void setRumble(boolean on) {
        mButtonControlBoard.setRumble(on);
    }
}
