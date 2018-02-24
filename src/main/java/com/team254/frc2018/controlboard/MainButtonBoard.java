package com.team254.frc2018.controlboard;

import com.team254.frc2018.Constants;
import edu.wpi.first.wpilibj.Joystick;

public class MainButtonBoard implements IButtonControlBoard {
    private static MainButtonBoard mInstance = null;

    public static MainButtonBoard getInstance() {
        if (mInstance == null) {
            mInstance = new MainButtonBoard();
        }
        return mInstance;
    }

    private final Joystick mButtonBoard;

    private MainButtonBoard() {
        mButtonBoard = new Joystick(2);
    }

    //Wrist
    @Override
    public boolean goToIntakingWrist() {
        return mButtonBoard.getRawAxis(0) < Constants.kJoystickThreshold;
    }

    @Override
    public boolean goToScoringWrist() {
        return mButtonBoard.getRawButton(7);
    }

    @Override
    public boolean goToVerticalWrist() {
        return mButtonBoard.getRawAxis(1) < Constants.kJoystickThreshold;
    }

    @Override
    public boolean goToStowWrist() {
        return mButtonBoard.getRawAxis(2) < Constants.kJoystickThreshold;
    }

    //Elevator
    @Override
    public boolean getGoToHighScaleHeight() {
        return mButtonBoard.getRawButton(6);
    }

    @Override
    public boolean getGoToNeutralScaleHeight() {
        return mButtonBoard.getRawButton(5);
    }

    @Override
    public boolean getGoToLowScaleHeight() {
        return mButtonBoard.getRawButton(4);
    }

    @Override
    public boolean getGoToSwitchHeight() {
        return mButtonBoard.getRawButton(3);
    }

    @Override
    public boolean getGoToStowHeight() {
        return mButtonBoard.getRawButton(10);
    }

    //Jog Elevator
    @Override
    public boolean getJogElevatorUp() {
    		return mButtonBoard.getRawButton(1);
    }

    @Override
    public boolean getJogElevatorDown() {
        return mButtonBoard.getRawButton(2);
    }

    //Jog Wrist
    @Override
    public boolean getJogWristBack() {
        return mButtonBoard.getRawButton(11);
    }

    @Override
    public boolean getJogWristForward() {
        return mButtonBoard.getRawButton(12);
    }

    //Intake
    @Override
    public boolean getRunIntake() {
        return mButtonBoard.getRawButton(9);
    }
}