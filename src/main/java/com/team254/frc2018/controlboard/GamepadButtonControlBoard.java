package com.team254.frc2018.controlboard;

import com.team254.frc2018.Constants;
import edu.wpi.first.wpilibj.Joystick;

public class GamepadButtonControlBoard implements IButtonControlBoard {
    private static GamepadButtonControlBoard mInstance = null;

    public static GamepadButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadButtonControlBoard();
        }

        return mInstance;
    }

    private Joystick mJoystick;

    private GamepadButtonControlBoard() {
        mJoystick = new Joystick(Constants.kButtonGamepadPort);
    }

    //Wrist
    @Override
    public boolean goToIntakingWrist() {
        return mJoystick.getPOV() == 90;
    }
    
    @Override
    public boolean goToScoringWrist() {
        return mJoystick.getPOV() == 90;
    }
    
    @Override
    public boolean goToVerticalWrist() {
    		return mJoystick.getPOV() == 0;
    }

    @Override
    public boolean goToStowWrist() {
        return mJoystick.getPOV() == 180;
    }

    //Elevator
    @Override
    public boolean getGoToHighScaleHeight() {
        return mJoystick.getRawButton(4);
    }

    @Override
    public boolean getGoToNeutralScaleHeight() {
        return mJoystick.getRawButton(2);
    }

    @Override
    public boolean getGoToLowScaleHeight() {
        return mJoystick.getRawButton(1);
    }

    @Override
    public boolean getGoToSwitchHeight() {
        return mJoystick.getRawButton(3);
    }

    @Override
    public boolean getGoToStowHeight() {
        return mJoystick.getRawButton(6);
    }

    //Jog Elevator
    @Override
    public boolean getJogElevatorUp() {
        return -mJoystick.getRawAxis(5) > Constants.kJoystickThreshold;
    }

    @Override
    public boolean getJogElevatorDown() {
        return -mJoystick.getRawAxis(5) < -Constants.kJoystickThreshold;
    }

    //Jog Wrist
    @Override
    public boolean getJogWristBack() {
        return mJoystick.getRawAxis(0) < -Constants.kJoystickThreshold;
    }

    @Override
    public boolean getJogWristForward() {
        return mJoystick.getRawAxis(0) > Constants.kJoystickThreshold;
    }

    //Intake
    @Override
    public boolean getRunIntake() {
        return mJoystick.getRawAxis(2) != 0;
    }
}