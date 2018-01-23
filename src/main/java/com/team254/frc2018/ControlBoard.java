package com.team254.frc2018;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Contains the button mappings for the competition control board. Like the drive code, one instance of the ControlBoard
 * object is created upon startup, then other methods request the singleton ControlBoard instance. Implements the
 * ControlBoardInterface.

 */
public class ControlBoard implements ControlBoardInterface {
    private static ControlBoardInterface mInstance = null;

    public static ControlBoardInterface getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }
        return mInstance;
    }

    private final Joystick mThrottleStick;
    private final Joystick mTurnStick;
    private final Joystick mButtonBoard;

    protected ControlBoard() {
        mThrottleStick = new Joystick(0);
        mTurnStick = new Joystick(1);
        mButtonBoard = new Joystick(2);
    }

    // DRIVER CONTROLS
    @Override
    public double getThrottle() {
        return -mThrottleStick.getRawAxis(0);
    }

    @Override
    public double getTurn() {
        return -mTurnStick.getY();
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
    public boolean getAimButton() {
        return mTurnStick.getRawButton(2);
    }

    public boolean getDriveAimButton() {
        return mThrottleStick.getRawButton(1);
    }

    // OPERATOR CONTROLS
    @Override
    public boolean getLeftFrontIntake() {
        return mButtonBoard.getRawButtonReleased(8);
    }

    @Override
    public boolean getLeftBackIntake() {
        return mButtonBoard.getRawButtonReleased(10);
    }

    @Override
    public boolean getRightFrontIntake() {
        return mButtonBoard.getRawButtonReleased(7);
    }

    @Override
    public boolean getRightBackIntake() {
        return mButtonBoard.getRawButtonReleased(9);
    }

    @Override
    public boolean getIncreaseVoltage() {
        return mButtonBoard.getRawButtonReleased(1);
    }

    @Override
    public boolean getDecreaseVoltage() {
        return mButtonBoard.getRawButtonReleased(2);
    }

    @Override
    public boolean getOverride() {
        return mButtonBoard.getRawAxis(3) > 0.5;
    }
}
