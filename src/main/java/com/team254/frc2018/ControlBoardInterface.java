package com.team254.frc2018;

/**
 * A basic framework for robot controls that other controller classes implement
 */
public interface ControlBoardInterface {
    // DRIVER CONTROLS
    double getThrottle();

    double getTurn();

    boolean getQuickTurn();

    boolean getLowGear();

    boolean getAimButton();

    boolean getDriveAimButton();

    // OPERATOR CONTROLS
    boolean getLeftFrontIntake();

    boolean getLeftBackIntake();

    boolean getRightFrontIntake();

    boolean getRightBackIntake();

    boolean getIncreaseVoltage();

    boolean getDecreaseVoltage();

    boolean getOverride();

}
