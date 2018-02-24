package com.team254.frc2018.controlboard;

public interface IButtonControlBoard {
    // Wrist
    boolean goToIntakingWrist();

    boolean goToScoringWrist();

    boolean goToVerticalWrist();

    boolean goToStowWrist();

    //Elevator
    boolean getGoToHighScaleHeight();

    boolean getGoToNeutralScaleHeight();

    boolean getGoToLowScaleHeight();

    boolean getGoToSwitchHeight();

    boolean getGoToStowHeight();

    //Jog Elevator
    boolean getJogElevatorUp();

    boolean getJogElevatorDown();

    //Jog Wrist
    boolean getJogWristBack();

    boolean getJogWristForward();

    //Intake
    boolean getRunIntake();

    boolean getIntakePosition();

    void setRumble(boolean on);
}