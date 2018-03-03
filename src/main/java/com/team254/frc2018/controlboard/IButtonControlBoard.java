package com.team254.frc2018.controlboard;

public interface IButtonControlBoard {
    // Wrist
    boolean goToIntakingWrist();

    boolean goToScoringWrist();

    boolean goToVerticalWrist();

    boolean goToStowWrist();

    boolean goToScoringAngledWrist();

    //Elevator
    boolean getGoToHighScaleHeight();

    boolean getGoToNeutralScaleHeight();

    boolean getGoToLowScaleHeight();

    boolean getGoToSwitchHeight();

    boolean getGoToStowHeight();

    //Jog Elevator
    double getJogElevatorThrottle();

    //Jog Wrist
    double getJogWristThrottle();

    //Intake
    boolean getRunIntake();

    boolean getIntakePosition();

    void setRumble(boolean on);
}