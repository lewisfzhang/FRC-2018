package com.team254.frc2018;

interface IControlBoard {
    double getThrottle();

    double getTurn();

    boolean getQuickTurn();

    boolean getJogWristBack();

    boolean getJogWristForward();

    boolean getOpenJaw();

    boolean getShoot();

    boolean getRunIntake();

    boolean getGoToStowHeight();

    boolean getGoToSwitchHeight();

    boolean getGoToLowScaleHeight();

    boolean getGoToNeutralScaleHeight();

    boolean getGoToHighScaleHeight();

    boolean goToIntakingWrist();

    boolean goToScoringWrist();

    boolean goToVerticalWrist();

    boolean goToStowWrist();

    boolean getJogElevatorUp();

    boolean getJogElevatorDown();

    boolean getHangMode(); // TODO

    double getHangThrottle();  // TODO
}
