package com.team254.frc2018.controlboard;

public interface IDriveControlBoard {
    double getThrottle();

    double getTurn();

    boolean getAutoExchange();

    boolean getQuickTurn();

    boolean getOpenJaw();

    boolean getShoot();
}