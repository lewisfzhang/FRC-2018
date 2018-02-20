package com.team254.frc2018;

interface IControlBoard {
    double getThrottle();

    double getTurn();

    boolean getQuickTurn();

    boolean getLowGear();

    boolean getJogWristStow();

    boolean getJogWristExtend();

    boolean getScore();

    boolean getFarScore();

    boolean getIntake();

    boolean getBackwardsModifier();

    boolean getHighScale();

    boolean getSwitch();

    boolean getStow();

    boolean getArmIn();

    boolean getNeutralScale();

    boolean getJogElevatorUp();

    boolean getExchange();

    boolean getLowScale();

    boolean getJogElevatorDown();

    boolean getHangMode(); // TODO

    double getHangThrottle();  // TODO
}
