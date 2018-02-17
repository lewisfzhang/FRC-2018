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

    boolean getExchangeIntake();

    boolean getBackwardsModifier();

    boolean getHighScale();

    boolean getSwitch();

    boolean getNeutralScale();

    boolean getJogElevatorUp();

    boolean getExchange();

    boolean getLowScale();

    boolean getJogElevatorDown();

    /**
     * @return switch position-> false is left, true is right
     */
    boolean getClimbMode();
}
