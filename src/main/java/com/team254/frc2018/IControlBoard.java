package com.team254.frc2018;

public interface IControlBoard {
    double getThrottle();

    double getTurn();

    boolean getQuickTurn();

    boolean getLowGear();

    boolean getIntakeTest();

    boolean getReverseIntakeTest();

    boolean getTestWristUp();

    boolean getTestWristDown();
}
