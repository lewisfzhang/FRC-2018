package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;

public class TestMode extends AutoModeBase{
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Test mode");
    }
}
