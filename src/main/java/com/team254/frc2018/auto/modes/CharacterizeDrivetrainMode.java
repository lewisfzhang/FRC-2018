package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.CollectDriveData;
import com.team254.frc2018.auto.actions.WaitAction;
import com.team254.lib.physics.DriveCharacterization;

import java.util.ArrayList;
import java.util.List;

public class CharacterizeDrivetrainMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        List<double[]> velocityData = new ArrayList<>();
        List<double[]> accelerationData = new ArrayList<>();

        runAction(new CollectDriveData(velocityData, 0.1, true));
        runAction(new WaitAction(2));
        runAction(new CollectDriveData(accelerationData, 0.5, true));

        DriveCharacterization.CharacterizationConstants constants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);
        System.out.println("ks: "+constants.ks);
        System.out.println("kv: "+constants.kv);
        System.out.println("ka: "+constants.ka);
    }
}
