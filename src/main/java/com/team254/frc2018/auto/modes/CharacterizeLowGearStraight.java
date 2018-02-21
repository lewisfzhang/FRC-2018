package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.CollectDriveData;
import com.team254.frc2018.auto.actions.WaitAction;
import com.team254.lib.physics.DriveCharacterization;

import java.util.ArrayList;
import java.util.List;

public class CharacterizeLowGearStraight extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        List<double[]> velocityData = new ArrayList<>();
        List<double[]> accelerationData = new ArrayList<>();

        runAction(new CollectDriveData(velocityData, 0.02, 0.25, false, false, false));
        runAction(new WaitAction(2));
        runAction(new CollectDriveData(velocityData, 0.5, 0.75, false, false, false));

        DriveCharacterization.CharacterizationConstants constants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);

        System.out.println("ks: "+constants.ks);
        System.out.println("kv: "+constants.kv);
        System.out.println("ka: "+constants.ka);
    }
}
