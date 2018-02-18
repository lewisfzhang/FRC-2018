package com.team254.lib.physics;

import com.team254.lib.util.Util;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class DriveCharacterizationTest {
    public static final double kTestEpsilon = Util.kEpsilon;

    @Test
    public void test() {
        List<double[]> velocityData = new ArrayList<>();
        velocityData.add(new double[]{0.0, 0.0});
        velocityData.add(new double[]{0.0, 0.1});
        velocityData.add(new double[]{0.4068646418, 0.15});
        velocityData.add(new double[]{0.580860176, 0.2});
        velocityData.add(new double[]{1.517387646, 0.25});
        velocityData.add(new double[]{1.488205486, 0.3});
        velocityData.add(new double[]{1.99721034, 0.35});
        velocityData.add(new double[]{2.418591372, 0.4});
        velocityData.add(new double[]{2.862789328, 0.45});
        velocityData.add(new double[]{3.325317897, 0.5});
        velocityData.add(new double[]{3.639369165, 0.55});
        velocityData.add(new double[]{3.751388775, 0.6});
        velocityData.add(new double[]{4.428653086, 0.65});
        velocityData.add(new double[]{4.419954156, 0.7});
        velocityData.add(new double[]{5.484034933, 0.75});
        velocityData.add(new double[]{5.944318249, 0.8});
        velocityData.add(new double[]{5.644578467, 0.85});
        velocityData.add(new double[]{6.068300422, 0.9});
        velocityData.add(new double[]{6.879827955, 0.95});
        velocityData.add(new double[]{6.740854678, 1.0});


        List<double[]> accelerationData = new ArrayList<>();
        accelerationData.add(new double[]{0.0, 0.0});
        accelerationData.add(new double[]{0.0, 0.1});
        accelerationData.add(new double[]{0.4068646418, 0.18});
        accelerationData.add(new double[]{0.580860176, 0.26});
        accelerationData.add(new double[]{1.517387646, 0.34});
        accelerationData.add(new double[]{1.488205486, 0.42});
        accelerationData.add(new double[]{1.99721034, 0.5});
        accelerationData.add(new double[]{2.418591372, 0.58});
        accelerationData.add(new double[]{2.862789328, 0.66});
        accelerationData.add(new double[]{3.325317897, 0.74});
        accelerationData.add(new double[]{3.639369165, 0.82});
        accelerationData.add(new double[]{3.751388775, 0.9});
        accelerationData.add(new double[]{4.428653086, 0.98});

        DriveCharacterization.CharacterizationConstants driveConstants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);

        assertEquals(driveConstants.ks, 0.09621205708274382, kTestEpsilon);
        assertEquals(driveConstants.kv, 0.12749072995763802, kTestEpsilon);
        assertEquals(driveConstants.ka, 0.07109838760407652, kTestEpsilon);
    }
}
