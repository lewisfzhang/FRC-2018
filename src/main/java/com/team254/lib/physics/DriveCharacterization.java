package com.team254.lib.physics;

import com.team254.lib.util.PolynomialRegression;
import com.team254.lib.util.Util;

import java.util.List;

public class DriveCharacterization {
    public static class CharacterizationConstants {
        public double ks; //voltage needed to break static friction
        public double kv;
        public double ka;
    }

    public static CharacterizationConstants characterizeDrive(List<double[]> velocityData, List<double[]> accelerationData) {
        CharacterizationConstants rv = getVelocityCharacterization(trimData(velocityData));
        getAccelerationCharacterization(trimData(accelerationData), rv);
        return rv;
    }

    private static CharacterizationConstants getVelocityCharacterization(double[][] points) {
        CharacterizationConstants constants = new CharacterizationConstants();
        if(points == null) {
            return constants;
        }
        PolynomialRegression p = new PolynomialRegression(points, 1);
        System.out.println("r^2: " + p.R2());
        constants.ks = p.beta(0);
        constants.kv = p.beta(1);
        return constants;
    }

    private static CharacterizationConstants getAccelerationCharacterization(double[][] points, CharacterizationConstants velocityChacterization) {
        if(points == null) {
            return velocityChacterization;
        }
        PolynomialRegression p = new PolynomialRegression(points, 1);
        System.out.println("r^2: " + p.R2());
        velocityChacterization.ka = p.beta(1) - velocityChacterization.kv;
        return velocityChacterization;
    }

    /**
     * removes data points with a velocity of zero to get a better line fit
     */
    private static double[][] trimData(List<double[]> input) {
        double[][] output = null;
        int startTrim = 0;
        for(int i = 0; i < input.size(); ++i) {
            if(input.get(i)[0] > Util.kEpsilon) {
                if(output == null) {
                    output = new double[input.size() - i][2];
                    startTrim = i;
                }
                output[i - startTrim] = input.get(i);
            }
        }
        return output;
    }
}
