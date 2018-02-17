package com.team254.lib.spline;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.Util;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class SplineGeneratorTest {
    public static final double kTestEpsilon = Util.kEpsilon;

    @Test
    public void test() {
        // Create the test spline
        Pose2d p1 = new Pose2d(new Translation2d(0, 0), new Rotation2d());
        Pose2d p2 = new Pose2d(new Translation2d(15, 10), new Rotation2d(1, -5, true));
        Spline s = new CubicHermiteSpline(p1, p2);

        List<Twist2d> arcs = SplineGenerator.parameterizeSpline(s);

        double arclength = 0;
        Pose2d finalPose = new Pose2d(p1);
        for (Twist2d t : arcs) {
            finalPose = finalPose.transformBy(Pose2d.exp(t));
            arclength += t.dx;
        }

        assertEquals(finalPose.getTranslation().x(), 15.0, kTestEpsilon);
        assertEquals(finalPose.getTranslation().y(), 10.000000000001135, kTestEpsilon);
        assertEquals(finalPose.getRotation().getDegrees(), -78.69006752597981, kTestEpsilon);
        assertEquals(arclength, 24.136686154402014, kTestEpsilon);
    }
}
