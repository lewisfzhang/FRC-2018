package com.team254.lib.spline;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;


public class QuinticHermiteOptimizerTest {

    @Test
    public void test() {
        Pose2d a = new Pose2d(new Translation2d(0, 100), Rotation2d.fromDegrees(270));
        Pose2d b = new Pose2d(new Translation2d(50, 0), Rotation2d.fromDegrees(0));
        Pose2d c = new Pose2d(new Translation2d(100, 100), Rotation2d.fromDegrees(90));

        List<QuinticHermiteSpline> splines = new ArrayList<>();
        splines.add(new QuinticHermiteSpline(a, b));
        splines.add(new QuinticHermiteSpline(b, c));

        assertTrue(QuinticHermiteSpline.optimizeSpline(splines) < 0.014);

        Pose2d d = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(90));
        Pose2d e = new Pose2d(new Translation2d(0, 50), Rotation2d.fromDegrees(0));
        Pose2d f = new Pose2d(new Translation2d(100, 0), Rotation2d.fromDegrees(90));
        Pose2d g = new Pose2d(new Translation2d(100, 100), Rotation2d.fromDegrees(0));

        List<QuinticHermiteSpline> splines1 = new ArrayList<>();
        splines1.add(new QuinticHermiteSpline(d, e));
        splines1.add(new QuinticHermiteSpline(e, f));
        splines1.add(new QuinticHermiteSpline(f, g));

        assertTrue(QuinticHermiteSpline.optimizeSpline(splines1) < 0.16);
    }
}
