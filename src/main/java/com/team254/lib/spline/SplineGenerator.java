package com.team254.lib.spline;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;

import java.util.ArrayList;
import java.util.List;

public class SplineGenerator {
    private static final double kMaxDX = 2.0; //inches
    private static final double kMaxDY = 0.05; //inches
    private static final double kMaxDTheta = 0.1; //radians!
    private static final int kMinSampleSize = 1;

    /**
     * Converts a spline into a list of Twist2d's.
     *
     * @param s  the spline to parametrize
     * @param t0 starting percentage of spline to parametrize
     * @param t1 ending percentage of spline to parametrize
     * @return list of twist2ds that approximates the original spline
     */
    public static List<Twist2d> parametrizeSpline(Spline s, double t0, double t1) {
        List<Twist2d> rv = new ArrayList<>();
        double dt = (t1 - t0);
        for (double t = 0; t < t1; t += dt / kMinSampleSize) {
            getSegmentArc(s, rv, t, t + dt / kMinSampleSize);
        }
        return rv;
    }

    /**
     * Convenience function to parametrize a spline from t 0 to 1
     */
    public static List<Twist2d> parametrizeSpline(Spline s) {
        return parametrizeSpline(s, 0.0, 1.0);
    }

    private static void getSegmentArc(Spline s, List<Twist2d> rv, double t0, double t1) {
        Translation2d p0 = s.getPoint(t0);
        Translation2d p1 = s.getPoint(t1);
        Rotation2d r0 = s.getHeading(t0);
        Rotation2d r1 = s.getHeading(t1);
        Pose2d transformation = new Pose2d(new Translation2d(p0, p1).rotateBy(r0.inverse()), r1.rotateBy(r0.inverse()));
        Twist2d twist = Pose2d.log(transformation);
        if (twist.dy > kMaxDY || twist.dx > kMaxDX || twist.dtheta > kMaxDTheta) {
            getSegmentArc(s, rv, t0, (t0 + t1) / 2);
            getSegmentArc(s, rv, (t0 + t1) / 2, t1);
        } else {
            rv.add(twist);
        }
    }

}
