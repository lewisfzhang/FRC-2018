package com.team254.lib.spline;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class CubicHermiteSpline implements Spline {
    double x0, x1, dx0, dx1, y0, y1, dy0, dy1;
    double ax, bx, cx, dx, ay, by, cy, dy;

    public CubicHermiteSpline(Pose2d p0, Pose2d p1) {
        double scale = 2 * p0.getTranslation().distance(p1.getTranslation());
        x0 = p0.getTranslation().x();
        x1 = p1.getTranslation().x();
        dx0 = p0.getRotation().cos() * scale;
        dx1 = p1.getRotation().cos()  * scale;
        y0 = p0.getTranslation().y();
        y1 = p1.getTranslation().y();
        dy0 = p0.getRotation().sin() * scale;
        dy1 = p1.getRotation().sin() * scale;
        ax = dx0 + dx1 + 2*x0 - 2*x1;
        bx = -2*dx0 - dx1 - 3*x0 + 3*x1;
        cx = dx0;
        dx = x0;
        ay = dy0 + dy1 + 2*y0 - 2*y1;
        by = -2*dy0 - dy1 - 3*y0 + 3*y1;
        cy = dy0;
        dy = y0;
    }

    @Override
    public Translation2d getPoint(double t) {
        double x = t*t*t * ax + t*t * bx + t * cx + dx;
        double y = t*t*t * ay + t*t * by + t * cy + dy;

        return new Translation2d(x, y);
    }

    @Override
    public Rotation2d getHeading(double t) {
        return null;
    }

    @Override
    public String toString() {
        return "x0: " + x0 + ", y0: " + y0 + ", x1: " + x1 + ", y1: " + y1 + ", dx0: " + dx0 + ", dy0: " + dy0 + ", dx1: " + dx1 + ", dy1: " + dy1;
    }
}
