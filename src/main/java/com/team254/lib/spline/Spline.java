package com.team254.lib.spline;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public interface Spline {
    public Translation2d getPoint(double t);

    public Rotation2d getHeading(double t);

    public String toString();
}
