package com.team254.lib.spline;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public interface Spline {
    Translation2d getPoint(double t);

    Rotation2d getHeading(double t);
}
