package com.team254.frc2018.paths;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.spline.QuinticHermiteSpline;

import java.util.List;

/**
 * Interface containing all information necessary for a path including the Path itself, the Path's starting pose, and
 * whether or not the robot should drive in reverse along the path.
 */
public interface PathContainer {
    List<QuinticHermiteSpline> buildPath();

    Pose2d getStartPose();

    boolean isReversed();
}