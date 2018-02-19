package com.team254.lib.trajectory;

import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Translation2d;
import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertFalse;

public class IntegrationTest {

    @Test
    public void testFollowerTrajectoryGenerator() {
        // Specify desired waypoints.
        List<Translation2d> waypoints = Arrays.asList(
                new Translation2d(0.0, 0.0),
                new Translation2d(24.0, 0.0),
                new Translation2d(36.0, 0.0),
                new Translation2d(36.0, 24.0),
                new Translation2d(60.0, 24.0));

        // Create the reference trajectory (straight line motion between waypoints).
        Trajectory<Translation2d> reference_trajectory = new Trajectory<>(waypoints);

        // Generate a smooth (continuous curvature) path to follow.
        IPathFollower path_follower = new PurePursuitController<Translation2d>(
                new DistanceView<>(reference_trajectory), /* sampling_dist */1.0, /* lookahead= */ 6.0,
                /* goal_tolerance= */ 0.1);
        Trajectory<Pose2dWithCurvature> smooth_path = TrajectoryUtil.trajectoryFromPathFollower(path_follower,
                Pose2dWithCurvature.identity(), /* step_size= */ 1.0, /* dcurvature_limit= */1.0);

        assertFalse(smooth_path.isEmpty());
        System.out.println(smooth_path.toCSV());

        // Time parameterize the path subject to our dynamic constraints.
        // TODO
    }

    @Test
    public void testSplineTrajectoryGenerator() {
        // TODO
    }

}
