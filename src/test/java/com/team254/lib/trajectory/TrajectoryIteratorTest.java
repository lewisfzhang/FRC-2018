package com.team254.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;
import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.List;

public class TrajectoryIteratorTest {
    public static final double kTestEpsilon = Util.kEpsilon;

    public static final List<Translation2d> kWaypoints = Arrays.asList(
            new Translation2d(0.0, 0.0),
            new Translation2d(24.0, 0.0),
            new Translation2d(36.0, 12.0),
            new Translation2d(60.0, 12.0));

    @Test
    public void test() {
        Trajectory<Translation2d> traj = new Trajectory<>(kWaypoints);
        TrajectoryIterator<Translation2d> iterator = new TrajectoryIterator<>(traj.getIndexView());

        // Initial conditions.
        assertEquals(0.0, iterator.getProgress(), kTestEpsilon);
        assertEquals(3.0, iterator.getRemainingProgress(), kTestEpsilon);
        assertTrue(iterator.getState().isEqual(kWaypoints.get(0)));
        assertFalse(iterator.isDone());

        // Advance forward.
        assertTrue(iterator.preview(0.5).state().isEqual(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.5)));
        assertTrue(iterator.advance(0.5).state().isEqual(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.5)));
        assertEquals(0.5, iterator.getProgress(), kTestEpsilon);
        assertEquals(2.5, iterator.getRemainingProgress(), kTestEpsilon);
        assertFalse(iterator.isDone());

        // Advance backwards.
        assertTrue(iterator.preview(-0.25).state().isEqual(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.25)));
        assertTrue(iterator.advance(-0.25).state().isEqual(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.25)));
        assertEquals(0.25, iterator.getProgress(), kTestEpsilon);
        assertEquals(2.75, iterator.getRemainingProgress(), kTestEpsilon);
        assertFalse(iterator.isDone());

        // Advance past end.
        assertTrue(iterator.preview(5.0).state().isEqual(kWaypoints.get(3)));
        assertTrue(iterator.advance(5.0).state().isEqual(kWaypoints.get(3)));
        assertEquals(3.0, iterator.getProgress(), kTestEpsilon);
        assertEquals(0.0, iterator.getRemainingProgress(), kTestEpsilon);
        assertTrue(iterator.isDone());

        // Advance past beginning.
        assertTrue(iterator.preview(-5.0).state().isEqual(kWaypoints.get(0)));
        assertTrue(iterator.advance(-5.0).state().isEqual(kWaypoints.get(0)));
        assertEquals(0.0, iterator.getProgress(), kTestEpsilon);
        assertEquals(3.0, iterator.getRemainingProgress(), kTestEpsilon);
        assertFalse(iterator.isDone());
    }

}
