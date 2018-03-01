package com.team254.frc2018;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

import org.junit.jupiter.api.Test;


public class AutoFieldStateTest {

    @Test
    public void test() {
        Pose2d startPose = new Pose2d(1, 2, new Rotation2d(3, 4, true));
        AutoFieldState state = new AutoFieldState(startPose);
        assertEquals(startPose, state.getStartPose());
        
        startPose = new Pose2d(5, 6, new Rotation2d(7, 8, true));
        state.setStartPose(startPose);
        assertEquals(startPose, state.getStartPose());
        
        String[] badStates = new String[] {null, "dakjfhaksdjh", "", "foo", "LRQ"};
        for (String str : badStates) {
            state = new AutoFieldState();
            assertFalse(state.setSides(str));
            assertNull(state.getOurSwitchSide());
            assertNull(state.getScaleSide());
            assertNull(state.getOpponentSwitchSide());
        }
        
        String[] goodStates = new String[] {"RLR", "    RLR", "RLR    ", "    RLR    "};
        for (String str : goodStates) {
            state = new AutoFieldState();
            assertTrue(state.setSides(str));
            assertEquals(AutoFieldState.Side.RIGHT, state.getOurSwitchSide());
            assertEquals(AutoFieldState.Side.LEFT,  state.getScaleSide());
            assertEquals(AutoFieldState.Side.RIGHT, state.getOpponentSwitchSide());
        }
    }
}
