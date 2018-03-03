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
        AutoFieldState.setStartPose(startPose);
        assertEquals(startPose, AutoFieldState.getStartPose());
        
        startPose = new Pose2d(5, 6, new Rotation2d(7, 8, true));
        AutoFieldState.setStartPose(startPose);
        assertEquals(startPose, AutoFieldState.getStartPose());
        
        String[] badStates = new String[] {null, "dakjfhaksdjh", "", "foo", "LRQ"};
        for (String str : badStates) {
            assertFalse(AutoFieldState.setSides(str));
            assertNull(AutoFieldState.getAllianceSwitchSide());
            assertNull(AutoFieldState.getScaleSide());
            assertNull(AutoFieldState.getOpponentSwitchSide());
        }
        
        String[] goodStates = new String[] {"RLR", "    RLR", "RLR    ", "    RLR    "};
        for (String str : goodStates) {
            assertTrue(AutoFieldState.setSides(str));
            assertEquals(AutoFieldState.Side.RIGHT, AutoFieldState.getAllianceSwitchSide());
            assertEquals(AutoFieldState.Side.LEFT,  AutoFieldState.getScaleSide());
            assertEquals(AutoFieldState.Side.RIGHT, AutoFieldState.getOpponentSwitchSide());
        }
    }
}
