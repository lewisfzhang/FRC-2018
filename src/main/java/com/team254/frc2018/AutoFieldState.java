package com.team254.frc2018;

import com.team254.lib.geometry.Pose2d;

/**
 * Represents the state of the robot and field at the beginning
 * of the autonomous period: the starting position of the robot,
 * and the state of each of the switch/scale plates.
 */
public class AutoFieldState {
    public static enum StartingPose {
        /// TODO: fill in these values, and later replace with dynamic LIDAR reading
        LEFT(new Pose2d()),
        CENTER(new Pose2d()),
        RIGHT(new Pose2d());

        private Pose2d mStartingPose;

        StartingPose(Pose2d mStartingPose) {
            this.mStartingPose = mStartingPose;
        }

        public Pose2d getStartingPose() {
            return this.mStartingPose;
        }

        public void setStartingPose(Pose2d mStartingPose) {
            this.mStartingPose = mStartingPose;
        }
    }

    public static enum Side { LEFT, RIGHT }
    
    protected static Side mAllianceSwitchSide, mScaleSide, mOpponentSwitchSide;
    protected static StartingPose mStartPose;

    /**
     * Sets the switch/scale sides based on the given GameSpecificMessage.
     * If the message is invalid or null, returns false and leaves this
     * object unchanged; otherwise, on success, returns true.
     */
    public static boolean setSides(String gameData) {
        if (gameData == null) {
            return false;
        }

        gameData = gameData.trim();

        if (gameData.length() != 3) {
            return false;
        }

        Side s0 = getCharSide(gameData.charAt(0));
        Side s1 = getCharSide(gameData.charAt(1));
        Side s2 = getCharSide(gameData.charAt(2));

        if (s0 == null || s1 == null || s2 == null) {
            return false;
        }

        mAllianceSwitchSide = s0;
        mScaleSide = s1;
        mOpponentSwitchSide = s2;
        return true;
    }
    
    /** Helper method to convert 'L' or 'R' to their respective Side. */
    private static Side getCharSide(char c) {
        return c == 'L' ? Side.LEFT : c == 'R' ? Side.RIGHT : null;
    }
    
    /**
     * Returns which Side of our switch is our alliance's.
     */
    public static Side getAllianceSwitchSide() {
        return mAllianceSwitchSide;
    }
    
    /**
     * Returns which Side of the scale is our alliance's.
     */
    public static Side getScaleSide() {
        return mScaleSide;
    }
    
    /**
     * Returns which Side of our opponent's switch is our alliance's.
     */
    public static Side getOpponentSwitchSide() {
        return mOpponentSwitchSide;
    }
    
    /**
     * Sets the inital pose of the robot at the beginning of auton.
     */
    public static void setStartPose(Pose2d startPose) {
        if (mStartPose == null) {
            mStartPose = AutoFieldState.StartingPose.LEFT; //TODO set default
        }

        mStartPose.setStartingPose(startPose);
    }

    public static void setStartPose(StartingPose startPose) {
        mStartPose = startPose;
    }

    /**
     * Returns the inital pose of the robot at the beginning of auton.
     */
    public static Pose2d getStartPose() {
        return mStartPose.getStartingPose();
    }

    @Override
    public String toString() {
        return "AutoFieldState{" +
            "ourSwitchSide=" + mAllianceSwitchSide +
            ", scaleSide=" + mScaleSide +
            ", opponentSwitchSide=" + mOpponentSwitchSide +
            ", startPose=" + mStartPose.getStartingPose() +
        '}';
    }
}