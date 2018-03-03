package com.team254.frc2018;

import com.team254.lib.geometry.Pose2d;

/**
 * Represents the state of the robot and field at the beginning
 * of the autonomous period: the starting position of the robot,
 * and the state of each of the switch/scale plates.
 */
public class AutoFieldState {
    
    /// TODO: fill in these values, and later replace with dynamic LIDAR reading
    public static final Pose2d LEFT = new Pose2d();
    public static final Pose2d CENTER = new Pose2d();
    public static final Pose2d RIGHT = new Pose2d();
    
    public static enum Side { LEFT, RIGHT }
    
    protected Side ourSwitchSide, scaleSide, opponentSwitchSide;
    protected Pose2d startPose;
    
    /**
     * Initializes the start pose to null.
     */
    public AutoFieldState() {
        startPose = null;
    }
    
    /**
     * Initializes the start pose to the given pose.
     */
    public AutoFieldState(Pose2d startPose) {
        this.startPose = startPose;
    }
    
    /**
     * Sets the switch/scale sides based on the given GameSpecificMessage.
     * If the message is invalid or null, returns false and leaves this
     * object unchanged; otherwise, on success, returns true.
     */
    public boolean setSides(String gameData) {
        if (gameData == null) return false;
        gameData = gameData.trim();
        if (gameData.length() != 3) return false;
        Side s0 = getCharSide(gameData.charAt(0));
        Side s1 = getCharSide(gameData.charAt(1));
        Side s2 = getCharSide(gameData.charAt(2));
        if (s0 == null || s1 == null || s2 == null) return false;
        ourSwitchSide = s0;
        scaleSide = s1;
        opponentSwitchSide = s2;
        return true;
    }
    
    /** Helper method to convert 'L' or 'R' to their respective Side. */
    private Side getCharSide(char c) {
        return c=='L'? Side.LEFT : c=='R'? Side.RIGHT : null;
    }
    
    /**
     * Returns which Side of our switch is our alliance's.
     */
    public Side getOurSwitchSide() {
        return ourSwitchSide;
    }
    
    /**
     * Returns which Side of the scale is our alliance's.
     */
    public Side getScaleSide() {
        return scaleSide;
    }
    
    /**
     * Returns which Side of our opponent's switch is our alliance's.
     */
    public Side getOpponentSwitchSide() {
        return opponentSwitchSide;
    }
    
    /**
     * Sets the inital pose of the robot at the beginning of auton.
     */
    public void setStartPose(Pose2d startPose) {
        this.startPose = startPose;
    }
    
    /**
     * Returns the inital pose of the robot at the beginning of auton.
     */
    public Pose2d getStartPose() {
        return startPose;
    }

    @Override
    public String toString() {
        return "AutoFieldState{" +
                "ourSwitchSide=" + ourSwitchSide +
                ", scaleSide=" + scaleSide +
                ", opponentSwitchSide=" + opponentSwitchSide +
                ", startPose=" + startPose +
                '}';
    }
}