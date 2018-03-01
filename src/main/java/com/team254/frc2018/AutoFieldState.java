package com.team254.frc2018;

import com.team254.lib.geometry.Pose2d;

public class AutoFieldState {
    
    public static final Pose2d LEFT = new Pose2d();
    public static final Pose2d CENTER = new Pose2d();
    public static final Pose2d RIGHT = new Pose2d();
    
    public static enum Side { LEFT, RIGHT }
    
    protected Side ourSwitchSide, scaleSide, opponentSwitchSide;
    protected Pose2d startPose;
    
    public AutoFieldState() {
        startPose = null;
    }
    
    public AutoFieldState(Pose2d startPose) {
        this.startPose = startPose;
    }
    
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
    
    private Side getCharSide(char c) {
        return c=='L'? Side.LEFT : c=='R'? Side.RIGHT : null;
    }
    
    public Side getOurSwitchSide() {
        return ourSwitchSide;
    }
    
    public Side getScaleSide() {
        return scaleSide;
    }
    
    public Side getOpponentSwitchSide() {
        return opponentSwitchSide;
    }
    
    public void setStartPose(Pose2d startPose) {
        this.startPose = startPose;
    }
    
    public Pose2d getStartPose() {
        return startPose;
    }
    
}