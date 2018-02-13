package com.team254.frc2018.subsystems;

import com.team254.frc2018.Constants;
import com.team254.frc2018.loops.Looper;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FollowerWheels extends Subsystem {
    private final Encoder mLeftFollower, mRightFollower, mRearFollower;
    public static double kCTREEnocderCPR = 1024;

    private static FollowerWheels sInstance = new FollowerWheels();

    public static FollowerWheels getInstance() {
        return sInstance;
    }

    private FollowerWheels() {
        mLeftFollower = new Encoder(Constants.kFollowerLeftAChannelId, Constants.kFollowerLeftBChannelId, true, CounterBase.EncodingType.k4X);
        mRightFollower = new Encoder(Constants.kFollowerRightAChannelId, Constants.kFollowerRightBChannelId, false, CounterBase.EncodingType.k4X);
        mRearFollower = new Encoder(Constants.kFollowerRearAChannelId, Constants.kFollowerRearBChannelId, true, CounterBase.EncodingType.k4X);



        mLeftFollower.setDistancePerPulse(Math.PI * Constants.kFollowerWheelDiameterInches * (1.0 / kCTREEnocderCPR));
        mRightFollower.setDistancePerPulse(Math.PI * Constants.kFollowerWheelDiameterInches * (1.0 / kCTREEnocderCPR));
        mRearFollower.setDistancePerPulse(Math.PI * Constants.kFollowerWheelDiameterInches * (1.0 / kCTREEnocderCPR));

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("leftFollowerDistance", getLeftDistance());
        SmartDashboard.putNumber("rightFollowerDistance", getRightDistance());
        SmartDashboard.putNumber("rearFollowerDistance", getRearDistance());
        SmartDashboard.putNumber("headingFollower", getHeading());
    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {
        mLeftFollower.reset();
        mRearFollower.reset();
        mRightFollower.reset();
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {

    }

    public double getLeftDistance() {
        return mLeftFollower.getDistance();
    }

    public double getLeftVelocity() {
        return mLeftFollower.getRate();
    }

    public double getRightDistance() {
        return mRightFollower.getDistance();
    }

    public double getRightVelocity() {
        return mRightFollower.getRate();
    }

    public double getRearDistance() {
        return mRearFollower.getDistance();
    }

    public double getRearVelocity() {
        return mRearFollower.getRate();
    }

    public double getHeading() {
        return Math.toDegrees((mRightFollower.getDistance() - mLeftFollower.getDistance()) / Constants.kFollowerWheelTrackWidthInches);
    }
}
