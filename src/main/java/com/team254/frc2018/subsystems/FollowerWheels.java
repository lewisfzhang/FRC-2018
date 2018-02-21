package com.team254.frc2018.subsystems;

import com.team254.frc2018.Constants;
import com.team254.frc2018.loops.Looper;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FollowerWheels extends Subsystem {
    private final Encoder mLeftFollower, mRightFollower, mRearFollower;
    private final Solenoid mDeploySolenoid;
    public static double kMagEncoderCPR = 1024;

    private static FollowerWheels sInstance = new FollowerWheels();

    public static FollowerWheels getInstance() {
        return sInstance;
    }

    private FollowerWheels() {
        mLeftFollower = new Encoder(Constants.kFollowerLeftAChannelId, Constants.kFollowerLeftBChannelId, true,
                CounterBase.EncodingType.k4X);
        mLeftFollower.setName("LeftFollower");
        mRightFollower = new Encoder(Constants.kFollowerRightAChannelId, Constants.kFollowerRightBChannelId, false,
                CounterBase.EncodingType.k4X);
        mRightFollower.setName("RightFollower");
        mRearFollower = new Encoder(Constants.kFollowerRearAChannelId, Constants.kFollowerRearBChannelId, true,
                CounterBase.EncodingType.k4X);
        mRearFollower.setName("RearFollower");

        mLeftFollower.setDistancePerPulse(Math.PI * Constants.kFollowerWheelDiameterInches * (1.0 / kMagEncoderCPR));
        mRightFollower.setDistancePerPulse(Math.PI * Constants.kFollowerWheelDiameterInches * (1.0 / kMagEncoderCPR));
        mRearFollower.setDistancePerPulse(Math.PI * Constants.kFollowerWheelDiameterInches * (1.0 / kMagEncoderCPR));

        mDeploySolenoid = Constants.makeSolenoidForId(Constants.kFollowerWheelSolenoid);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Left Follower Distance", getLeftDistance());
        SmartDashboard.putNumber("Right Follower Distance", getRightDistance());
        SmartDashboard.putNumber("Rear Follower Distance", getRearDistance());
        SmartDashboard.putNumber("Follower Wheel Heading", getHeading());
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

    public void deploy() {
        mDeploySolenoid.set(true);
    }

    public void retract() {
        mDeploySolenoid.set(false);
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
        return Math.toDegrees((mRightFollower.getDistance() - mLeftFollower.getDistance()) / Constants
                .kFollowerWheelTrackWidthInches);
    }
}
