package com.team254.frc2018.subsystems;

import com.team254.frc2018.Constants;
import com.team254.frc2018.loops.Loop;
import com.team254.frc2018.loops.Looper;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FollowerWheels extends Subsystem {
    private final Encoder mLeftFollower, mRightFollower, mRearFollower;
    private final Solenoid mDeploySolenoid;
    public static double kMagEncoderCPR = 1024;
    private double mLeftDistance, mRightDistance, mRearDistance;
    private double mLeftTicks, mRightTicks, mRearTicks;

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
        SmartDashboard.putNumber("Left Ticks", mLeftFollower.getRaw());
        SmartDashboard.putNumber("Right Ticks", mRightFollower.getRaw());
        SmartDashboard.putNumber("Rear Ticks", mRearFollower.getRaw());

    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {
        mLeftFollower.reset();
        mRearFollower.reset();
        mRightFollower.reset();
        mLeftDistance = 0.0;
        mRightDistance = 0.0;
        mRearDistance = 0.0;
        mLeftTicks = 0.0;
        mRightTicks = 0.0;
        mRearTicks = 0.0;
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                zeroSensors();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (FollowerWheels.this) {
                    //Left follower
                    double currentLeftTicks = mLeftFollower.getRaw();
                    double deltaLeftTicks = ((currentLeftTicks - mLeftTicks) / 4096.0) * Math.PI;
                    if(deltaLeftTicks > 0.0) {
                        mLeftDistance += deltaLeftTicks * Constants.kFollowerWheelDiameterInchesForwards;
                    } else {
                        mLeftDistance += deltaLeftTicks * Constants.kFollowerWheelDiameterInchesReverse;
                    }
                    mLeftTicks = currentLeftTicks;

                    //Right follower
                    double currentRightTicks = mRightFollower.getRaw();
                    double deltaRightTicks = ((currentRightTicks - mRightTicks) / 4096.0) * Math.PI;
                    if(deltaRightTicks > 0.0) {
                        mRightDistance += deltaRightTicks * Constants.kFollowerWheelDiameterInchesForwards;
                    } else {
                        mRightDistance += deltaRightTicks * Constants.kFollowerWheelDiameterInchesReverse;
                    }
                    mRightTicks = currentRightTicks;

                    //Back follower, just use standard distance for now.  Might want to check rotation to apply different wheel radii
                    double rearDistanceRadians = (mRearFollower.getRaw() / 4096.0) * Math.PI;
                    mRearDistance = rearDistanceRadians * Constants.kFollowerWheelDiameterInches;
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });

    }

    public void deploy() {
        mDeploySolenoid.set(true);
    }

    public void retract() {
        mDeploySolenoid.set(false);
    }

    public double getLeftDistance() {
        return mLeftDistance;
    }

    public double getLeftVelocity() {
        return mLeftFollower.getRate();
    }

    public double getRightDistance() {
        return mRightDistance;
    }

    public double getRightVelocity() {
        return mRightFollower.getRate();
    }

    public double getRearDistance() {
        return mRearDistance;
    }

    public double getRearVelocity() {
        return mRearFollower.getRate();
    }

    public double getHeading() {
        return Math.toDegrees((getRightDistance() - getLeftDistance()) / Constants
                .kFollowerWheelTrackWidthInches);
    }
}
