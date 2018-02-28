package com.team254.frc2018.subsystems;

import com.team254.frc2018.Constants;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FollowerWheels extends Subsystem {
    public static double kMagEncoderCPR = 1024;
    private static FollowerWheels sInstance = new FollowerWheels();
    private final Encoder mLeftFollower, mRightFollower, mRearFollower;
    private final Solenoid mDeploySolenoid;
    private PeriodicInputs mPeriodicInputs = new PeriodicInputs();

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

        mLeftFollower.setDistancePerPulse(1);
        mRightFollower.setDistancePerPulse(1);
        mRearFollower.setDistancePerPulse(1);

        mDeploySolenoid = Constants.makeSolenoidForId(Constants.kFollowerWheelSolenoid);
    }

    public static FollowerWheels getInstance() {
        return sInstance;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        //Left follower
        int currentLeftTicks = mLeftFollower.getRaw();
        double deltaLeftTicks = ((currentLeftTicks - mPeriodicInputs.left_position_ticks_) / 4096.0) * Math.PI;
        if (deltaLeftTicks > 0.0) {
            mPeriodicInputs.left_distance_ += deltaLeftTicks * Constants.kFollowerWheelDiameterInchesForwards;
        } else {
            mPeriodicInputs.left_distance_ += deltaLeftTicks * Constants.kFollowerWheelDiameterInchesReverse;
        }
        mPeriodicInputs.left_position_ticks_ = currentLeftTicks;
        mPeriodicInputs.left_velocity_ticks_per_s = mLeftFollower.getRate();

        //Right follower
        int currentRightTicks = mRightFollower.getRaw();
        double deltaRightTicks = ((currentRightTicks - mPeriodicInputs.right_position_ticks_) / 4096.0) * Math.PI;
        if (deltaRightTicks > 0.0) {
            mPeriodicInputs.right_distance_ += deltaRightTicks * Constants.kFollowerWheelDiameterInchesForwards;
        } else {
            mPeriodicInputs.right_distance_ += deltaRightTicks * Constants.kFollowerWheelDiameterInchesReverse;
        }
        mPeriodicInputs.right_distance_ = currentRightTicks;
        mPeriodicInputs.right_velocity_ticks_per_s = mRightFollower.getRate();

        //Back follower, just use standard distance for now.  Might want to check rotation to apply different wheel
        // radii
        mPeriodicInputs.rear_position_ticks_ = mRearFollower.getRaw();
        double rearDistanceRadians = (mPeriodicInputs.rear_position_ticks_ / 4096.0) * Math.PI;
        mPeriodicInputs.rear_distance_ = rearDistanceRadians * Constants.kFollowerWheelDiameterInches;
        mPeriodicInputs.rear_velocity_ticks_per_s = mRearFollower.getRate();
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
        SmartDashboard.putNumber("Left Ticks", getLeftTicks());
        SmartDashboard.putNumber("Right Ticks", getRightTicks());
        SmartDashboard.putNumber("Rear Ticks", getRearTicks());

    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {
        mLeftFollower.reset();
        mRearFollower.reset();
        mRightFollower.reset();
        mPeriodicInputs = new PeriodicInputs();
    }

    public void deploy() {
        mDeploySolenoid.set(true);
    }

    public void retract() {
        mDeploySolenoid.set(false);
    }

    public int getLeftTicks() {
        return mPeriodicInputs.left_position_ticks_;
    }

    public double getLeftDistance() {
        return mPeriodicInputs.left_distance_;
    }

    public double getLeftVelocity() {
        return mPeriodicInputs.left_velocity_ticks_per_s;
    }

    public int getRightTicks() {
        return mPeriodicInputs.right_position_ticks_;
    }

    public double getRightDistance() {
        return mPeriodicInputs.right_distance_;
    }

    public double getRightVelocity() {
        return mPeriodicInputs.right_velocity_ticks_per_s;
    }

    public int getRearTicks() {
        return mPeriodicInputs.rear_position_ticks_;
    }

    public double getRearDistance() {
        return mPeriodicInputs.rear_distance_;
    }

    public double getRearVelocity() {
        return mPeriodicInputs.rear_velocity_ticks_per_s;
    }

    public double getHeading() {
        return Math.toDegrees((getRightDistance() - getLeftDistance()) / Constants
                .kFollowerWheelTrackWidthInches);
    }

    private static class PeriodicInputs {
        public double left_distance_;
        public double right_distance_;
        public double rear_distance_;
        public int left_position_ticks_;
        public int right_position_ticks_;
        public int rear_position_ticks_;
        public double left_velocity_ticks_per_s;
        public double right_velocity_ticks_per_s;
        public double rear_velocity_ticks_per_s;
    }
}
