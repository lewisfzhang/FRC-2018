package com.team254.frc2018.auto.actions;

import com.team254.frc2018.statemachines.IntakeStateMachine;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.subsystems.Intake;
import com.team254.frc2018.subsystems.Superstructure;
import edu.wpi.first.wpilibj.Timer;

public class ShootCube implements Action {
    private static final Intake mIntake = Intake.getInstance();
    private static final double kShootTime = 0.75;

    private double mStartTime;

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mIntake.shoot();
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime > kShootTime;
    }

    @Override
    public void done() {
        mIntake.setState(IntakeStateMachine.WantedAction.WANT_MANUAL);
        mIntake.setPower(0.0);
    }
}
