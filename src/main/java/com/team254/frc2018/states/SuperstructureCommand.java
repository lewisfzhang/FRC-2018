package com.team254.frc2018.states;

import com.team254.frc2018.statemachines.IntakeStateMachine;

public class SuperstructureCommand {
    public double height = 0;
    public double wristAngle = 0;
    IntakeStateMachine.WantedAction intakeAction = IntakeStateMachine.WantedAction.IDLE;
}
