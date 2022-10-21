package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeRollersCommand extends CommandBase{

    private final double ROLLER_SPEED = 0.25; //TODO 0.5;

    private final IntakeSubsystem intakeSubsystem;

    public RunIntakeRollersCommand(IntakeSubsystem m_rollerMotor) {
        intakeSubsystem = m_rollerMotor;
    }

    @Override
    public void initialize() {
        //TODO look at ShooterHoodSubsystem(setHoodPosition) intakeSubsystem.setRollerMotorSpeed(ROLLER_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopRollerMotor(0);
    }
    
}
