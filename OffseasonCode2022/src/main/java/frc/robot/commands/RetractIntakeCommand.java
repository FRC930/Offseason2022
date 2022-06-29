package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RetractIntakeCommand extends CommandBase{

    private final double EXTENSION_SPEED = 0.5;

    private final IntakeSubsystem intakeSubsystem;

    public RetractIntakeCommand(IntakeSubsystem m_extensionMotor) {
        intakeSubsystem = m_extensionMotor;
    }

    @Override
    public void initialize() {
        intakeSubsystem.setExtensionMotorSpeed(-EXTENSION_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopExtensionMotor(0);
    }
    
}
