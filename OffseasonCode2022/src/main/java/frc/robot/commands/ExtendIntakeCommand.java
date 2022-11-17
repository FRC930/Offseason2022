package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeExtensionSubsystem;

public class ExtendIntakeCommand extends CommandBase {

    private final IntakeExtensionSubsystem intakeEncoderSubsystem;

    public ExtendIntakeCommand(IntakeExtensionSubsystem m_extensionMotor) {
        intakeEncoderSubsystem = m_extensionMotor;
    }

    @Override
    public void initialize() {
        intakeEncoderSubsystem.extendIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeEncoderSubsystem.retractIntake();
    }
}
