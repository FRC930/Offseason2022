package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerEjectCommand extends CommandBase {

    private final double motorSpeed;
    
    private final IndexerSubsystem indexerSubsystem;

    /**
     * <h3>IndexerCommand</h3>
     * 
     * Decides where to move cargo around in indexer system.
     * 
     * @param m_indexerSubsystem Motor Subsystem for indexer system
     * @param speed the speed to set the motor
     */
    public IndexerEjectCommand(IndexerSubsystem m_indexerSubsystem, double speed) {
        indexerSubsystem = m_indexerSubsystem;
        motorSpeed = speed;

        this.addRequirements(m_indexerSubsystem);
    }

    @Override
    public void execute() {
        indexerSubsystem.setLoadedMotorSpeed(0.0); // Loaded motor stopped
        indexerSubsystem.setStagedMotorSpeed(-motorSpeed); // Staged motor CCW
        indexerSubsystem.setEjectionMotorSpeed(motorSpeed); // Ejection motor CC
    }

    @Override
    public boolean isFinished() { // when true, ends command
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopMotors();
    }

}
