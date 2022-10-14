package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.lib.util.IndexerSensorUtility;

public class IndexerCommand extends CommandBase {

    private final double motorSpeed;
    
    private final IndexerSubsystem indexerSubsystem;
    private final IndexerSensorUtility indexerSensorUtility;

    /**
     * <h3>IndexerCommand</h3>
     * 
     * Decides where to move cargo around in indexer system.
     * 
     * @param m_indexerSubsystem Motor Subsystem for indexer system
     * @param m_indexerSensor Sensor Utility for indexer system
     * @param speed the speed to set the motor
     */
    public IndexerCommand(IndexerSubsystem m_indexerSubsystem, IndexerSensorUtility m_indexerSensor, double speed) {
        indexerSubsystem = m_indexerSubsystem;
        indexerSensorUtility = m_indexerSensor;
        motorSpeed = speed;

        this.addRequirements(m_indexerSubsystem);
    }

    @Override
    public void execute() {
        if(indexerSensorUtility.loadedIsTouching() == false) { // No loaded cargo
            indexerSubsystem.setLoadedMotorSpeed(motorSpeed); // Loaded motor CW
            indexerSubsystem.setStagedMotorSpeed(-motorSpeed); // Staged motor CCW
            indexerSubsystem.setEjectionMotorSpeed(-motorSpeed); // Ejection motor CCW
        } else if(indexerSensorUtility.stagedIsTouching() == false && indexerSensorUtility.loadedIsTouching() == true) { // Loaded cargo, but No staged cargo
            indexerSubsystem.setLoadedMotorSpeed(motorSpeed); // Loaded motor CW
            indexerSubsystem.setStagedMotorSpeed(-motorSpeed); // Staged motor CCW
            indexerSubsystem.setEjectionMotorSpeed(0.0); // Ejection motor CCW
        } else { // Loaded cargo and staged cargo
            indexerSubsystem.setLoadedMotorSpeed(0.0); // Loaded motor CW
            indexerSubsystem.setStagedMotorSpeed(0.0); // Staged motor CCW
            indexerSubsystem.setEjectionMotorSpeed(0.0); // Ejection motor CCW
        }

    }

    @Override
    public boolean isFinished() { // when true, ends command
        if (indexerSensorUtility.stagedIsTouching() && indexerSensorUtility.loadedIsTouching()) {
            indexerSubsystem.stopMotors();
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopMotors();
    }

}
