package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LoadedMotorSubsystem;
import frc.lib.util.IndexerSensorUtility;

public class IndexerCommand extends CommandBase {

    private final double motorSpeed;
    
    private final IndexerSubsystem indexerSubsystem;
    private final LoadedMotorSubsystem loadedMotor;

    private final IndexerSensorUtility indexerSensorUtility;

    /**
     * <h3>IndexerCommand</h3>
     * 
     * Decides where to move cargo around in indexer system.
     * 
     * @param m_indexerSubsystem Motor Subsystem for indexer system
     * @param m_indexerSensor Sensor Utility for indexer system
     * @param m_loadedMotor Loaded Motor Subsystem
     * @param speed the speed to set the motor
     */
    public IndexerCommand(IndexerSubsystem m_indexerSubsystem, IndexerSensorUtility m_indexerSensor, LoadedMotorSubsystem m_loadedMotor, double speed) {
        indexerSubsystem = m_indexerSubsystem;
        indexerSensorUtility = m_indexerSensor;
        loadedMotor = m_loadedMotor;
        motorSpeed = speed;

        this.addRequirements(m_indexerSubsystem);
    }

    @Override
    public void execute() {
        if(indexerSensorUtility.loadedIsTouching() == false) { // No loaded cargo
            indexerSubsystem.setStagedMotorSpeed(motorSpeed); // Staged motor CCW
            indexerSubsystem.setEjectionMotorSpeed(-motorSpeed); // Ejection motor CCW
            loadedMotor.setSpeed(0.25);
        } else if(indexerSensorUtility.stagedIsTouching() == false && indexerSensorUtility.loadedIsTouching() == true) { // Loaded cargo, but No staged cargo
            indexerSubsystem.setStagedMotorSpeed(motorSpeed); // Staged motor CCW
            indexerSubsystem.setEjectionMotorSpeed(0.0); // Ejection motor CCW
            loadedMotor.setSpeed(0.0);
        } else { // Loaded cargo and staged cargo
            indexerSubsystem.setStagedMotorSpeed(0.0); // Staged motor CCW
            indexerSubsystem.setEjectionMotorSpeed(0.0); // Ejection motor CCW
            loadedMotor.setSpeed(0.0);
        }

    }

    @Override
    public boolean isFinished() { // when true, ends command
        if (indexerSensorUtility.stagedIsTouching() && indexerSensorUtility.loadedIsTouching()) {
            indexerSubsystem.stopMotors();
            loadedMotor.setSpeed(0.0);
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopMotors();
        loadedMotor.setSpeed(0.0);
    }

}
