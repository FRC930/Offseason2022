package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.lib.util.IndexerSensorUtility;

public class IndexerCommand extends CommandBase {

    private final double motorSpeed;
    
    private final IndexerSubsystem indexerSubsystem;
    private final IndexerSensorUtility indexerSensorUtility;

    private int cargoCount; // Basic counter to decide what steps to take with motors

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
    }

    @Override
    public void execute() {


        if (indexerSensorUtility.intakeIsTouching() == true) { // TODO: Remove if statement, redundant because command is only run whenever the intake sensor is touching

            cargoCount = 0; // Resets counter

            if (indexerSensorUtility.loadedIsTouching() == true) { // Add one to the counter if sensor is touching
                cargoCount++;
            }
    
            if (indexerSensorUtility.stagedIsTouching() == true) { // Add one to the counter if sensor is touching
                cargoCount++;
            }
    
            if (cargoCount >= 2) { // failsafe
                cargoCount = 2;
            }
    
    
            switch (cargoCount) { // Decides where cargo should be moved
    
                case 0: // If there is no cargo in the robot already, moves new cargo to loaded sensor
                    while (indexerSensorUtility.loadedIsTouching() == false) {
                        indexerSubsystem.setLoadedMotorSpeed(motorSpeed); // Loaded motor CW
                        indexerSubsystem.setStagedMotorSpeed(-motorSpeed); // Staged motor CCW
                        indexerSubsystem.setEjectionMotorSpeed(-motorSpeed); // Ejection motor CW
                    }
                    indexerSubsystem.stopMotors(); // stops all motors
                    break;
    
                case 1: // If there is 1 cargo in the robot already, moves new cargo to staged sensor
                    while (indexerSensorUtility.stagedIsTouching() == false) {
                        indexerSubsystem.setLoadedMotorSpeed(motorSpeed); // Loaded motor CW
                        indexerSubsystem.setStagedMotorSpeed(-motorSpeed); // Staged motor CCW
                    }
                    indexerSubsystem.stopMotors(); // stops all motors
                    break;

                default: // If there is 2 cargo in the robot already, moves staged cargo to ejection sensor and new cargo to staged sensor
                    while (indexerSensorUtility.ejectionIsTouching() == false) {
                        indexerSubsystem.setStagedMotorSpeed(-motorSpeed); // Staged motor CCW
                        indexerSubsystem.setEjectionMotorSpeed(motorSpeed); // Ejection motor CW
                    }
                    
                    indexerSubsystem.setEjectionMotorSpeed(0.0); // Stops ejection motor
                    while (indexerSensorUtility.stagedIsTouching() == false) {
                        indexerSubsystem.setStagedMotorSpeed(-motorSpeed); // Staged motor CCW
                        indexerSubsystem.setEjectionMotorSpeed(motorSpeed); // Ejection motor CW
                    }
                    indexerSubsystem.stopMotors(); // stops all motors
                    break;
    
            }
        }
    

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
