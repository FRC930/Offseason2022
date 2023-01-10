package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LoadedMotorSubsystem;

public class RunIndexerCommand extends CommandBase {

    private final int INDEXER_DELAY = 20;

    private final double motorSpeed;
    private int counter;

    private final IndexerSubsystem indexerSubsystem;
    private final LoadedMotorSubsystem loadedMotor;

    /**
     * <h3>RunIndexerCommand</h3>
     * 
     * Decides where to move cargo around in indexer system.
     * 
     * @param m_indexerSubsystem Motor Subsystem for indexer system
     * @param m_indexerSensor Sensor Utility for indexer system
     * @param speed the speed to set the motor
     */
    public RunIndexerCommand(IndexerSubsystem m_indexerSubsystem,  LoadedMotorSubsystem m_loadedMotor, double speed) {
        counter = 0;
        indexerSubsystem = m_indexerSubsystem;
        loadedMotor = m_loadedMotor;
        motorSpeed = speed;

        this.addRequirements(m_indexerSubsystem);
    }

    @Override
    public void execute() {
        counter++;
        // Waits for delay before activating indexer system
        if (counter >= INDEXER_DELAY) {
            loadedMotor.setSpeed(motorSpeed);
            indexerSubsystem.setStagedMotorSpeed(motorSpeed); //TODO 1.0);
            indexerSubsystem.setEjectionMotorSpeed(-motorSpeed); //TODO -1.0);
        }

    }

    @Override
    public boolean isFinished() { // when true, ends command

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopMotors();
        loadedMotor.setSpeed(0.0);
        counter = 0;
    }

}
