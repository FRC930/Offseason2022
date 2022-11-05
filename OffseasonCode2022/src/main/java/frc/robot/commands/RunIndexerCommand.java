package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LoadedMotorSubsystem;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;
import frc.lib.util.IndexerSensorUtility;

public class RunIndexerCommand extends CommandBase {

    private final int INDEXER_DELAY = 5; //20;

    private final double motorSpeed;
    private int counter;

    private final IndexerSubsystem indexerSubsystem;
    private final LoadedMotorSubsystem loadedMotor;

    private boolean motorsRunning;

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
        motorsRunning = false;

        this.addRequirements(m_indexerSubsystem);
    }

    // public RunIndexerCommand(IndexerSubsystem m_IndexerSubsystem, LoadedMotorSubsystem m_LoadedMotorSubsystem,
    //         double indexerMotorSpeed, boolean b) {
        //TODO   
    //}

    @Override
    public void execute() {
        double speed = (double) ShuffleboardUtility.getInstance().getFromShuffleboard(
            ShuffleboardKeys.SHOOTER_SPEED).getData();
        if (Math.abs(speed) > 0) {
            counter++;
        }
        // Waits for delay before activating indexer system
        if (counter >= INDEXER_DELAY) {
            loadedMotor.setSpeed(motorSpeed);
            indexerSubsystem.setStagedMotorSpeed(motorSpeed); //TODO 1.0);
            indexerSubsystem.setEjectionMotorSpeed(-motorSpeed); //TODO -1.0);
            motorsRunning = true;
            counter = 0;
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
        motorsRunning = false;
        counter = 0;
    }

}
