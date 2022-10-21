/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterMotorSubsystem;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

/**
 * <h3>ShootCargoCommand</h3>
 * 
 * Complete shooting command.
 */
public class ShooterCommand extends CommandBase {

    // -----CONSTANTS----\\
    public static final double TELEOP_SHOOT_TIME = 5.0;
    // Number of cycles to wait before sending balls into shooter. (Cycles = time(in
    // seconds) / 0.02)
    private final int INDEXER_DELAY = 20;

    // -----VARIABLES----\\
    private final ShooterMotorSubsystem shooterSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private boolean usingShuffleboard;
    private double bottomSpeed;
    private double topSpeed;
    private int counter;

    /**
     * <h3>ShootCargoCommand</h3>
     * Uses shuffleboard for speeds.
     * 
     * @param shooter The ShooterSubsystem to use
     * @param indexer The IndexerMotorSubsystem to use
     */
    public ShooterCommand(ShooterMotorSubsystem shooter, IndexerSubsystem indexer) {
        // ---CANNOT USE this() BECAUSE OF BOOLEAN FLAG---\\
        shooterSubsystem = shooter;
        indexerSubsystem = indexer;
        usingShuffleboard = true;
        addRequirements(shooterSubsystem, indexerSubsystem);
    }

    /**
     * <h3>ShootCargoCommand</h3>
     * 
     * @param shooter The ShooterSubsystem to use
     * @param indexer The IndexerMotorSubsystem to use
     * @param speed   The speed(in PercentOutput) you want both wheels to spin at
     */
    public ShooterCommand(ShooterMotorSubsystem shooter, IndexerSubsystem indexer, double speed) {
        // Applies speed to both motors
        this(shooter, indexer, speed, speed);
    }

    /**
     * <h3>ShootCargoCommand</h3>
     * 
     * @param shooter     The ShooterSubsystem to use
     * @param indexer     The IndexerMotorSubsystem to use
     * @param topSpeed    The speed(in PercentOutput) you want the top wheel to spin
     *                    at
     * @param bottomSpeed The speed(in PercentOutput) you want the bottom wheel to
     *                    spin at
     */
    public ShooterCommand(ShooterMotorSubsystem shooter, IndexerSubsystem indexer, double topSpeed,
            double bottomSpeed) {
        shooterSubsystem = shooter;
        indexerSubsystem = indexer;
        usingShuffleboard = false;
        this.topSpeed = topSpeed;
        this.bottomSpeed = bottomSpeed;
        addRequirements(shooterSubsystem, indexerSubsystem);
    }

    @Override
    public void initialize() {
        // Gets values from shuffleboard driver tab
        if (usingShuffleboard) {
            this.bottomSpeed = (double) ShuffleboardUtility.getInstance().getFromShuffleboard(
                    ShuffleboardKeys.SHOOTER_BOTTOM_SPEED).getData();
            this.topSpeed = (double) ShuffleboardUtility.getInstance().getFromShuffleboard(
                    ShuffleboardKeys.SHOOTER_TOP_SPEED).getData();
        } else {
            ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                    ShuffleboardKeys.SHOOTER_BOTTOM_SPEED, new ShuffleBoardData<Double>(bottomSpeed));
            ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                    ShuffleboardKeys.SHOOTER_TOP_SPEED, new ShuffleBoardData<Double>(topSpeed));
        }

        shooterSubsystem.setRightSpeed(bottomSpeed);
        counter = 0;
    }

    @Override
    public void execute() {
        counter++;
        // Waits for delay before activating indexer system
        if (counter == INDEXER_DELAY) {
            
            indexerSubsystem.setStagedMotorSpeed(0.25); //TODO 1.0);
            indexerSubsystem.setLoadedMotorSpeed(0.25); //TODO 1.0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
        // Finishes if no balls are left
        //return !BallSensorUtility.getInstance().intakeIsTripped() &&
        //        !BallSensorUtility.getInstance().loadedIsTripped();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopMotors();
        indexerSubsystem.stopMotors();
    }
} // End of CLass
