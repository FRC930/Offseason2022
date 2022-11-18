/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
    private boolean usingShuffleboard;
    private double loadedSpeed;
    private double speed;
    private int counter;

    /**
     * <h3>ShootCargoCommand</h3>
     * Uses shuffleboard for speeds.
     * 
     * @param shooter The ShooterSubsystem to use
     * @param indexer The IndexerMotorSubsystem to use
     */
    public ShooterCommand(ShooterMotorSubsystem shooter) {
        // ---CANNOT USE this() BECAUSE OF BOOLEAN FLAG---\\
        shooterSubsystem = shooter;
        usingShuffleboard = true;
        addRequirements(shooterSubsystem);
    }

    /**
     * <h3>ShootCargoCommand</h3>
     * 
     * @param shooter The ShooterSubsystem to use
     * @param indexer The IndexerMotorSubsystem to use
     * @param speed   The speed(in PercentOutput) you want both wheels to spin at
     */
    public ShooterCommand(ShooterMotorSubsystem shooter,  double speed) {
        // Applies speed to both motors
        this(shooter, speed, speed);
    }

    /**
     * <h3>ShootCargoCommand</h3>
     * 
     * @param shooter     The ShooterSubsystem to use
     * @param indexer     The IndexerMotorSubsystem to use
     * @param speed    The speed(in PercentOutput) you want the other wheels to spin
     *                    at
     * @param laodedSpeed The speed(in PercentOutput) you want the loaded wheel to
     *                    spin at
     */
    public ShooterCommand(ShooterMotorSubsystem shooter, double speed, double loadedSpeed) {
        shooterSubsystem = shooter;
        usingShuffleboard = false;
        this.speed = speed;
        this.loadedSpeed = loadedSpeed;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        // Gets values from shuffleboard driver tab
        if (usingShuffleboard) {
            this.loadedSpeed = (double) ShuffleboardUtility.getInstance().getFromShuffleboard(
                    ShuffleboardKeys.SHOOTER_LOADED_SPEED).getData();
            this.speed = (double) ShuffleboardUtility.getInstance().getFromShuffleboard(
                    ShuffleboardKeys.SHOOTER_SPEED).getData();
        } else {
            ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                    ShuffleboardKeys.SHOOTER_LOADED_SPEED, new ShuffleBoardData<Double>(loadedSpeed));
            ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                    ShuffleboardKeys.SHOOTER_SPEED, new ShuffleBoardData<Double>(speed));
        }

        shooterSubsystem.setRightSpeed(speed, loadedSpeed);
        counter = 0;
    }

    @Override
    public void execute() { // TODO: Pull out indexer and see how we did it on last year's bot
      
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
    }
} // End of CLass
