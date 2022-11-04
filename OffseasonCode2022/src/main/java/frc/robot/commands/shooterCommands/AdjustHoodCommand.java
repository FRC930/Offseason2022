package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterMotorSubsystem;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

/**
 * <h3>AdjustHoodCommand</h3>
 * 
 * Adjusts position of hood to a specific angle
 */
public class AdjustHoodCommand extends CommandBase {

    private final ShooterHoodSubsystem shooterHoodSubsystem;
    private double pos;
    private boolean usingShuffleboard;

    /**
     * <h3>AdjustHoodCommand</h3>
     * 
     * Adjusts position of hood with value
     * 
     * @param shooterHood The subsystem of the shooter's hood
     * @param pos         The position to set the hood in degrees
     */
    public AdjustHoodCommand(ShooterHoodSubsystem shooterHood, double pos) {
        shooterHoodSubsystem = shooterHood;
        this.pos = pos;
        usingShuffleboard = false;
    }

    /**
     * <h3>AdjustHoodCommand</h3>
     * 
     * Adjusts position of hood using shuffleboard values.
     * 
     * @param m_ShooterMotorSubsystem The subsystem of the shooter's hood
     */
    public AdjustHoodCommand(ShooterHoodSubsystem m_ShooterHoodSubsystem) {
        this(m_ShooterHoodSubsystem, 0);
        usingShuffleboard = true;
    }

    @Override
    public void initialize() {
        // Gets values from shuffleboard driver tab
        if (usingShuffleboard) {
            pos = (double) ShuffleboardUtility.getInstance().getFromShuffleboard(
                    ShuffleboardKeys.SHOOTER_HOOD_POSITION).getData();
        } else {
            ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                    ShuffleboardKeys.SHOOTER_HOOD_POSITION, new ShuffleBoardData<Double>(pos));
        }
        // Sets the position for the hood

        shooterHoodSubsystem.setHoodPosition(pos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
