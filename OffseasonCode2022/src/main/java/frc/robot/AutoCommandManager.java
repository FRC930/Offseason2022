package frc.robot;

import java.util.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.autos.*;
import frc.robot.subsystems.*;

/**
 * <h3>AutonomouseCommandManager</h3>
 * 
 * Manages the autonomous paths by creating an instance of them and putting them
 * into the Shuffleboard.
 */
public class AutoCommandManager {
    HashMap<String, Subsystem> subsystemMap = new HashMap<String, Subsystem>();

    public static enum subNames {
        Swerve("Drive");

        final String m_name;

        subNames(String name) {
            m_name = name;
        }
    }

    /**
     * Adds a subbsystem to the subystem map
     *
     * @param SubNames
     * @param subsbystem
     */
    public void addSubsystem(subNames SubNames, Subsystem subsystem) {
        subsystemMap.put(SubNames.toString(), subsystem);
    }

    /**
     * Creates instances of each autonomous path command
     */
    public void initCommands() {

        TaxiOneBall TaxiOneBall = new TaxiOneBall(
                (Swerve) subsystemMap.get(subNames.Swerve.toString()));

        TaxiTwoBall TaxiTwoBall = new TaxiTwoBall(
                (Swerve) subsystemMap.get(subNames.Swerve.toString()));

        TerminalPickup TerminalPickup = new TerminalPickup(
                (Swerve) subsystemMap.get(subNames.Swerve.toString()));

        FiveBallAuto FiveBallAuto = new FiveBallAuto(
                (Swerve) subsystemMap.get(subNames.Swerve.toString()));

        DefensiveThreeBall DefensiveThreeBall = new DefensiveThreeBall(
                (Swerve) subsystemMap.get(subNames.Swerve.toString()));

        DefensiveHangarCleanup DefensiveHangarCleanup = new DefensiveHangarCleanup(
                (Swerve) subsystemMap.get(subNames.Swerve.toString()));

        // Adding auto paths to the Shuffleboard to be used by the get autonomous command
        ShuffleboardUtility.getInstance().setDefaultAutonOptions("(None)", null);
        ShuffleboardUtility.getInstance().addAutonOptions("TaxiTwoBall", TaxiTwoBall);
        // ShuffleboardUtility.getInstance().addAutonOptions("TaxiOneBall", TaxiOneBall);
        // ShuffleboardUtility.getInstance().addAutonOptions("TerminalPickup", TerminalPickup);
        // ShuffleboardUtility.getInstance().addAutonOptions("DefensiveThreeBall", DefensiveThreeBall);
        // ShuffleboardUtility.getInstance().addAutonOptions("DefensiveHangarCleanup", DefensiveHangarCleanup);
        // ShuffleboardUtility.getInstance().addAutonOptions("FiveBallAuto", FiveBallAuto);
    }

    /**
     *
     * Gets the autonomous path that is selected in the Shuffleboard
     *
     * @return The selected autonomous command
     */
    public Command getAutonomousCommand() {
        return ShuffleboardUtility.getInstance().getSelectedAutonPath();
    }

}
