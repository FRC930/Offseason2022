package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndgamePistonSubsystem;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

public class EngageEndgamePistonCommand extends CommandBase {

    EndgamePistonSubsystem m_endgamePistonSubsystem;
    
    public EngageEndgamePistonCommand(EndgamePistonSubsystem epSubsystem) {
        m_endgamePistonSubsystem = epSubsystem;
    }

    @Override
    public void initialize() {
        m_endgamePistonSubsystem.setEndgameSolenoidState(true);
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.testingTab, ShuffleboardKeys.PRESSURE,new ShuffleBoardData<Boolean>(true));
    }

    @Override 
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_endgamePistonSubsystem.setEndgameSolenoidState(false);
    }
}
