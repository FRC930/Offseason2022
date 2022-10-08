package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndgamePistonSubsystem;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

public class DisengageEndgamePistonCommand extends CommandBase{

    EndgamePistonSubsystem m_endgamePistonSubsystem;
    
    public DisengageEndgamePistonCommand(EndgamePistonSubsystem epSubsystem) {
        epSubsystem.setEndgameSolenoidState(false);
    }
      @Override
    public void initialize() {
        m_endgamePistonSubsystem.setEndgameSolenoidState(false);
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
