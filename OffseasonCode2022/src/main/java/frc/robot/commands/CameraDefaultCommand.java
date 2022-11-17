package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

//
//  this class will be used to testing purposes
//  -- still needs work.
public class CameraDefaultCommand extends CommandBase {
    private CameraSubsystem m_cameraSubsystem ;

    public CameraDefaultCommand(CameraSubsystem vCameraSub) {
        m_cameraSubsystem = vCameraSub ;
        addRequirements(m_cameraSubsystem) ;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab, "Robot X Position", new ShuffleBoardData<Double>(m_cameraSubsystem.getRobotPosition().getX()));
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab, "Robot Y Position", new ShuffleBoardData<Double>(m_cameraSubsystem.getRobotPosition().getY()));
    }

    @Override 
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
