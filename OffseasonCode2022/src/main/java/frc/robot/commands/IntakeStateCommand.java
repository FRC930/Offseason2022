package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeVoltageSubsystem;

public class IntakeStateCommand extends CommandBase{
    
    private IntakeVoltageSubsystem m_intakeVoltageSubsystem;
    // Whether the intake is retracting or extending
    private boolean m_retract;
    

    public IntakeStateCommand(IntakeVoltageSubsystem intakeVoltageSubsystem, boolean retract) {
        m_intakeVoltageSubsystem = intakeVoltageSubsystem;
        addRequirements(m_intakeVoltageSubsystem);
        m_retract = retract;
    }
   
    @Override
    public void execute() {
        double speed = (m_retract)?-1.0:1.0;
        if(m_intakeVoltageSubsystem.getCurrent() > 60){
            //TODO check speeds, commented ones are Citrus' speeds
            //TODO put shuffleboard values
            speed *= 0.5; //1.2;
        } else {
            speed *= 2.0; //4.0;
        }
        m_intakeVoltageSubsystem.setSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void  end() {
        m_intakeVoltageSubsystem.setSpeed(0.0);
    }
}
