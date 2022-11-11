package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeVoltageSubsystem;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;

public class IntakeStateCommand extends CommandBase{
    
    private IntakeVoltageSubsystem m_intakeVoltageSubsystem;
    // Whether the intake is retracting or extending
    private boolean m_retract;
    private boolean m_hold;
    private Debouncer m_Debouncer;
    

    public IntakeStateCommand(IntakeVoltageSubsystem intakeVoltageSubsystem, boolean retract) {
        m_intakeVoltageSubsystem = intakeVoltageSubsystem;
        addRequirements(m_intakeVoltageSubsystem);
        m_retract = retract;
        m_Debouncer = new Debouncer(0.3);
    }
   
    @Override
    public void initialize(){
        m_hold = false;
        //System.out.println("Init()");
    }

    @Override
    public void execute() {
        double speed = (m_retract)?-1.0:1.0;
        // Volts
        double lowSpeed = speed * 1.6;
        double highSpeed = speed * 7.0;

        if(m_hold){
            speed = lowSpeed;
        }
        else{
            SmartDashboard.putNumber("current", m_intakeVoltageSubsystem.getCurrent());
            if(m_Debouncer.calculate(m_intakeVoltageSubsystem.getCurrent() > 40.0)){
                SmartDashboard.putBoolean("In Hold", true);
                speed = lowSpeed;
                m_hold = true;
                //System.out.println("Slowed down  Speed: " + speed + "Current:" + m_intakeVoltageSubsystem.getCurrent());
            } else {
                speed = highSpeed;
                SmartDashboard.putBoolean("In Hold", false);
                //System.out.println("Speed: "+ speed + "Current:" + m_intakeVoltageSubsystem.getCurrent());
            }
            m_intakeVoltageSubsystem.setSpeed(speed);
        }
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void  end() {
        m_intakeVoltageSubsystem.setSpeed(0.0);
    }
}
