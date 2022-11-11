package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.utilities.SparkMaxWrapper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase{

    private SparkMaxWrapper m_centeringMotor;
    private SparkMaxWrapper m_rollerMotor;

    public IntakeSubsystem(int rollerID, int centeringID) {
        m_rollerMotor = new SparkMaxWrapper(rollerID, MotorType.kBrushless);
        m_centeringMotor = new SparkMaxWrapper(centeringID, MotorType.kBrushless);


        m_rollerMotor.restoreFactoryDefaults();
        m_centeringMotor.restoreFactoryDefaults();
        
        this.followIntakeMotors();
    }

    public void followIntakeMotors() {
        m_rollerMotor.setInverted(false);
        //m_centeringMotor.follow(m_rollerMotor, true);
    }

    public void setRollerMotorSpeed(double speed) {
        m_rollerMotor.set(speed);
        m_centeringMotor.set(-speed);
    }

    public void stopRollerMotor() {
        m_rollerMotor.set(0);
        m_centeringMotor.set(0);
    }

    public double getrollerMotorSpeed() {
        return m_rollerMotor.get();
    }

}
