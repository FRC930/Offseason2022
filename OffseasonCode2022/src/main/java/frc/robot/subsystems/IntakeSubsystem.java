package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase{

    private CANSparkMax m_centeringMotor;
    private CANSparkMax m_rollerMotor;

    public IntakeSubsystem(int rollerID, int centeringID) {
        m_rollerMotor = new CANSparkMax(rollerID, MotorType.kBrushless);
        m_centeringMotor = new CANSparkMax(centeringID, MotorType.kBrushless);


        m_rollerMotor.restoreFactoryDefaults();
        m_centeringMotor.restoreFactoryDefaults();
    }

    public void refollowIntakeMotors() {
        m_rollerMotor.setInverted(false);
        m_centeringMotor.follow(m_rollerMotor);
    }

    public void setRollerMotorSpeed(double speed) {
        m_rollerMotor.set(speed);
    }

    public void stopRollerMotor(double speed) {
        m_rollerMotor.set(speed);
    }

    public double getrollerMotorSpeed() {
        return m_rollerMotor.get();
    }

}
