package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase{

    private CANSparkMax m_extensionMotor;
    private CANSparkMax m_centeringMotor;
    private CANSparkMax m_rollerMotor;

    public IntakeSubsystem(int extensionID, int rollerID, int centeringID) {
        m_extensionMotor = new CANSparkMax(extensionID, MotorType.kBrushless);
        m_rollerMotor = new CANSparkMax(rollerID, MotorType.kBrushless);
        m_centeringMotor = new CANSparkMax(centeringID, MotorType.kBrushless);

        m_extensionMotor.restoreFactoryDefaults();
        m_rollerMotor.restoreFactoryDefaults();
        m_centeringMotor.restoreFactoryDefaults();

        m_rollerMotor.setInverted(false);
        m_centeringMotor.follow(m_rollerMotor);

    }

    public void setExtensionMotorSpeed(double speed) {
        m_extensionMotor.set(speed);
    }

    public void setRollerMotorSpeed(double speed) {
        m_rollerMotor.set(speed);
    }

    public void stopExtensionMotor(double speed) {
        m_extensionMotor.set(0);
    }

    public void stopRollerMotor(double speed) {
        m_rollerMotor.set(0);
    }

    public double getExtensionMotorSpeed() {
        return m_extensionMotor.get();
    }

    public double getrollerMotorSpeed() {
        return m_rollerMotor.get();
    }

    public void setExtensionMotorPosition(double position) {
        m_extensionMotor.set(position);
    }

    public void setRollerMotorPosition(double position) {
        m_rollerMotor.set(position);
    }

}
