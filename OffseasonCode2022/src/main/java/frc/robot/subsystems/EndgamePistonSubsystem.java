package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndgamePistonSubsystem extends SubsystemBase {

    private final Solenoid m_solenoid;

    public EndgamePistonSubsystem(int solenoidID) {
        m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, solenoidID);
    }

    public void setEndgameSolenoidState(boolean state) {
        m_solenoid.set(state);
    }

    public boolean getEndgameSolenoidState() {
        return m_solenoid.get();
    }
}
