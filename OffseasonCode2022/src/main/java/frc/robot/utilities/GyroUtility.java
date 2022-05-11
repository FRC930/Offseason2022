package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class GyroUtility {
    // ----- SINGLETON IMPLEMENTATION ----- \\

    private static GyroUtility instance;

    private GyroUtility() {

    }

    public static GyroUtility getInstance() {
        if (instance == null) {
            instance = new GyroUtility();
        }
        return instance;
    }

    // ----- CLASS IMPLEMENTATION ----- \\
    private PigeonIMU m_gyro = new PigeonIMU(new TalonSRX(17));

    public PigeonIMU getGyro() {
        return m_gyro;
    }
}
