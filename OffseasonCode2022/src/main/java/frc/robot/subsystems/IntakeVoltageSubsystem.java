package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

public class IntakeVoltageSubsystem extends SubsystemBase{
    
    // -------- CONSTANTS --------\\
    // Clicks of the TalonFX encoder per rotation of motor shaft
    private static final double TALON_CPR = 2048.0;
    // Gear ratio from motor to the hood(if the hood were a full circle)
    // private static final double GEAR_RATIO = (16.0 / 36.0) * (15.0 / 235.0);
    // Position when the intake is extended
    private static final double EXTENDED_POSITION = 0.0;
    // Position when the intake is retracted
    private static final double RETRACTED_POSITION = -5050.0;
    // PID values
    private static final double MOTOR_KP = 5.0;
    private static final double MOTOR_KD = 15;

    // -------- DECLARATIONS --------\\
    private final WPI_TalonFX intakeMotor;

    // -------- CONSTRUCTOR --------\\
    /**
     * <h3>IntakeVoltageSubsystem</h3>
     * Creates a subsystem class to manage the intake motor.
     * 
     * @param intakeMotorID ID of the intake motor
     */
    public IntakeVoltageSubsystem(int intakeMotorID) {

        // Motor declaration
        intakeMotor = new WPI_TalonFX(intakeMotorID);

        // Config object for all config values
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Set current and voltage control for brake mode
        config.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 40, 40, 0.2);
        config.statorCurrLimit.enable = true;

        // Sets motor so it can't be manually moved when neutral
        intakeMotor.setNeutralMode(NeutralMode.Coast);

        // Motor is not inverted
        intakeMotor.setInverted(InvertType.None);

        //Repeats sending the config until successful
        ErrorCode error = ErrorCode.CAN_INVALID_PARAM;
        do {
            error = intakeMotor.configAllSettings(config, 500);
            //System.out.println("Trying!");
        } while (error != ErrorCode.OK);
    }

    public void setSpeed(double speed) {

         intakeMotor.set(ControlMode.PercentOutput, speed / 12.0);
        //  System.out.println("Speed: " + speed);
    }

    public double getCurrent() {

        return intakeMotor.getStatorCurrent();
    }

    @Override
    public void periodic() {
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab, 
                                                                ShuffleboardKeys.INTAKE_POSITIONING, 
                                                                new ShuffleBoardData<Double>(getCurrent()));
        // System.out.println(getCurrent());
    }

}
// end of class IntakeVoltageSubsystem

