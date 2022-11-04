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

public class IntakeExtensionSubsystem extends SubsystemBase{
    
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
     * <h3>IntakeEncoderSubsystem</h3>
     * Creates a subsystem class to manage the intake motor.
     * 
     * @param intakeMotorID ID of the intake motor
     */
    public IntakeExtensionSubsystem(int intakeMotorID) {

        // Motor declaration
        intakeMotor = new WPI_TalonFX(intakeMotorID);

        // Config object for all config values
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Set PID values
        config.slot0.kP = MOTOR_KP;
        config.slot0.kD = MOTOR_KD;


        // Allow encoder deadband to prevent oscillation
        config.slot0.allowableClosedloopError = 25;


        // Set current and voltage control for brake mode
        config.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 40, 40, 0.2);
        config.statorCurrLimit.enable = true;

        config.voltageCompSaturation = 11.0;

        // Sets encoder limits so the intake can't break itself by going too far
       // config.forwardSoftLimitThreshold = ((EXTENDED_POSITION / 360.0) * TALON_CPR / GEAR_RATIO);
       // config.reverseSoftLimitThreshold = ((RETRACTED_POSITION / 360.0) * TALON_CPR / GEAR_RATIO);
       // config.forwardSoftLimitEnable = true;
       // config.reverseSoftLimitEnable = true;


        // Sets motor so it can't be manually moved when neutral
        intakeMotor.setNeutralMode(NeutralMode.Brake);

        // Motor is not inverted
        intakeMotor.setInverted(InvertType.None);

        //Repeats sending the config until successful
        ErrorCode error = ErrorCode.CAN_INVALID_PARAM;
        do {
            error = intakeMotor.configAllSettings(config, 500);
            //System.out.println("Trying!");
        } while (error != ErrorCode.OK);
    }

    public void extendIntake() {
         // Converts degrees into encoder ticks
         intakeMotor.set(ControlMode.Position, EXTENDED_POSITION,
         DemandType.ArbitraryFeedForward, 0.7);
    }

    public void retractIntake() {
         // Converts degrees into encoder ticks
         intakeMotor.set(ControlMode.Position, RETRACTED_POSITION,
         DemandType.ArbitraryFeedForward, 0.8);
    }

}
// end of class IntakeEncoderSubsystem

