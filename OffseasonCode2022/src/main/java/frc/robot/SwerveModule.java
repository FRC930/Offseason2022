package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {

    // ----- CONSTANTS -----\\
    
    private static final double wheelDiameter = Units.inchesToMeters(4);
    private static final double wheelCircumference = wheelDiameter * Math.PI;
    private static final double driveGearRatio = (6.86 / 1.0); //6.86:1
    private static final double angleGearRatio = (12.8 / 1.0); //12.8:1
    public static final double maxSpeed = 4.5; //4.9; //meters per second
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;
    public static final boolean driveMotorInvert = false;
    public static final boolean angleMotorInvert = false;
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.6;
    public static final double angleKI = 0.0;
    public static final double angleKD = 12.0;
    public static final double angleKF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.076301; //0.23; //0.1; // Changed characterization value of 2.7695 to 0.23 by dividing 2.7695 by 12;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = (0.62538 / 12); //(0.66791 / 12); //(0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
    public static final double driveKV = (0.059455 / 12); //(2.1522 / 12); //(2.44 / 12);
    public static final double driveKA = (0.0030568 / 12); //(0.23908 / 12); //(0.27 / 12);

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;    

    public int moduleNumber;
    private double angleOffset;
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;
    private double lastAngle;
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(driveKS, driveKV, driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){

        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            angleEnableCurrentLimit, 
            angleContinuousCurrentLimit, 
            anglePeakCurrentLimit, 
            anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = angleKP;
        swerveAngleFXConfig.slot0.kI = angleKI;
        swerveAngleFXConfig.slot0.kD = angleKD;
        swerveAngleFXConfig.slot0.kF = angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;


        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            driveEnableCurrentLimit, 
            driveContinuousCurrentLimit, 
            drivePeakCurrentLimit, 
            drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = driveKP;
        swerveDriveFXConfig.slot0.kI = driveKI;
        swerveDriveFXConfig.slot0.kD = driveKD;
        swerveDriveFXConfig.slot0.kF = driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = closedLoopRamp;

        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, wheelCircumference, driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, angleGearRatio)); 
        lastAngle = angle;
    }

    private void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(swerveAngleFXConfig);
        mAngleMotor.setInverted(angleMotorInvert);
        mAngleMotor.setNeutralMode(angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(swerveDriveFXConfig);
        mDriveMotor.setInverted(driveMotorInvert);
        mDriveMotor.setNeutralMode(driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), angleGearRatio));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState(){
        double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), wheelCircumference, driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), wheelCircumference, driveGearRatio),
            getAngle()
        );
    }
    
}