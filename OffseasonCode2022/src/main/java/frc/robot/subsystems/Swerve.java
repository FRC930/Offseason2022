package frc.robot.subsystems;
import com.ctre.phoenix.sensors.Pigeon2;

import frc.lib.util.SwerveModuleConstants;
import frc.robot.SwerveModule;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {

    // ----- CONSTANTS -----\\

    public static final int pigeonID = 13;
    public static final double trackWidth = Units.inchesToMeters(23);
    public static final double wheelBase = Units.inchesToMeters(23);
    public static final double maxSpeed = 4.5; //4.9; //meters per second
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
    
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = 180; //Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 180; // Math.PI;
            
    public static final double kPXController = 0.4; //0.076301;
    public static final double kPYController = 0.4; //0.076301;
    public static final double kPThetaController = SwerveModule.angleKP;
    public static final double kDThetaController = SwerveModule.angleKD;
            
    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public SwerveModulePosition[] mSwerveModPositions;
    public Pigeon2 gyro;
    public PIDController autoXController;
    public PIDController autoYController;
    public PIDController autoThetaController;
    public SwerveModule swerveModule1;
    public SwerveModule swerveModule2;
    public SwerveModule swerveModule3;
    public SwerveModule swerveModule4;

    public Swerve(SwerveModuleConstants frontLeftModuleConstants, SwerveModuleConstants frontRightModuleConstants, SwerveModuleConstants backLeftModuleConstants, SwerveModuleConstants backRightModuleConstants) {
        gyro = new Pigeon2(pigeonID);
        //System.out.println(gyro.getTemp());
        gyro.configFactoryDefault();
        zeroGyro();

        autoXController = new PIDController(kPXController, 0, 0);
        autoYController = new PIDController(kPYController, 0, 0);
        autoThetaController = new PIDController(
            0.33, 0, 0);
        
        swerveOdometry = new SwerveDriveOdometry(swerveKinematics, getYaw(), mSwerveModPositions);

        swerveModule1 = new SwerveModule(0, frontLeftModuleConstants);
        swerveModule2 = new SwerveModule(1, frontRightModuleConstants);
        swerveModule3 = new SwerveModule(2, backLeftModuleConstants);
        swerveModule4 = new SwerveModule(3, backRightModuleConstants);

        mSwerveMods = new SwerveModule[] {  
            swerveModule1,
            swerveModule2,
            swerveModule3,
            swerveModule4
        };

        mSwerveModPositions = new SwerveModulePosition[] {
            swerveModule1.getPosition(),
            swerveModule2.getPosition(),
            swerveModule3.getPosition(),
            swerveModule4.getPosition()
        };
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), mSwerveModPositions, pose);
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    public SwerveDriveKinematics getSwerveKinematics() {
        return swerveKinematics;
    }

    public PIDController getAutoXController() {
        return autoXController;
    }

    public PIDController getAutoYController() {
        return autoYController;
    }

    public PIDController getAutoThetaController() {
        return autoThetaController;
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), mSwerveModPositions);  

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond); 
        }
        SmartDashboard.putNumber("X", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Y", swerveOdometry.getPoseMeters().getY());  
    }
}