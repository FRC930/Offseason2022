/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ShooterUtility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

//-------- SUBSYSTEM CLASS --------\\

/**
 * <h3>ShooterSubsystem</h3>
 * Subsystem class to manage the shooter wheels.
 */
public class ShooterMotorSubsystem extends SubsystemBase {

    // -------- DECLARATIONS --------\\
    // motor controllers for the shooter wheels
    private final WPI_TalonFX RightShooterMaster;
    private final WPI_TalonFX LeftShooterFollower;
    private final WPI_TalonFX Loaded;

    // -------- CONSTRUCTOR --------\\
    /**
     * <h3>ShooterSubsystem</h3>
     * Creates a subsystem class to manage the shooter.
     * 
     * @param RightShooterMasterID   ID of the bottom motor master of the shooter
     * @param LeftShooterFollowerID ID of the bottom motor follower of the shooter
     * 
     */
    public ShooterMotorSubsystem(int RightShooterMasterID,
            int LeftShooterFollowerID, int LoadedID) {

        // Motor declaration
        RightShooterMaster = new WPI_TalonFX(RightShooterMasterID);
        LeftShooterFollower = new WPI_TalonFX(LeftShooterFollowerID);
        Loaded = new WPI_TalonFX(LoadedID);

        // Reset motors
        RightShooterMaster.configFactoryDefault();
        LeftShooterFollower.configFactoryDefault();
        Loaded.configFactoryDefault();

        // Sets motors to coast so that they can move freely when neutral
        RightShooterMaster.setNeutralMode(NeutralMode.Coast);
        LeftShooterFollower.setNeutralMode(NeutralMode.Coast);
        Loaded.setNeutralMode(NeutralMode.Coast);

        // All motors are not inverted
        refollowShooterMotors();

        // Need to bring shooter values with 0 to prevent null pointer
        ShooterUtility.setValuesToShuffleboard(0.0);
    }

    /**
     * <h3>refollowShooterMotors</h3>
     * Refollows the bottom shooter motors.
     */
    public void refollowShooterMotors() {
        LeftShooterFollower.follow(RightShooterMaster, FollowerType.PercentOutput);
        LeftShooterFollower.setInverted(true);
    }

    // -------- METHODS --------\\
    /**
     * <h3>setBottomSpeed</h3>
     * Sets the bottom shooter motor speed.
     *
     * @param speed speed of the bottom wheel in percent output
     */
    public void setRightSpeed(double rightShooterSpeed, double loadedSpeed) {
        // Sets speed to 0 if speed argument is less than 0
        RightShooterMaster.set(ControlMode.PercentOutput, Math.max(0.0, rightShooterSpeed));
        Loaded.set(ControlMode.PercentOutput, Math.max(0.0, loadedSpeed));
    }


    /**
     * <h3>stopMotors</h3>
     * Stops both shooter motors.
     */
    public void stopMotors() {
        RightShooterMaster.stopMotor();
        Loaded.stopMotor();
    }

    /**
     * <h3>getBottomSpeed</h3>
     * Gets the bottom shooter motor speed.
     *
     * @return speed of the bottom wheel in percent output
     */
    public double getBottomSpeed() {
        return RightShooterMaster.getMotorOutputPercent();
        

    }

    
} // end of class ShooterSubsystem