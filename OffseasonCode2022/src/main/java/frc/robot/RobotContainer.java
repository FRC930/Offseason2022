// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer {
  private final SwerveSubsystem drivetrain = new SwerveSubsystem();

  private final XboxController driverController = new XboxController(0);

  public RobotContainer() {
    drivetrain.setDefaultCommand(new TeleopDriveCommand(drivetrain, () -> -modifyAxis(driverController.getLeftY()),
        () -> -modifyAxis(driverController.getLeftX()), () -> -modifyAxis(driverController.getRightX())));
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
