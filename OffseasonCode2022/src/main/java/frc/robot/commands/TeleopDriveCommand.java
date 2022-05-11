package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDriveCommand extends CommandBase {
    private final SwerveSubsystem drive;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    public TeleopDriveCommand(
            SwerveSubsystem swerveDrive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotSupplier) {
        drive = swerveDrive;
        translationXSupplier = xSupplier;
        translationYSupplier = ySupplier;
        rotationSupplier = rotSupplier;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        double translationXPercent = translationXSupplier.getAsDouble();
        double translationYPercent = translationYSupplier.getAsDouble();
        double rotationPercent = rotationSupplier.getAsDouble();

        drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                translationXPercent * SwerveSubsystem.MAX_LINEAR_VELOCITY_METERS_PER_SECOND,
                translationYPercent * SwerveSubsystem.MAX_LINEAR_VELOCITY_METERS_PER_SECOND,
                rotationPercent * SwerveSubsystem.MAX_ANGULAR_VELOCITY_METERS_PER_SECOND, drive.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
