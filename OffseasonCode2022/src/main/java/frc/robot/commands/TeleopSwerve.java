package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class TeleopSwerve extends CommandBase {

    // ----- CONSTANTS -----\\

    /**
     *
     */
    private static final double SPEED_MULIPLE = 0.90;
    private double stickDeadband = 0.1;
    public static final double maxAngularVelocity = 11.5;
    public static final double maxSpeed = 4.9; //meters per second

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private CommandXboxController controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, CommandXboxController controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        double yAxis = -controller.getHID().getRawAxis(translationAxis);
        double xAxis = -controller.getHID().getRawAxis(strafeAxis);
        double rAxis = controller.getHID().getRawAxis(rotationAxis);
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < stickDeadband) ? 0 : rAxis;

        translation = new Translation2d(yAxis * SPEED_MULIPLE, xAxis * SPEED_MULIPLE).times(maxSpeed);
        rotation = -1 * rAxis * SPEED_MULIPLE * maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}
