package frc.robot.autos;

import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TaxiTwoBall extends SequentialCommandGroup {
    public TaxiTwoBall(Swerve s_Swerve) {

        // An example trajectory to follow.  All units in meters.
        PathPlannerTrajectory pathPlannerExample = PathPlanner.loadPath("TaxiTwoBall", 1, 2.5, true);
        Trajectory exampleTrajectory = pathPlannerExample;

        var thetaController = s_Swerve.getAutoThetaController();
        thetaController.enableContinuousInput(-180.0, 180.0); //-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveControllerCommand = 
            new PPSwerveControllerCommand(
                pathPlannerExample,
                s_Swerve::getPose,
                s_Swerve.getSwerveKinematics(),
                s_Swerve.getAutoXController(),
                s_Swerve.getAutoYController(),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
}