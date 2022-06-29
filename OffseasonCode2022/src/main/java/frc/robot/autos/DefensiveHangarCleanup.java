package frc.robot.autos;

import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DefensiveHangarCleanup extends SequentialCommandGroup {
    public DefensiveHangarCleanup(Swerve s_Swerve) {

        // An example trajectory to follow.  All units in meters.
        PathPlannerTrajectory path1 = PathPlanner.loadPath("DefensiveHangarCleanup1", 1, 2.5, true);
        Trajectory path1Trajectory = path1;
        PathPlannerTrajectory path2 = PathPlanner.loadPath("DefensiveHangarCleanup2", 1, 2.5, false);
        Trajectory path2Trajectory = path2;
        PathPlannerTrajectory path3 = PathPlanner.loadPath("DefensiveHangarCleanup3", 1, 2.5, true);
        Trajectory path3Trajectory = path3;

        var thetaController = s_Swerve.getAutoThetaController();
        thetaController.enableContinuousInput(-180.0, 180.0); //-Math.PI, Math.PI);

        PPSwerveControllerCommand path1Command = 
            new PPSwerveControllerCommand(
                path1,
                s_Swerve::getPose,
                s_Swerve.getSwerveKinematics(),
                s_Swerve.getAutoXController(),
                s_Swerve.getAutoYController(),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        PPSwerveControllerCommand path2Command = 
            new PPSwerveControllerCommand(
                path2,
                s_Swerve::getPose,
                s_Swerve.getSwerveKinematics(),
                s_Swerve.getAutoXController(),
                s_Swerve.getAutoYController(),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        PPSwerveControllerCommand path3Command = 
            new PPSwerveControllerCommand(
                path3,
                s_Swerve::getPose,
                s_Swerve.getSwerveKinematics(),
                s_Swerve.getAutoXController(),
                s_Swerve.getAutoYController(),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(path1Trajectory.getInitialPose())),
            path1Command,
            path2Command,
            path3Command
        );
    }
}