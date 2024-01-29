package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

public class ChoreoAuto extends Command{
      Field2d m_field = new Field2d();

    ChoreoTrajectory traj;
    SwerveDrive m_robotDrive;
    Command swerveCommand;

    public ChoreoAuto(String path){
        m_robotDrive = RobotContainer.m_robotDrive;
        traj = Choreo.getTrajectory(path);
        m_field.getObject("traj").setPoses(traj.getInitialPose(), traj.getFinalPose());
        m_field.getObject("trajPoses").setPoses(traj.getPoses());

    var thetaController = new PIDController(0, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    m_robotDrive.resetPose(traj.getInitialPose());
    swerveCommand = Choreo.choreoSwerveCommand(
        traj, // Choreo trajectory from above
        m_robotDrive::getPose, // A function that returns the current field-relative pose of the robot: your
                               // wheel or vision odometry
        new PIDController(0, 0.0, 0.0), // PIDController for field-relative X
                                                                                   // translation (input: X error in meters,
                                                                                   // output: m/s).
        new PIDController(0, 0.0, 0.0), // PIDController for field-relative Y
                                                                                   // translation (input: Y error in meters,
                                                                                   // output: m/s).
        thetaController, // PID constants to correct for rotation
                         // error
        (ChassisSpeeds speeds) -> m_robotDrive.drive( // needs to be robot-relative
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            false),
        m_robotDrive::shouldFlipPath, // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
        m_robotDrive // The subsystem(s) to require, typically your drive subsystem only
    );

        
    }
    @Override
    public void initialize() {
            Commands.sequence(
        Commands.runOnce(() -> m_robotDrive.resetPose(traj.getInitialPose())),
        swerveCommand,
        m_robotDrive.run(() -> m_robotDrive.drive(0, 0, 0, false))).schedule();

    }
}
