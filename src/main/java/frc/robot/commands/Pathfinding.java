// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.SwerveDrive;

public class Pathfinding extends Command implements Constants {
  private Pose2d updatedPose;
  private Command pathfindingCommand;
  private SwerveDrive swerveDrive;

  /**
   * Creates a new Pathfinding.
   * 
   * @param Camera
   */
  public Pathfinding(Pose2d updatedRobotPose, Camera camera, SwerveDrive swerve) {
    swerve.pathfinding = true;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(camera, swerve);

    updatedPose = updatedRobotPose;
    swerveDrive = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // The following line IS TO BE REMOVED after testing and command is functional.
    // The sole perpose of this line is to tell the robot that it's inside of the
    // boundaries so it doesn't
    // perform any random triangular paths in attempt to get within bouds on the
    // telemetry page of Pathplanner. - TK
    swerveDrive.resetPose(new Pose2d(3.5, 6.5, new Rotation2d(0)));
    
    // This will prevent Pathplanner from mirroring the generated camera path
    // once the Pathfinding command hits it's end state it will be allowed to 
    // path mirror again. - TK
    swerveDrive.allowPathMirroring = false;

    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        maxSpeed, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    pathfindingCommand = AutoBuilder.pathfindToPose(
        updatedPose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
    );

    if (!pathfindingCommand.isScheduled()) {
      pathfindingCommand.schedule();
      System.out.println("!!!!!!!!!!!!!!!!!!!!\nScheduled Pathfinding\n!!!!!!!!!!!!!!!!!!!!");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Following Path...");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.allowPathMirroring = true; 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
     * Make sure this command has an end state when the current swerve Pose is equal
     * to the
     * targetPose
     */
    if (swerveDrive.getPose() == updatedPose) {
      return true;
    } else {
      return false;
    }
  }
}
