// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.Camera;

public class Pathfinding extends Command implements Constants {
  private Pose2d updatedPose;
  private Command pathfindingCommand;

  /**
   * Creates a new Pathfinding.
   * 
   * @param Camera
   */
  public Pathfinding(Pose2d updatedRobotPose, Camera camera, SwerveDrive swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(camera, swerve);

    updatedPose = updatedRobotPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    // Make sure this is only scheduled when the last Pathfining command is complete!
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*Make sure this command has an end state when the current swerve Pose is equal to the 
      targetPose*/
    if (swerve.getPose() == targetPose) {
      return true;
    } else {
      return false;
    }
  }
}
