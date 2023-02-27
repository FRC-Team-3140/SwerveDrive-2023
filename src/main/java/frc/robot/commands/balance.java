// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class balance extends CommandBase {
  private final SwerveDrive m_swerve;

  private final double max_speed = 0.2;
  private final double max_angle = 10;
  
  private final PIDController pidController = new PIDController(max_speed / max_angle, 0.02, 0.04);

  /** Creates a new balance. */
  public balance(SwerveDrive swerveDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerveDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = m_swerve.getAngleFiltered();
    double speed = pidController.calculate(-angle);
    speed = Math.min(Math.max(speed, -max_speed), max_speed);
    m_swerve.setChassisSpeeds(speed, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
