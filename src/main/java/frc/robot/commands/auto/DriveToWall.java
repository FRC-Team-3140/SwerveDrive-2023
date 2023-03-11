// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class DriveToWall extends CommandBase {
  SwerveDrive m_drive;
  Accelerometer accelerometer;
  double Threshold = 0.1;
  /** Creates a new DriveToEnd. */
  public DriveToWall(SwerveDrive m_drive) {
    this.m_drive = m_drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    accelerometer = m_drive.getAccelerometer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(accelerometer.getX() <= Threshold){
      m_drive.setChassisSpeeds(0, 0, 0);
    } else {
      m_drive.setChassisSpeeds(0.1, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(accelerometer.getX() <= Threshold){
      return true;
    }else{return false;}
  }
}
