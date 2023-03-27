// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

//import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class DriveToWall extends CommandBase {
  SwerveDrive m_drive;
  double Threshold = 1;
  boolean hasMoved = false;
  
  /** Creates a new DriveToEnd. */
  public DriveToWall(SwerveDrive m_drive) {
    this.m_drive = m_drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }
  long startTime = 0;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setLocked(false);
    startTime = System.currentTimeMillis();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_drive.setChassisSpeeds(0.2, 0, 0);
      if(System.currentTimeMillis() - startTime > 200){
        hasMoved = true;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setChassisSpeeds(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((SwerveDrive.m_gyro.getRawAccelY() * 9.8 > Threshold && hasMoved == true)){
      return true;
    }else{return false;}
  }
}
