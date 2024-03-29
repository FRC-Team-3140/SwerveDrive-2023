// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class TurnAndDrive extends CommandBase {
  /** Creates a new TurnAndDrive. */
  SwerveDrive swerve;
  double DriveDistance = 0.0;
  double startPosition;
  
  double targetAngle;
  double turnSpeed;
  double distance;
  double xSpeed;
  double ySpeed;
  boolean headless = true;

  public TurnAndDrive(SwerveDrive swerve, double dist, double turnDegrees, double xSpeed, double ySpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);
    targetAngle = turnDegrees;
    this.distance = dist;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    SwerveDrive.headless = headless;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //arc length formula (theta * radius where radius is half of the diagonal of the robot)
    double bot_length = .3302;
    double bot_width = .2794;
    distance = distance + targetAngle/180 * Math.PI * Math.hypot(bot_length, bot_width)/2;

    startPosition = swerve.getPosition();
    swerve.setLocked(false);
    turnSpeed = Math.copySign(.35, targetAngle - SwerveDrive.m_gyro.getAngle());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(swerve.getPosition() - startPosition) > Math.abs(distance)){
      xSpeed = 0;
      ySpeed = 0;
      System.out.println("Reached Didstance");
    }
    if(Math.abs(targetAngle - (SwerveDrive.m_gyro.getYaw())) <= 5){
      turnSpeed = 0;
      System.out.println("Reached ngle");
    }
    swerve.setChassisSpeeds(xSpeed, ySpeed,turnSpeed);
  }
  
  /*************************Random Comments From Random People That I Don't Know...*****************************************************/
  
  // I am your god, bow down to me my children - Joseph
  // There Once Was a not safe for work comment here form Pragya... - Jonathan & Tyler P.S. Pragya deleted it so she wouldn't get in trouble
  // Mechenical was here -Kaitlyn

  /*************************End Of Random Comments From Random People That I Don't Know...**********************************************/
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setChassisSpeeds(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(targetAngle - (SwerveDrive.m_gyro.getYaw())) <= 5 && Math.abs(swerve.getPosition() - startPosition) > Math.abs(distance);
  }
}
