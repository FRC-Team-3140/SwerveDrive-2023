// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Balance;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class AutoOvershootEStop extends CommandBase {
  /** Creates a new AutoOvershootEStop. */
  SwerveDrive swerve;
  private double lastPos;
  private double stopPos = 1;
  private Timer timer;

  public AutoOvershootEStop(SwerveDrive swerveDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.hasElapsed(3)){
      if(Math.abs(swerve.getPosition()) > (stopPos + lastPos)){
        return true;
      } else {
        return false; 
      }
    } else {
      lastPos = Math.abs(swerve.getPosition());
      return false;
    }
  }
}
