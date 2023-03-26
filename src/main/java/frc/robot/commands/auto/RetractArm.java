// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auto2.Arm.ArmPosition;
import frc.robot.commands.Auto2.Arm.WristPosition;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Wrist;

public class RetractArm extends CommandBase {
  /** Creates a new RetractArm. */
  Arm arm;
  Wrist wrist;
  public RetractArm(Arm arm, Wrist wrist) {
    this.arm  = arm;
    this.wrist = wrist;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new SequentialCommandGroup( 
      new WristPosition(wrist, 84),
      new ParallelCommandGroup(
        new ArmPosition(arm, 34),
        new WristPosition(wrist, 168)
  )).schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Put Angles that get the arm to the almost "stowaway" position

    
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
