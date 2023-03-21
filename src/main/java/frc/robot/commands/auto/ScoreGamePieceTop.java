// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auto2.Arm.ArmTopAuto;
import frc.robot.commands.Drivetrain.EncoderDriveDistance;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Arm.Wrist;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class ScoreGamePieceTop extends CommandBase {
  /** Creates a new ScoreGamePieceTop. */
  SwerveDrive swerve;
  Arm arm;
  Wrist wrist; 
  Claw claw; 

  public ScoreGamePieceTop(SwerveDrive swerveIn, Arm armIn, Wrist wristIn, Claw clawIn) {
    this.swerve = swerveIn;
    this.arm = armIn;
    this.wrist = wristIn;
    this.claw = clawIn;
    addRequirements(swerve, arm, wrist, claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new SequentialCommandGroup(
      new ArmTopAuto(arm, wrist),
      new InstantCommand(() -> claw.clawOpen()),
      new ParallelCommandGroup(
        new RetractArm(arm, wrist), 
        new EncoderDriveDistance(swerve, -1, 0.6, 0)
      )
    );
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
