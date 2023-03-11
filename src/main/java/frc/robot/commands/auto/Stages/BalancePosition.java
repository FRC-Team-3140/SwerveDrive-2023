// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.Stages;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.DriveToWall;
import frc.robot.commands.auto.findPath;
import frc.robot.commands.auto.reachTop;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Claw;

public class BalancePosition extends SequentialCommandGroup {
  public BalancePosition(Arm m_arm, SwerveDrive m_swerve, Claw m_claw) {
    addCommands(
      new findPath(m_swerve)
    );
  }
}
