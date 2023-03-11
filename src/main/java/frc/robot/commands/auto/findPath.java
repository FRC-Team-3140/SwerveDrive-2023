// // Copyright (c) FIRST and other WPILib contributors.
// // Open Sou*rce Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.auto;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.commands.DriveDistance;
// import frc.robot.commands.TargetAlign;
// import frc.robot.subsystems.SwerveDrive;

// public class findPath extends CommandBase {
//   /** Creates a new findPath. */
//   SwerveDrive m_swerve;

//   public findPath(SwerveDrive swervey) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(swervey);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     new DriveDistance(m_swerve, 0.3, -0.2, 0);
//     new WaitCommand(1);
//     new TargetAlign(m_swerve);
//     new WaitCommand(2 );
//     switch (m_swerve.getTarget().getFiducialId()) {
//       //Blue Alliance
//       case 6:
//         new Blue1(m_swerve);
//         break;
//       case 7:
//         new Blue2(m_swerve);
//         break;

//       case 8:
//         new Blue3(m_swerve);
//         break;
//       //Red Alliance
//       case 3:
//         new Red1(m_swerve);
//         break;

//       case 2:
//         new Red2(m_swerve);
//         break;

//       case 1:
//         new Red3(m_swerve);
//         break;

//       default:
//         break;
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }

//Work on this
