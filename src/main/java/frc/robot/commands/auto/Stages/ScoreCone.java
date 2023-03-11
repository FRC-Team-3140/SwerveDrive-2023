// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.auto.Stages;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.commands.auto.DriveToWall;
// import frc.robot.commands.auto.reachTop;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.SwerveDrive;
// import frc.robot.subsystems.Claw;
// import frc.robot.commands.auto.DriveToWall;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class ScoreCone extends SequentialCommandGroup {
//   /** Creates a new ScoreCone. */
//   public ScoreCone(Arm m_arm, SwerveDrive m_swerve, Claw m_claw) {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());

//     addCommands(
//       new reachTop(m_arm),
//       new WaitCommand(4),
//       new DriveToWall(m_swerve),
//       new WaitCommand(1),
//       new InstantCommand(()->{
//         m_claw.clawOpen();
//       }),
//       new WaitCommand(1),
//       new InstantCommand(()->{
//         m_claw.clawClosed();
//         m_swerve.setChassisSpeeds(-0.2, 0, 0);
//         }),
//       new WaitCommand(1),
//       new BalancePosition(m_arm, m_swerve, m_claw)
//     );
//   }
// }
