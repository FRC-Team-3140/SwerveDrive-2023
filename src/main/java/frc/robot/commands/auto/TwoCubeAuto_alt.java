// package frc.robot.commands.auto;

// import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.Auto2.TurnDegrees;
// import frc.robot.commands.Drivetrain.DriveToObject;
// import frc.robot.commands.Vision.TurnToObject;
// import frc.robot.subsystems.Arm.Arm;
// import frc.robot.subsystems.Arm.Claw;
// import frc.robot.subsystems.Arm.Wrist;
// import frc.robot.subsystems.Swerve.SwerveDrive;

// public class TwoCubeAuto_alt extends CommandBase{
//     SwerveDrive swervedrive;
//     Arm arm;
//     Wrist wrist;
//     Claw claw;
//     public TwoCubeAuto_alt(SwerveDrive swerveDrive, Arm arm, Wrist wrist, Claw claw){
//         this.swervedrive = swerveDrive;
//         this.arm = arm;
//         this.wrist = wrist;
//         this.claw = claw;
//     }
//     @Override
//     public void initialize() {
//         new SequentialCommandGroup(
//             new ScoreGamePieceTop(swervedrive, arm, wrist, claw),
//             new TurnDegrees(swervedrive, 180, true),
//             new TurnToObject(swervedrive),
//             new DriveToObject(swervedrive),
//             new TurnToObject(swervedrive),
//             new 
//         ).schedule();
//     }
// }
