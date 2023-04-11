package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Auto2.TurnDegrees;
import frc.robot.commands.Auto2.Arm.ArmPosition;
import frc.robot.commands.Auto2.Arm.WristPosition;
import frc.robot.commands.Balance.BalanceTogether;
import frc.robot.commands.Drivetrain.EncoderDriveDistance;
import frc.robot.commands.Vision.TargetAlign;
import frc.robot.commands.Vision.VisionDistance;
//import frc.robot.commands.Drivetrain.TurnDegrees;
import frc.robot.subsystems.Arm.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Arm.Wrist;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class AllAuto extends CommandBase{
    SwerveDrive m_drive;
    Claw claw;
    Arm arm;
    Wrist wrist;

    public AllAuto(SwerveDrive swerveDrive, Claw claw, Arm arm, Wrist wrist, boolean headless){
        addRequirements(swerveDrive, arm, claw, wrist);
        m_drive = swerveDrive;
        this.claw = claw;
        this.arm = arm;
        this.wrist = wrist;

        SwerveDrive.headless = headless;
    
       
    }
    @Override
    public void initialize() {
        new SequentialCommandGroup(
        //     new ParallelCommandGroup(
        //         new ArmPosition(arm, 150),
        //     new SequentialCommandGroup(
        //         new WaitCommand(1.2),
        //         new WristPosition(wrist, 58))
        //     ),
        //    new DriveToWall(m_drive),
        //    new OpenClaw(claw),
        //    new EncoderDriveDistance(m_drive, .4, -.5, 0),
        new TargetAlign(m_drive),
        new WristPosition(wrist, 190),
       new ParallelCommandGroup(
           new ArmPosition(arm, 163),
           new SequentialCommandGroup(
               new WaitCommand(1.2),
               new WristPosition(wrist, 60)
           )
       ),
      // new InstantCommand(()-> {Stage.setString("DriveToWallStart");}),
      new DriveToWall(m_drive).withTimeout(2.5),
      new WaitCommand(.05),
   //Make sure turn and drive works
   //    new ArmTopAuto(arm, wrist),
      new OpenClaw(claw),
      new EncoderDriveDistance(m_drive, .75, -.1, 0).withTimeout(1),
      new VisionDistance(m_drive, 1.05), //new EncoderDistance(swerve, .05, -.1, 0).withTimeout(1),
    //  new InstantCommand(()-> {Stage.setString("Retract Arm");}),
    //   new ParallelCommandGroup(
    //    new SequentialCommandGroup( 
    //        new WristPosition(wrist, 84),
    //        new ParallelCommandGroup(
    //          new ArmPosition(arm, 39),
    //          new WristPosition(wrist, 168)
    //    ))),
           new ParallelCommandGroup(
               //new TurnDegrees(m_drive, 180, true),
               new WristPosition(wrist, 170),
               new SequentialCommandGroup(
                new WaitCommand(1),
                new ArmPosition(arm, 20)        
               )
                   
          ),
           new CloseClaw(claw),
           new EncoderDriveDistance(m_drive, .2, .1, 0),
           new BalanceTogether(m_drive,1)
        ).schedule();
    }

}