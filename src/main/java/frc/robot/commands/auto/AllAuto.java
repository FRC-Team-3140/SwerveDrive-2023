package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auto2.Arm.ArmPosition;
import frc.robot.commands.Auto2.Arm.WristPosition;
import frc.robot.commands.Balance.BalanceTogether;
//import frc.robot.commands.Drivetrain.TurnDegrees;
import frc.robot.subsystems.Arm.Arm;
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
        //new SequentialCommandGroup(
        //    new ArmPosition(arm, 150),
        //    new WristPosition(wrist, 58),
        //    new InstantCommand(() -> claw.clawOpen()),
        //    new ParallelCommandGroup(
        //        new TurnDegrees(m_drive, 180, true),
        //        new SequentialCommandGroup(new WaitCommand(.5),
        //            new ParallelCommandGroup(
        //                new ArmPosition(arm, 20),
        //                new WristPosition(wrist, 170)
        //            )
        //        )
        //            
        //    ),
        //    new BalanceTogether(m_drive)
        //).schedule();
    }

}