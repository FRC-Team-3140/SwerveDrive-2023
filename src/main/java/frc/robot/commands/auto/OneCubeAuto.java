package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drivetrain.DriveDistance;
import frc.robot.commands.Auto2.Arm.ArmPosition;
import frc.robot.commands.Auto2.Arm.WristPosition;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Arm.Wrist;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.subsystems.Arm.Arm;

public class OneCubeAuto extends CommandBase {
    SwerveDrive m_drive;
    Claw claw;
    Arm arm;
    Wrist wrist;
    
    public OneCubeAuto(SwerveDrive swerveDrive, Claw claw, Arm arm, Wrist wrist, boolean headless){
        addRequirements(swerveDrive, arm, claw);
        m_drive = swerveDrive;
        this.claw = claw;
        this.arm = arm;
        this.wrist = wrist;

        SwerveDrive.headless = headless;
    }
    @Override
    public void initialize() {
        new SequentialCommandGroup(
            new InstantCommand(()-> {SwerveDrive.m_gyro.reset();}),
            new InstantCommand(() -> {System.out.println("Arm top");}), 
            //new ArmTopAuto(arm),
           // new ArmPosition(arm, -55),
            new ArmPosition(arm, 150),
            new WristPosition(wrist, 58),
            new DriveToWall(m_drive),
            new InstantCommand(() -> {
                System.out.println("Open claw");
                claw.clawOpen();
            }),
            new InstantCommand(() -> {claw.holdCube();}),
            
            new WaitCommand(1),
            new ParallelCommandGroup(
                new WristPosition(wrist, 168),
                new DriveDistance(m_drive, -1.5, -.6, 0)
                ),
            new ArmPosition(arm, 34),
            new WaitCommand(.1),
            new InstantCommand(() -> {
                System.out.println("stop");
                m_drive.setChassisSpeeds(0, 0, 0);
            }),
            new InstantCommand(() -> {
                System.out.println("Close claw");
                claw.clawClosed();
            })
            //new TurnDegrees(m_drive, 180, true),
           // new WaitCommand(2)
            //new DriveDistance(m_drive, 2, 0.5, 0)
            
        ).schedule();
    }



}
