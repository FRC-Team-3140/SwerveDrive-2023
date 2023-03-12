package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drivetrain.TurnDegrees;
import frc.robot.commands.reachTop;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.subsystems.Arm.Arm;

public class OneCubeAuto extends CommandBase {
    SwerveDrive m_drive;
    Claw claw;
    Arm arm;
    
    public OneCubeAuto(SwerveDrive swerveDrive, Claw claw, Arm arm){
        addRequirements(swerveDrive, arm, claw);
        m_drive = swerveDrive;
        this.claw = claw;
        this.arm = arm;
    }

    @Override
    public void initialize() {

        new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                
                new reachTop(arm)
            ),
            new InstantCommand(() -> {
                claw.clawOpen();
            }),
            new WaitCommand(1),
            new InstantCommand(() -> {
                m_drive.setChassisSpeeds(-.5, 0, 0);
            }),
            new WaitCommand(1),
            new InstantCommand(() -> {
                m_drive.setChassisSpeeds(0, 0, 0);
            }),
            new InstantCommand(() -> {
                claw.clawClosed();
            }),
            new TurnDegrees(m_drive, 180),
            new WaitCommand(2)
            
        );
    }


}