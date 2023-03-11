package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TurnDegrees;
import frc.robot.commands.auto.reachTop;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Arm;

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
                new TurnDegrees(m_drive, 180),
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
