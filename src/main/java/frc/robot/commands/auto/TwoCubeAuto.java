package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.TurnAndDrive;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Arm.Wrist;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class TwoCubeAuto extends CommandBase{
    SwerveDrive swerve;
    Arm arm;
    Wrist wrist;
    Claw claw; 
    public TwoCubeAuto(SwerveDrive swerve, Arm arm, Wrist wrist, Claw claw){
        this.swerve = swerve;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        addRequirements(swerve, arm, wrist);

    }
    @Override
    public void initialize() {
        new SequentialCommandGroup(
        //    new ScoreGamePieceTop(swerve, arm, wrist, claw),
        //    new TurnAndDrive(swerve, 0, 0, 0, 0),
        //    new InstantCommand(() -> claw.clawClosed()),
           new TurnAndDrive(swerve, 1, 180, .5, 0)//,
        //    new EncoderDriveDistance(swerve, 0.3, .3, 0),
        //    new ScoreGamePieceTop(swerve, arm, wrist, claw)
        );
    }

    @Override
    public void end(boolean interrupted) {
        
    }
    
}
