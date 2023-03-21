package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.EncoderDriveDistance;
import frc.robot.commands.Drivetrain.TurnDegrees;
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
            new ScoreGamePieceTop(swerve, arm, wrist, claw),
            new TurnDegrees(swerve, 180, false),
            new EncoderDriveDistance(swerve, 1, 0.5,0)
        );
    }

    @Override
    public void end(boolean interrupted) {
        
    }
    
}
