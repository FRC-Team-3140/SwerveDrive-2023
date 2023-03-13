package frc.robot.commands.Auto2.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Claw;


import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drivetrain.TurnDegrees;
import frc.robot.commands.reachTop;
import frc.robot.subsystems.Swerve.SwerveDrive;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;


public class ArmTopAuto extends SequentialCommandGroup {
    Claw claw;
    Arm arm;
    double x = 0;
    
    public ArmTopAuto(Claw claw, Arm arm){
        this.claw = claw;
        this.arm = arm;
        addRequirements(claw, arm);

        addCommands(
            new ArmPosition(arm, 145),
            new WristPosition(arm, 58)
        );
    }
    
}