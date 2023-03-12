package frc.robot.commands.Auto2;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Claw;


import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drivetrain.TurnDegrees;
import frc.robot.commands.reachTop;
import frc.robot.subsystems.Swerve.SwerveDrive;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class ArmTopAuto extends SequentialCommandGroup {
    Claw claw;
    Arm arm;
    
    public ArmTopAuto(Claw claw, Arm arm){
        this.claw = claw;
        this.arm = arm;
        addRequirements(claw, arm);

        addCommands(
            new ArmTop(arm),
            new InstantCommand(() -> {claw.clawOpen();})
        );
    }
    
}