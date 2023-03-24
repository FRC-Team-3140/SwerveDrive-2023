package frc.robot.commands.Auto2.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Arm.Wrist;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;


public class ArmTopAuto extends SequentialCommandGroup {
    Claw claw;
    Arm arm;
    Wrist wrist;
    double x = 0;
    
    public ArmTopAuto(Arm arm, Wrist wrist){
        this.arm = arm;
        this.wrist = wrist;
        addRequirements(arm, wrist);

        addCommands( 
            new ArmPosition(arm, 155),
            new WristPosition(wrist, 58)
        );
    }
    
}