package frc.robot.commands.Auto2.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Arm.Wrist;


public class ArmTopAuto extends CommandBase {
    Claw claw;
    Arm arm;
    Wrist wrist;
    double x = 0;
    
    public ArmTopAuto(Arm arm, Wrist wrist){
        this.arm = arm;
        this.wrist = wrist;
        addRequirements(arm, wrist);
    }
    @Override
    public void initialize() {
        new SequentialCommandGroup(
            new ArmPosition(arm, 115),
            new ParallelCommandGroup(
                new ArmPosition(arm, 155),
                new WristPosition(wrist, 58)
            )
        ).schedule();
    }
    @Override
    public boolean isFinished() {
        return true;
    }

@Override
public void end(boolean interrupted) {
}
    
    
}