package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Claw;

public class CloseClaw extends CommandBase{
    Claw claw;
    public CloseClaw(Claw claw){
        this.claw = claw;
        addRequirements(claw);
    }
    @Override
    public void initialize() {
        claw.clawClosed();
    }
}
