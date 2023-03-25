package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Claw;

public class OpenClaw extends CommandBase{
    Claw claw;
    public OpenClaw(Claw claw){
        this.claw = claw;
        addRequirements(claw);
    }
    @Override
    public void initialize() {
        claw.clawOpen();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
