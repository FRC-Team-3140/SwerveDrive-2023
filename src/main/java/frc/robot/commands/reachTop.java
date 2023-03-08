package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class reachTop extends CommandBase{
    Arm arm;
    reachTop(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }
    @Override
    public void execute() {
        arm.setArmSpeed(2);
    }
}
