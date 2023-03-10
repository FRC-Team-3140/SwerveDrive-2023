package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class reachTop extends CommandBase{
    Arm arm;
    double topDegreeArm = 0;
    double topDegreeWrist = 0;

    public reachTop(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }
    @Override
    public void execute() {
        arm.setArmAngle(topDegreeArm);
        arm.setWristAngle(topDegreeWrist);
    }
     @Override
     public boolean isFinished() {
         return arm.atSetpoint();
     }

}
