package frc.robot.commands.Auto2.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.Functions;

public class ArmPosition extends CommandBase{    
    private final double minAngleArm = 0;
    private final double maxAngleArm = 360;
    int deadband = 10;
    double ArmAngle;
    double currentAngle;

    Arm arm;
    double angle;

    public ArmPosition(Arm arm, double angle) {
        this.arm = arm;
        this.angle = angle;
        ArmAngle = Math.min(Math.max(angle, minAngleArm), maxAngleArm);
        currentAngle = arm.getArmAngle();
        System.out.println("Current angle " + currentAngle);
        
        addRequirements(arm);
    }
    

    @Override
    public void execute() {
        System.out.println("Position adjusted");


        this.currentAngle = arm.getArmAngle();
        double diff = Functions.angleDiff(ArmAngle, currentAngle);
        System.out.println("diff = "+diff);
        if (diff > 0)
             arm.ArmSparkMax.setVoltage(1);
        else
            arm.ArmSparkMax.setVoltage(-1);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(this.currentAngle - ArmAngle) < deadband);
        
    }
    @Override
    public void end(boolean interrupted) {
        arm.ArmSparkMax.setVoltage(0);
        System.out.println("Position reached");

        super.end(interrupted);
    }



}



