package frc.robot.commands.Auto2.Arm;
import frc.robot.Functions;
import frc.robot.subsystems.Arm.Arm;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class WristPosition extends CommandBase{
   
    private final double minAngleWrist = 0;
    private final double maxAngleWrist = 209;
    int deadband = 10;
    double WristAngle;
    double currentAngle;

    Arm arm;
    double angle;

    public WristPosition(Arm arm, double angle) {
        this.arm = arm;
        this.angle = angle;
        WristAngle = Math.min(Math.max(angle, minAngleWrist), maxAngleWrist);
        currentAngle = arm.getWristAngle();
        System.out.println("Current angle " + currentAngle);
        
        addRequirements(arm);
    }
    

    @Override
    public void execute() {
       // System.out.println("Position adjusted");


        this.currentAngle = arm.getWristAngle();
        double diff = Functions.angleDiff(WristAngle, currentAngle);
        //System.out.println("diff = "+diff);
        if (diff > 0)
             arm.wristSparkMax.setVoltage(.5);
        else
            arm.wristSparkMax.setVoltage(-.5);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(this.currentAngle - WristAngle) < deadband);
        
    }
    @Override
    public void end(boolean interrupted) {
        arm.wristSparkMax.setVoltage(0);
        System.out.println("Position reached");

        super.end(interrupted);
    }



}
