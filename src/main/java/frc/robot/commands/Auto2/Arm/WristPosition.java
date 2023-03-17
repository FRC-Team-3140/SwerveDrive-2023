package frc.robot.commands.Auto2.Arm;
import frc.robot.subsystems.Arm.Wrist;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WristPosition extends CommandBase{
   
    private final double minAngleWrist = 0;
    private final double maxAngleWrist = 209;
    int deadband = 1;
    double WristAngle;
    double currentAngle;

    Wrist wrist;
    double angle;

    public WristPosition(Wrist wrist, double angle) {
        this.wrist = wrist;
        this.angle = angle;
        WristAngle = Math.min(Math.max(angle, minAngleWrist), maxAngleWrist);
        currentAngle = wrist.getWristAngle();
        System.out.println("Current angle " + currentAngle);
        
        addRequirements(wrist);
    }
    

    @Override
    public void execute() {
       // System.out.println("Position adjusted");


        this.currentAngle = wrist.getWristAngle();
        //double diff = Functions.angleDiff(WristAngle, currentAngle);
        //System.out.println("diff = "+diff);
        //TODO Uncomment and change setVoltage to setWristAngle
        //if (diff > 0)
            // wrist.setWristVoltage(2);
        //else
            //wrist.setWristVoltage(-2);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(this.currentAngle - WristAngle) < deadband);
        
    }
    @Override
    public void end(boolean interrupted) {
        //wrist.setWristVoltage(0);
        System.out.println("Position reached");

        super.end(interrupted);
    }



}
