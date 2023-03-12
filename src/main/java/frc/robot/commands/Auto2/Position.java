package frc.robot.commands.Auto2;
import frc.robot.Functions;
import frc.robot.subsystems.Arm.Arm;

import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Position extends CommandBase{
    private final double minAngleArm = 0;
    private final double maxAngleArm = 360;
    private final double minAngleWrist = 0;
    private final double maxAngleWrist = 209;
    int deadband = 10;
    double WristAngle;
    double currentAngle;

    Arm arm;
    double angle;

    public Position(Arm arm, double angle) {
        this.arm = arm;
        this.angle = angle;
        WristAngle = Math.min(Math.max(angle, minAngleWrist), maxAngleWrist);
        currentAngle = arm.getWristAngle();
        
        addRequirements(arm);
    }
    

    @Override
    public void execute() {
        double diff = Functions.angleDiff(WristAngle, currentAngle);
        if (diff < 0)
             arm.wristSparkMax.setVoltage(.5);
        else
            arm.wristSparkMax.setVoltage(-.5);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getWristAngle() - WristAngle) < deadband;
    }
    @Override
    public void end(boolean interrupted) {
        arm.wristSparkMax.setVoltage(0);
        super.end(interrupted);
    }



}
