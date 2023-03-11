package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Functions;

public class TurnDegrees extends CommandBase {
    SwerveDrive m_drive;
    double objectAngle;
    double currentAngle;
    double targetAngle; 

    public TurnDegrees(SwerveDrive swerveDrive, double angle){
        addRequirements(swerveDrive);
        m_drive = swerveDrive;
        targetAngle = angle;

        while (targetAngle < 0) {
            targetAngle += 360;
        }
        while (targetAngle >= 360) {
            targetAngle -= 360;
        }
    }

    @Override
    public void initialize() {
        m_drive.setLocked(false);
    }

    @Override
    public void execute() {
                currentAngle = SwerveDrive.m_gyro.getAngle();
        m_drive.setChassisSpeeds(0, 0, Math.copySign(0.1, Functions.angleDiff(targetAngle, currentAngle)));
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setChassisSpeeds(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Functions.angleDiff(targetAngle, currentAngle)) <= 3;
    }
}