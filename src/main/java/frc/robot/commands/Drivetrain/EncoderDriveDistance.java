package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class EncoderDriveDistance extends CommandBase{
    double startPosition;
    SwerveDrive swerve;
    double distance;
    double xSpeed;
    double ySpeed;
    public EncoderDriveDistance(SwerveDrive swerve, double distance, double xSpeed, double ySpeed){
        this.swerve = swerve;
        this.distance = distance;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        addRequirements(swerve);
    }
    @Override
    public void initialize() {
        startPosition = swerve.getPosition();
    }
    @Override
    public void execute() {
        swerve.setChassisSpeeds(xSpeed, ySpeed, 0);
    }
    @Override
    public boolean isFinished() {
        return Math.abs(swerve.getPosition() - startPosition) == Math.abs(distance);
        
    }
    @Override
    public void end(boolean interrupted) {
        swerve.setChassisSpeeds(0,0,0);
    
    }
}
