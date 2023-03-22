package frc.robot.commands.Drivetrain;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class EncoderDriveDistance extends CommandBase{
    double startPosition;
    SwerveDrive swerve;
    double distance;
    double xSpeed;
    double ySpeed;
    double currentPos;
    public EncoderDriveDistance(SwerveDrive swerve, double distance, double xSpeed, double ySpeed){
        this.swerve = swerve;
        this.distance = distance;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        addRequirements(swerve);
    }
    @Override
    public void initialize() {

        startPosition = Math.abs(swerve.getPosition());
        
    }
    @Override
    public void execute() {
        swerve.setChassisSpeeds(xSpeed, ySpeed, 0);
        System.out.println("Start Position " + startPosition);
        currentPos = Math.abs(swerve.getPosition());
        NetworkTableInstance.getDefault().getTable("EncoderDriveDistance").getEntry("Start Position").setDouble(startPosition);
        NetworkTableInstance.getDefault().getTable("EncoderDriveDistance").getEntry("Position").setDouble(currentPos);
        NetworkTableInstance.getDefault().getTable("EncoderDriveDistance").getEntry("Distance").setDouble(Math.abs(currentPos - startPosition));
    }
    @Override
    public boolean isFinished() {
        if(Math.abs(currentPos - startPosition) > Math.abs(distance)){
            swerve.setChassisSpeeds(0,0,0);
            return true;
        }else{
            return false;
        }

        
    }
    @Override
    public void end(boolean interrupted) {
        
    
    }
}