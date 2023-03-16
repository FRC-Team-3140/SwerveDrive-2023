package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class DriveDistanceEncoder extends CommandBase{
    SwerveDrive swerve;
    double Xspeed;
    double Yspeed;
    double distance;
    public DriveDistanceEncoder(SwerveDrive swerve, double Xspeed, double Yspeed, double distance){
        this.swerve = swerve;
        this.Xspeed = Xspeed;
        this.Yspeed = Yspeed;
        this.distance = distance;
        addRequirements(swerve);
        
    }
    @Override
    public void initialize() {
        
    }
    @Override
    public void execute() {
        swerve.setChassisSpeeds(Xspeed, Yspeed, 0);
        
    }
    @Override
    public boolean isFinished() {
        //change the == to a >
        return Math.abs(swerve.getPosition())  == distance ? true : false;
    }
    @Override
    public void end(boolean interrupted) {
        swerve.setChassisSpeeds(0,0,0);
    }
}