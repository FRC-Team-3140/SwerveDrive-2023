package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.Timer;

public class DriveDistance extends CommandBase{
    SwerveDrive m_drive;
    double distance;
    Timer timer = new Timer();
    double time = 0;
    double speedX; 
    double speedY;
    
    public DriveDistance(SwerveDrive swerveDrive, double dist, double speedx, double speedy){
        addRequirements(swerveDrive);
        //Make sure one of the speed directions is 0.
        m_drive = swerveDrive;
        distance = dist;
        speedX = speedx;
        speedY = speedy;
        if(speedX != 0){
            time = dist/speedX;
        }else if(speedY !=0) {
            time = dist/speedY;
        }
        

    }

    
    @Override
    public void initialize() {
        m_drive.setLocked(false);
        m_drive.setIdleModes(IdleMode.kBrake);
        timer.start();
    }
    
    @Override
    public void execute() {
        m_drive.setChassisSpeeds(speedX, speedY, 0);

    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setChassisSpeeds(0, 0, 0);
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= time;   
    }
}

























































































































































































































































































































































































































































//500 lines