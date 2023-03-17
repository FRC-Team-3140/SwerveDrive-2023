package frc.robot.commands.Balance;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class balance_alt_support extends CommandBase{
    
    SwerveDrive m_drive;
    double speed = 0;
    double Threshold = 2;
    
    public balance_alt_support(SwerveDrive swerveDrive) {
        m_drive = swerveDrive;
        addRequirements(swerveDrive);
        
    }
    
    @Override
    public void initialize() {
        speed = NetworkTableInstance.getDefault().getTable("Extra Support").getEntry("Speed (Support)").getDouble(0.8);
        speed = SwerveDrive.m_gyro.getPitch() > Threshold ? speed : -speed;
        m_drive.setLocked(false);
    }
    
    @Override
    public void execute(){
        m_drive.setChassisSpeeds(speed,0,0);
    }
    
    public boolean isFinished(){
        return Math.abs(SwerveDrive.m_gyro.getPitch()) < Threshold;
    }

    public void end(boolean interrupted){
        m_drive.setChassisSpeeds(0, 0, 0);
        m_drive.setLocked(true);
    }
}