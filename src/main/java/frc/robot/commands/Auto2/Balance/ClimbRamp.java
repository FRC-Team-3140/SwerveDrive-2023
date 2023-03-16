// package frc.robot.commands.Auto2.Balance;

// public class ClimbRamp {
    
// }


package frc.robot.commands.Auto2.Balance;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class ClimbRamp extends CommandBase {
    SwerveDrive m_drive;
    NetworkTable navxTable;
    double stopAngle = 10;
    public ClimbRamp(SwerveDrive swerveDrive){
        addRequirements(swerveDrive);
        m_drive = swerveDrive;
        
    }

    @Override
    public void execute() {
        double angle = SwerveDrive.m_gyro.getRoll();
        if(angle<0){
            m_drive.setChassisSpeeds(0.2, 0, 0);
        }else{
            m_drive.setChassisSpeeds(-0.2, 0, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setChassisSpeeds(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        double angle = SwerveDrive.m_gyro.getRoll();
        return Math.abs(angle) < stopAngle;
    }
}
