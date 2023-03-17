// package frc.robot.commands.Auto2.Balance;

// public class ClimbRamp {
    
// }


package frc.robot.commands.Balance;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class ClimbRamp extends CommandBase {
    SwerveDrive m_drive;
    NetworkTable navxTable;
    double stopAngle = 10; // Needs to be less than the MoveToRamp stop angle!
    double climbSpeed = 0.2; // Needs to be slow and controlled! 
    double angle = stopAngle + 1; // Greater than stopAngle so that it doesn't stop instantly - JIC

    public ClimbRamp(SwerveDrive swerveDrive){
        addRequirements(swerveDrive);
        m_drive = swerveDrive;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        navxTable = inst.getTable("SmartDashboard").getSubTable("DataNAVX");
        
    }
    @Override
    public void initialize() {
        System.out.println("Climbing");
    }

    @Override
    public void execute() {
        angle = navxTable.getEntry("navx_filtered_pitch").getDouble(0.0);
        if(angle<0){
            m_drive.setChassisSpeeds(climbSpeed, 0, 0);
        }else{
            m_drive.setChassisSpeeds(-climbSpeed, 0, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setChassisSpeeds(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(angle) < stopAngle;
    }
}
