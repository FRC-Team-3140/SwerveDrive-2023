// package frc.robot.commands.Auto2.Balance;

// public class ClimbRamp {
    
// }


package frc.robot.commands.Balance;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Drivetrain.EncoderDriveDistance;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class ClimbRamp extends CommandBase {
    SwerveDrive m_drive;
    NetworkTable navxTable;
    double stopAngle; // Needs to be less than the MoveToRamp stop angle!
    double climbSpeed; // Needs to be slow and controlled! 
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
        stopAngle = NetworkTableInstance.getDefault().getTable("Balance").getEntry("Approach Ramp Stop Angle").getDouble(0.0);
        climbSpeed = NetworkTableInstance.getDefault().getTable("Balance").getEntry("Climb Speed").getDouble(0.0);
        angle = navxTable.getEntry("navx_filtered_pitch").getDouble(0.0);
        System.out.printf("Stop Angle %f angle %f/n",stopAngle,angle);
        if(angle<0){
            new EncoderDriveDistance(m_drive, 2, climbSpeed, 0).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
        }else{
            new EncoderDriveDistance(m_drive, 2, -climbSpeed, 0).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
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
