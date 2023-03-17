package frc.robot.commands.Auto2.Balance;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class MoveToRamp extends CommandBase {
    SwerveDrive m_drive;
    NetworkTable navxTable;
    double stopAngle = 7; 
    public MoveToRamp(SwerveDrive swerveDrive){
        addRequirements(swerveDrive);
        m_drive = swerveDrive;
        navxTable = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("DataNAVX");
        //AHRS navx = SwerveDrive.m_gyro;
    }
    @Override
    public void initialize() {
        System.out.println("Move to ramp");
    }

    @Override
    public void execute() {
        m_drive.setChassisSpeeds(0.5, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return navxTable.getEntry("navx_filtered_pitch").getDouble(0.0) > stopAngle;
    }
    @Override
    public void end(boolean interrupted) {
        m_drive.setChassisSpeeds(0, 0, 0);
    }
}
