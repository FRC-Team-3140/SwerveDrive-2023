package frc.robot.commands.Balance;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class MoveToRamp extends CommandBase {
    SwerveDrive m_drive;
    NetworkTable navxTable;
    double stopAngle = 12; 
    double rampApproachVelocity = 0.3; // percentage multiplier for the NEO

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
        m_drive.setChassisSpeeds(rampApproachVelocity, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(navxTable.getEntry("navx_filtered_pitch").getDouble(0.0)) > stopAngle;
    }
    @Override
    public void end(boolean interrupted) {
        m_drive.setChassisSpeeds(0, 0, 0);
    }
}