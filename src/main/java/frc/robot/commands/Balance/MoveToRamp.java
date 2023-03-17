package frc.robot.commands.Balance;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class MoveToRamp extends CommandBase {
    SwerveDrive m_drive;
    NetworkTable navxTable;
    double stopAngle; 
    double rampApproachVelocity; // percentage multiplier for the NEO

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
        stopAngle = NetworkTableInstance.getDefault().getTable("Balance").getEntry("Approach Ramp Stop Angle").getDouble(0.0);
        rampApproachVelocity = NetworkTableInstance.getDefault().getTable("Balance").getEntry("Approach Ramp Velocity").getDouble(0.0);
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
