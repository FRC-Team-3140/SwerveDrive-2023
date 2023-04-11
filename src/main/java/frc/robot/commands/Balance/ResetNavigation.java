package frc.robot.commands.Balance;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveOdometer;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class ResetNavigation extends CommandBase {
    SwerveDrive m_drive;
    NetworkTable navxTable;
    double stopAngle; 
    double rampApproachVelocity; // percentage multiplier for the NEO

    public ResetNavigation(SwerveDrive swerveDrive){
        addRequirements(swerveDrive);

        m_drive = swerveDrive;
        //AHRS navx = SwerveDrive.m_gyro;
    }
    @Override
    public void initialize() {
        System.out.println("Reset Navigation");
        SwerveOdometer.getInstance().reset();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
    }
}
