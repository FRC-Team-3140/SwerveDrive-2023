package frc.robot.commands.Auto2;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class TurnDegrees extends CommandBase {
    SwerveDrive m_drive;
    double objectAngle;
    double currentAngle;
    double targetAngle; 
    boolean resetGyro;
    double speed;
    public TurnDegrees(SwerveDrive swerveDrive, double angle, boolean resetGyro){
        addRequirements(swerveDrive);
        m_drive = swerveDrive;
        targetAngle = angle;
        this.resetGyro = resetGyro;
        while (targetAngle < 0) {
            targetAngle += 360;
        }
        while (targetAngle >= 360) {
            targetAngle -= 360;
        }
    }

    @Override
    public void initialize() {
        if(resetGyro){
            SwerveDrive.m_gyro.zeroYaw();
        }

        m_drive.setLocked(false);
        speed = Math.copySign(.5, targetAngle - currentAngle);
    }

    @Override
    public void execute() {
        currentAngle = SwerveDrive.m_gyro.getAngle();
        m_drive.setChassisSpeeds(0, 0, speed );
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setChassisSpeeds(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        NetworkTableInstance.getDefault().getTable("AngelTesting").getEntry("Bruh").setDouble(targetAngle);
        NetworkTableInstance.getDefault().getTable("AngelTesting").getEntry("Bruh2"
        ).setDouble(SwerveDrive.m_gyro.getYaw() + 180);
        return Math.abs(targetAngle - (NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("DataNAVX").getEntry("navx_yaw").getDouble(currentAngle))) <= 3;
    }
}