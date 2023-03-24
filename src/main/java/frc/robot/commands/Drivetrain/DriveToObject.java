package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Vision.TurnToObject;
import frc.robot.subsystems.Swerve.SwerveDrive;

/**
 * DriveToObject
 */
public class DriveToObject extends CommandBase{
    SwerveDrive swerveDrive;
    public DriveToObject(SwerveDrive swerveDrive){
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
        
    }
    @Override
    public void initialize() {
        SwerveDrive.m_gyro.zeroYaw();
        new TurnToObject(swerveDrive);
        swerveDrive.setLocked(false);
    }
    @Override
    public void execute() {
        swerveDrive.setChassisSpeeds(.3, 0, 0);
    }
    @Override
    public boolean isFinished() {
        return swerveDrive.getDistanceToConeFromCamera() <0.1;
    }
    @Override
    public void end(boolean interrupted) {
        swerveDrive.setChassisSpeeds(0, 0, 0);
    }


}