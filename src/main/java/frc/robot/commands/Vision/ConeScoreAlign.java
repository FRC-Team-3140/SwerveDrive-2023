package frc.robot.commands.Vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Drivetrain.EncoderDriveDistance;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class ConeScoreAlign extends CommandBase{
    SwerveDrive swerveDrive;
    public ConeScoreAlign(SwerveDrive swerveDrive){
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }
    double setPoint = 0;
    @Override
    public void initialize() {
        SwerveDrive.m_gyro.zeroYaw();
        swerveDrive.setLocked(false);
        setPoint = swerveDrive.getTarget().getBestCameraToTarget().getY() + Units.feetToMeters(2);
        new EncoderDriveDistance(swerveDrive, setPoint, 0, .3);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
