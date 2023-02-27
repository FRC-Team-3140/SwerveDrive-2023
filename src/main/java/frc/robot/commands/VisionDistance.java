package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class VisionDistance extends CommandBase {
    double distance;
    SwerveDrive mDrive;
    public VisionDistance(SwerveDrive swerveDrive, double distance){
        addRequirements(swerveDrive);
        mDrive = swerveDrive;
        this.distance = distance;
    }
    @Override
    public void initialize() {
        mDrive.setIdleModes(IdleMode.kBrake);
    }
    @Override
    public void execute() {
        if(mDrive.getTarget() != null){
            mDrive.setChassisSpeeds(-Math.pow((mDrive.getTarget().getBestCameraToTarget().getX() - distance), 2)* .1 + .075, 0, 0);                
        }
    }
    @Override
    public boolean isFinished() {
        return distance > mDrive.getTarget().getBestCameraToTarget().getX();
    }
}
