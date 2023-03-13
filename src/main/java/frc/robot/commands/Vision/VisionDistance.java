package frc.robot.commands.Vision;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;

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
            mDrive.setChassisSpeeds(Math.pow(mDrive.getTarget().getBestCameraToTarget().getX() - distance, 3) * .15 + .075, 0, 0);                
        }
        //Math.pow((mDrive.getTarget().getBestCameraToTarget().getX() - distance), 3)* .15 + .075
    }
    @Override
    public boolean isFinished() {
        if(mDrive.getTarget() != null){
            return distance >= Math.sqrt(Math.pow(mDrive.getTarget().getBestCameraToTarget().getX(),2) - .1264);
        }else{
            return true;
        }
        
    }
    @Override
    public void end(boolean interrupted) {
        mDrive.setChassisSpeeds(0, 0, 0);
    }
}
