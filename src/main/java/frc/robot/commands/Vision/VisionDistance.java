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
            try {
                mDrive.setChassisSpeeds(Math.pow(mDrive.getTarget().getBestCameraToTarget().getX() - distance, 3) * .15 + .075, 0, 0);              
            } catch (Exception bruh) {
                System.out.println("This fricken failed man");
            }
        }
        //Math.pow((mDrive.getTarget().getBestCameraToTarget().getX() - distance), 3)* .15 + .075
    }
    @Override
    public boolean isFinished() {
        if(mDrive.getTarget() != null){
            //.33555 is the vertical distance from the apriltag to the camera
            return distance >= Math.hypot(mDrive.getTarget().getBestCameraToTarget().getX(), .3555);
        }else{
            return true;
        }
        
    }
    @Override
    public void end(boolean interrupted) {
        mDrive.setChassisSpeeds(0, 0, 0);
    }
}
