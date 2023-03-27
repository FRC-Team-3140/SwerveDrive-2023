
package frc.robot.commands.auto;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.EncoderDriveDistance;
import frc.robot.commands.Drivetrain.TurnAndDrive;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class TestAuto extends CommandBase {
    SwerveDrive swerve;
    public TestAuto(SwerveDrive swerve){
        this.swerve = swerve;
       // swerve.h


    }
    @Override
    public void initialize() {
        new SequentialCommandGroup(
            //Turns Woo
            // new TurnAndDrive(swerve, 1, 180, .5, 0)
            new TurnAndDrive(swerve, 4.1,180, -.5, 0)
            //new TurnAndDrive(swerve, 1, 180, -.5, 0)


            //Moves Forward/Backward
           // new EncoderDriveDistance(swerve, .5, .5, 0),
            //new EncoderDriveDistance(swerve, .5, -.5, 0),

            //Moves Right and left
            //new EncoderDriveDistance(swerve, .5, 0 , .5)
           //new EncoderDriveDistance(swerve, .5, 0 , -.5)
            
        ).schedule();
    }

    
}
