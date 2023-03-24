
package frc.robot.commands.auto;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.EncoderDriveDistance;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class TestAuto extends CommandBase {
    SwerveDrive swerve;
    public TestAuto(SwerveDrive swerve){


    }
    @Override
    public void initialize() {
        new SequentialCommandGroup(
            new EncoderDriveDistance(swerve, 1, .5, 0)
        );
    }

    
}
