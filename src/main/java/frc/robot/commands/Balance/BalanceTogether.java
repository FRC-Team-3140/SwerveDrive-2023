package frc.robot.commands.Balance;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class BalanceTogether extends SequentialCommandGroup {

    SwerveDrive swerve;

    public BalanceTogether(SwerveDrive swerve) {
        addCommands(
            //Modify Speeds in the Commands below so that the robot doen't move too fast!

            /*
             To,
                morning Brogan (who will problably forget this by morning) and other programers:

             Posible issue is that pitch vs roll is different on the parade bot and the compition bot. 

             I just looked through some pictures of the bot and pictures of the board, and I think the values
             may need to be negative, as the current tilt may be going from 0 to -16 as it tilts up. We also
             need to chage the gyroscope reorient to only zero out the yaw as to prevent prolems with getting 
             pitch and rollafter a reset if the bot was tillted a bit.

             We need to test in the morining by just reading network tables and tilting the bot. 
            
             From,
                delerious 11 o'clock Brogan
             */

            // new MoveToRamp(swerve).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
            new ClimbRamp(swerve).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
            // new BalanceAndEngage(swerve).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
            );
    }
}
