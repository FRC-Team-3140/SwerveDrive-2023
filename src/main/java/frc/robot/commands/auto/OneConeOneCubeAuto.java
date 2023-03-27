package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auto2.Arm.ArmPosition;
import frc.robot.commands.Auto2.Arm.ArmTopAuto;
import frc.robot.commands.Auto2.Arm.WristPosition;
import frc.robot.commands.Drivetrain.EncoderDriveDistance;
import frc.robot.commands.Drivetrain.TurnAndDrive;
import frc.robot.commands.Vision.TargetAlign;
import frc.robot.commands.Vision.TurnToObject;
import frc.robot.commands.Vision.VisionDistance;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Arm.Wrist;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class OneConeOneCubeAuto extends CommandBase{
    SwerveDrive swerve;
    Arm arm;
    Wrist wrist;
    Claw claw;
    NetworkTableEntry Stage =  NetworkTableInstance.getDefault().getTable("2-CubeAuto").getEntry("Stage");
    public OneConeOneCubeAuto(SwerveDrive swerve, Arm arm, Wrist wrist, Claw claw){
        this.swerve = swerve;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        addRequirements(swerve, arm, wrist);

    }
    @Override
    public void initialize() {

        new SequentialCommandGroup(
            new ParallelRaceGroup(
        new WristPosition(wrist, 190).withTimeout(.2),
        new ArmPosition(arm, 80).withTimeout(.2)),
       new ParallelCommandGroup(
           new ArmPosition(arm, 163),
           new SequentialCommandGroup(
               new WaitCommand(1.2),
               new WristPosition(wrist, 60)
           )
       ),
      // new InstantCommand(()-> {Stage.setString("DriveToWallStart");}),
      new DriveToWall(swerve).withTimeout(2.5),
      new WaitCommand(.02),
   //Make sure turn and drive works
   //    new ArmTopAuto(arm, wrist),
      new OpenClaw(claw),
      
           new InstantCommand(()-> {Stage.setString("Retract Arm");}),
           new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new EncoderDriveDistance(swerve, 1.2, -0.67, 0)),
            new SequentialCommandGroup( 
                new WristPosition(wrist, 84),
                new ParallelCommandGroup(
                  new ArmPosition(arm, 39),
                  new WristPosition(wrist, 168)
            ))),

                //new RetractArm(arm, wrist), 
                
                new InstantCommand(()-> {Stage.setString("Move Back");}),
                
          //  ).withTimeout(3),
          new InstantCommand(()-> {Stage.setString("Turn n drive");}),
        new ParallelCommandGroup(
          new TurnAndDrive(swerve, 1.25, 180, -.65, -.003467), //.withTimeout(3),
          new ArmPosition(arm, 54),
          new WristPosition(wrist, 170)),
          new WristPosition(wrist, 150),
           //new InstantCommand(()-> {Stage.setString("turn to bject");}),
           //new TurnToObject(swerve),
           new InstantCommand(()-> {Stage.setString("Mvoe and pick up");}),
           new TurnToObject(swerve),
           new ParallelDeadlineGroup(
            new SensorCloseClaw(claw),
            new EncoderDriveDistance(swerve, 1.5, -.23, 0)
           ),
           new WristPosition(wrist, 190),//.withTimeout(5),
           new InstantCommand(()-> {Stage.setString("turn n drive 2");}),

                new TurnAndDrive(swerve, 4.1, -1, .6, 0)
                //new ArmTopAuto(arm, wrist),
           
           /* 
           new InstantCommand(()-> {Stage.setString("Driving time");}),
           new ArmTopAuto(arm, wrist).withTimeout(2),
           new DriveToWall(swerve),
           new EncoderDriveDistance(swerve, 2.4, 0, -.5), 
            //Might have to change speed on this one
           
           new TargetAlign(swerve),
           
           new InstantCommand(()-> {Stage.setString("Drive To Wall");}),

           new DriveToWall(swerve),
           new InstantCommand(()-> {Stage.setString("Score Game Piece");}),
           new ArmTopAuto(arm, wrist),
            new OpenClaw(claw)
            */
        ).schedule();
    }
    @Override
    public void end(boolean interrupted) {
        
    }
    
}
