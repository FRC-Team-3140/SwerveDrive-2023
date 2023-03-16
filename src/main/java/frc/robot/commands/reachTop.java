// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Arm.Arm;

// public class reachTop extends CommandBase{
//     Arm arm;
//     double topDegreeArm = 120.9;
//     double topDegreeWrist = 203.2;

//     public reachTop(Arm arm){
//         this.arm = arm;
//         addRequirements(arm);
//     }
    
//     @Override
//     public void execute() {
//         arm.setArmAngle(topDegreeArm);
//         arm.setWristAngle(topDegreeWrist);
//     }
    
//      @Override
//      public boolean isFinished() {
//          return arm.getArmAngle() <= topDegreeArm;
//      }

// }

// // Work on this
// // reachTop, findPath
