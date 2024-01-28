// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.libs.XboxCotroller;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static XboxCotroller controller = new XboxCotroller(0);
  public static AHRS gyro = new AHRS(Port.kMXP);
  public static SwerveDrive swerve = new SwerveDrive();
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  Field2d field = new Field2d();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    ChoreoTrajectory traj = Choreo.getTrajectory("NewPath");
    Command swerveCommand = Commands.sequence(Commands.runOnce(()-> swerve.resetPose(traj.getInitialPose())), Choreo.choreoSwerveCommand(traj, swerve::getPose, new PIDController(0, 0, 0), new PIDController(0, 0, 0), new PIDController(0, 0, 0), swerve::driveRobotRelative, swerve::shouldFlipPath, swerve), new InstantCommand(()-> {swerve.drive(0, 0, 0, false);}));
    autoChooser.addOption("Auto1", new PathPlannerAuto("Auto1"));
    autoChooser.addOption("Auto2", new PathPlannerAuto("Auto2"));
    autoChooser.addOption("Auto3", new PathPlannerAuto("Special"));
    autoChooser.addOption("Auto4", swerveCommand);
    // autoChooser.addOption("Auto 5", AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("traj")));
    field.getObject("traj").setPoses(traj.getInitialPose(), traj.getFinalPose());
    field.getObject("trajPoses").setPoses(traj.getPoses());
    SmartDashboard.putData("Auto", autoChooser);
    
    
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new JoystickButton(controller, Button.kA.value).onTrue(new InstantCommand(this::resetGyro));
    new JoystickButton(controller, Button.kB.value).onTrue(new InstantCommand(()-> swerve.resetPose(new Pose2d())));
    new JoystickButton(controller, Button.kX.value).onTrue(new PathPlannerAuto("Special"));
  }
  

  public void resetGyro(){
    gyro.reset();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
//    */



  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
