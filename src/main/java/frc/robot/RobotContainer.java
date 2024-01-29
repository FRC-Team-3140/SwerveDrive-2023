// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ChoreoAuto;
import frc.robot.libs.XboxCotroller;
import frc.robot.subsystems.SwerveDrive;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public static SwerveDrive m_robotDrive = new SwerveDrive();
  public static AHRS gyro = SwerveDrive.gyro;
  // The driver's controller
  public static XboxCotroller controller = new XboxCotroller(0);
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  Field2d m_field = new Field2d();

  ChoreoTrajectory traj;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    autoChooser.addOption("StraightLine", new ChoreoAuto("StraightLine"));
    autoChooser.addOption("TrunBackwards", new ChoreoAuto("TurnBackward"));
    SmartDashboard.putData("Auto", autoChooser);
    SmartDashboard.putData(m_field);


    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      m_robotDrive.run(() -> m_robotDrive.drive(
          controller.getLeftY(),
          controller.getLeftX(),
          controller.getRightX(),
          false)
      )
  );
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void periodic() {
    m_field.setRobotPose(m_robotDrive.getPose());
  }
}