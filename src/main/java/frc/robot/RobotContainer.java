// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.
/*
-LEDs attached
-Camera (on the top for the driver), and on the drivers station
-Pressure gauge 
-
*/
package frc.robot;
import java.util.function.BooleanSupplier;

import com.ctre.phoenixpro.signals.Led1OffColorValue;

import frc.robot.commands.Auto2.Arm.ArmPosition;
import frc.robot.commands.Auto2.Arm.ArmTopAuto;
import frc.robot.commands.Balance.balance;
import frc.robot.commands.Balance.balance_alt;
import frc.robot.commands.Drivetrain.SwerveController;
import frc.robot.commands.Drivetrain.TurnDegrees;
import frc.robot.commands.Vision.TargetAlign;
import frc.robot.commands.Vision.TurnToObject;
import frc.robot.commands.Vision.TurnToTarget;
import frc.robot.commands.Vision.VisionDistance;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Arm.Wrist;
import frc.robot.subsystems.Swerve.SwerveDrive;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController.Button;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.DoNothingAuto;
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.commands.auto.DriveToWall;
import frc.robot.commands.auto.EverythingAuto;
import frc.robot.commands.auto.OneCubeAuto;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {
  private LED led = new LED();
  private static DigitalInput limitSwitchUpper = new DigitalInput(3);
  private static DigitalInput limitSwitchLower = new DigitalInput(2);
  private static DigitalInput limitSwitchWrist = new DigitalInput(4);


  private static RobotContainer m_robotContainer = new RobotContainer();
  
  public static DigitalInput getLimitSwitchUper(){
    return limitSwitchUpper; 

  }

  public static DigitalInput getLimitSwitchLower(){
    return limitSwitchLower; 
  }

  public static DigitalInput getlimitSwitchWrist(){
    return limitSwitchWrist;
  }

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
  // The robot's subsystems
  public final PowerManagment m_powerManagment = new PowerManagment();

  // public final SwerveModule m_swerveModule_fr = new
  // SwerveModule("fr",10,1,2,303.0+45);
  // public final SwerveModule m_swerveModule_fl = new
  // SwerveModule("fl",11,3,4,358 + 315);
  // public final SwerveModule m_swerveModule_br = new
  // SwerveModule("br",12,5,6,242.0 + 135);
  // public final SwerveModule m_swerveModule_bl = new
  // SwerveModule("bl",13,7,8,187.0 + 225);

  public SwerveDrive swerveDrive = new SwerveDrive();
  public boolean aDown = false;

  public Claw claw = new Claw(2, 0, 1);

  // Joysticks

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
  private double rumble = 0;
  public SwerveController drive_robot;

  public NetworkTable DataNAVX;

  public XboxController m_xbox_cotroller = new XboxController(0);
  public XboxController m_xbox_cotroller_2 = new XboxController(1);
  public Arm arm = new Arm(10);
  public Wrist wrist = new Wrist(9);

  private final NetworkTable swerve_table;
  private final NetworkTableEntry speedDampening;

  
  


    // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    swerve_table = NetworkTableInstance.getDefault().getTable("swerve_chassis");
    speedDampening = swerve_table.getEntry("speed_dampening");
    DataNAVX = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("DataNAVX");
    // // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Smartdashboard Subsystems

    // SmartDashboard Buttons
    // SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
    // SmartDashboard.putData("TestTurnMotor", new TestTurnMotor( m_swerveModule ));
    // SmartDashboard.putData("TestDriveMotor", new TestDriveMotor( m_swerveModule
    // ));
    // SmartDashboard.putData("TestBothMotors", new TestBothMotors( m_swerveModule
    // ));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND
    
    // Configure autonomous sendable chooser
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    m_chooser.addOption("Balance_alt", new balance_alt(swerveDrive));
    m_chooser.addOption("Do Nothing Auto", new DoNothingAuto());
    m_chooser.addOption("One Cube Auto ", new OneCubeAuto(swerveDrive, claw, arm, wrist, SwerveDrive.headless));
    m_chooser.addOption("Everything", new EverythingAuto(swerveDrive, arm, wrist, claw));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    SmartDashboard.putData("Auto Mode", m_chooser);
  }
  public SwerveDrive getSwerve(){
    return swerveDrive;
  }

  public Arm getArm(){
    return arm;
  }

  public XboxController getController1(){
    return m_xbox_cotroller;
  }

  public XboxController getController2(){
    return m_xbox_cotroller_2;

  }
  public Wrist getWrist(){
    return wrist;
  }


  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */


  private void configureButtonBindings() {
    drive_robot = new SwerveController(swerveDrive,
        () -> {
          return -m_xbox_cotroller.getLeftY();
        },
        () -> {
          return -m_xbox_cotroller.getLeftX();
        },
        () -> {
          return -m_xbox_cotroller.getRightX();
        });
    


    new JoystickButton(m_xbox_cotroller, Button.kA.value)
        .onTrue(new InstantCommand(()->{
          swerveDrive.setLocked(true);
        }))
        .onFalse(new InstantCommand(()->{
          swerveDrive.setLocked(false);
        }));

    new JoystickButton(m_xbox_cotroller, Button.kY.value)
          .onTrue(new InstantCommand(()->{
              SwerveDrive.headless = true;
              System.out.println("Headless orientation");
            }))
          .whileTrue(new RunCommand(()->{
            if(rumble < 0.075 && rumble > -0.1) {
              m_xbox_cotroller.setRumble(RumbleType.kBothRumble, rumble);
              rumble += 0.001;
            } else {
              if(rumble > -1) SwerveDrive.m_gyro.zeroYaw();
              rumble = -1;
              m_xbox_cotroller.setRumble(RumbleType.kBothRumble, 0);
              
            }
          }))
          .onFalse(new InstantCommand(()->{
            m_xbox_cotroller.setRumble(RumbleType.kBothRumble, 0);
            rumble = 0;
          }));
    
    //new JoystickButton(m_xbox_cotroller, Button.kX.value)
            
         
              
    new JoystickButton(m_xbox_cotroller, Button.kX.value)
        .onTrue(new InstantCommand(()->{
          SwerveDrive.headless = false;
          System.out.println("Headed orientation");
        }));
          // .whileTrue(new RunCommand(()->{
          //   if(rumble < 0.075 && rumble > -0.1) {
          //     m_xbox_cotroller.setRumble(RumbleType.kBothRumble, rumble);
          //     rumble += 0.001;
          //   } else {
          //     if(rumble > -1) SwerveDrive.m_gyro.zeroYaw();
          //     rumble = -1;
          //     m_xbox_cotroller.setRumble(RumbleType.kBothRumble, 0);
              
          //   }
          // }))
          // .onFalse(new InstantCommand(()->{
          //   m_xbox_cotroller.setRumble(RumbleType.kBothRumble, 0);
          //   rumble = 0;
          // }));
    BooleanSupplier clawButtonPressedSupplier = () -> m_xbox_cotroller_2.getRightBumper();
    new JoystickButton(m_xbox_cotroller_2, Button.kRightBumper.value)
       .onTrue(new RunCommand(() -> {
          claw.toggleClaw();
       }).until(clawButtonPressedSupplier));/*.withInterruptBehavior(InterruptionBehavior.kCancelSelf))
       .onTrue(new RunCommand(()->{}));*/

    //  new JoystickButton(m_xbox_cotroller_2, Button.kA.value)
    //    .whileTrue(new RunCommand(() -> {claw.clawClosed();}))
    //    .onFalse(new RunCommand(() -> {claw.clawOpen();}));
    
   
 //   m_xbox_cotroller.x(new EventLoop()).;
    new JoystickButton(m_xbox_cotroller, Button.kB.value)
          .onTrue(new InstantCommand(()->{ 
            if(1 - m_xbox_cotroller.getRightTriggerAxis() < 1) {
              speedDampening.setDouble(1 - m_xbox_cotroller.getRightTriggerAxis());
            } else {
              speedDampening.setDouble(1);
            }
          }));
          
    new JoystickButton(m_xbox_cotroller, Button.kLeftBumper.value)
      .onTrue(new TurnDegrees(swerveDrive, 90, true));

      
      BooleanSupplier bruh = ()-> {return limitSwitchLower.get();}; 
    new Trigger(bruh).onFalse(new InstantCommand(()-> {
      arm.getArmEncoder().setPosition(0);
    }
    ));
  
   
    //  new JoystickButton(m_xbox_cotroller, Button.kRightBumper.value).whileTrue(new TargetAlign(swerveDrive));
      new JoystickButton(m_xbox_cotroller, Button.kBack.value).onTrue(new TargetAlign(swerveDrive)); //window button (two rectangles)
    // This command schedules the drive_robot method
    // *** Makes the robot run *** 
    swerveDrive.setDefaultCommand(drive_robot);

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
    // Create some buttons

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS

    final JoystickButton rightBumper = new JoystickButton(m_xbox_cotroller, Button.kRightBumper.value);

     rightBumper.whileTrue(new balance(swerveDrive));

    new JoystickButton(m_xbox_cotroller_2, Button.kLeftBumper.value).onTrue(new InstantCommand(() -> led.toggleLED()));
    

  }

  /*What Controller Button Does What!
    Primary Controller:
      -Joysticks- Moves the swerve drive
      -A- Locks Wheels
      -Y- Headless, hold y to reset 0 (theoretically)
      -X- Head
      -B and Right Trigger- Speed Dampener
      -Left Bumper- Turn to object
      -Right Bumper- Charging Station
      -Window Button- Vision Distance
      
    Secondary Controller:
      -Left Joystick- Wrist 
      -Right Joystick- Arm
      -Left Bumper-Drive to wall
      -Right Bumper- Claw
      -Left Bumper- LEDs



  
   */








  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

    
  
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
   return m_chooser.getSelected();
    //return ArmTopAuto(claw, arm);
  }
}
