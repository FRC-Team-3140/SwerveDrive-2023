// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Robot. 

package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Swerve.SwerveDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in
 * the project.
 */
public class Robot extends TimedRobot {

    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    // Arm & Claw Subsystems
    // private final Arm ARM = new Arm(Constants.armID, Constants.wristID);

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        System.out.println("AVE CHRISTUS REX but in spanish");
        System.out
                .println("SANCTA MARIA, MATER DEI, ORA PRO NOBIS PECCATORIBUS ET NOBIS VICTORIAM REDDE but in spanish");
        NetworkTableInstance.getDefault().getTable("Extra Support").getEntry("Distance (Alt)").setDouble(.975);
        NetworkTableInstance.getDefault().getTable("Extra Support").getEntry("Angle (Alt)").setDouble(11.27);
        NetworkTableInstance.getDefault().getTable("Extra Support").getEntry("Speed (Support)").setDouble(.08);
        NetworkTableInstance.getDefault().getTable("PID").getEntry("P").setDouble(.0075000);
        NetworkTableInstance.getDefault().getTable("PID").getEntry("I").setDouble(0);
        NetworkTableInstance.getDefault().getTable("Balance").getEntry("Balance P").setDouble(6);
        NetworkTableInstance.getDefault().getTable("Balance").getEntry("Balance D").setDouble(0.0);
        NetworkTableInstance.getDefault().getTable("Balance").getEntry("Approach Ramp Stop Angle").setDouble(9);
        NetworkTableInstance.getDefault().getTable("Balance").getEntry("Approach Ramp Velocity").setDouble(0.5);
        NetworkTableInstance.getDefault().getTable("Balance").getEntry("Climb Ramp Stop Angle").setDouble(8);
        NetworkTableInstance.getDefault().getTable("Balance").getEntry("Climb Speed").setDouble(0.5);
        // .025
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = RobotContainer.getInstance();
        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);
        SwerveDrive.zeroNavx();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    // This function is called once each time the robot enters Disabled mode.
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    // This autonomous runs the autonomous command selected by your {@link
    // RobotContainer} class.
    @Override
    public void autonomousInit() {
        SwerveDrive.zeroNavx();
        SwerveDrive.headless = true;// false
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    // This function is called periodically during autonomous.
    @Override
    public void autonomousPeriodic() {
        NetworkTableInstance.getDefault().getTable("NAVX Angle").getEntry("Yaw")
                .setDouble(SwerveDrive.m_gyro.getYaw());
        // m_robotContainer.updateNavX();
    }

    @Override
    public void teleopInit() {

        SwerveDrive.headless = true;

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        NetworkTableInstance.getDefault().getTable("BotPosBR").getEntry("Distance")
                .setDouble(RobotContainer.getInstance().getSwerve().getBRModule().getEncoder().getPosition());
        NetworkTableInstance.getDefault().getTable("BotPosBR").getEntry("DistanceConversion").setDouble(
                RobotContainer.getInstance().getSwerve().getBRModule().getEncoder().getPositionConversionFactor());
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        // m_robotContainer.getArm().ArmSparkMax.getForwardLimitSwitch(Type.kNormallyClosed);
        // m_robotContainer.getArm().ArmSparkMax.getReverseLimitSwitch(Type.kNormallyClosed);
        // m_robotContainer.getArm().wristSparkMax.getForwardLimitSwitch(Type.kNormallyClosed);
    }

    public static double wristDampener = 0.5;
    public static double armDampener = 1;

    // This function is called periodically during operator control.
    @Override
    public void teleopPeriodic() {

        double position = m_robotContainer.getSwerve().getPosition();

        // This command will schedule the robot to drive via teleop if
        // setDefaultCommand isn't used in RobotContainer
        // m_robotContainer.drive_robot.execute();
        NetworkTableInstance.getDefault().getTable("Velocity").getEntry("AvgDriveVelocity")
                .setDouble(SwerveDrive.getAvgVelocity());
        NetworkTableInstance.getDefault().getTable("Velocity").getEntry("FRDriveVelocity")
                .setDouble(SwerveDrive.getFRVelocity());
        NetworkTableInstance.getDefault().getTable("Velocity").getEntry("FlDriveVelocity")
                .setDouble(SwerveDrive.getFLVelocity());
        NetworkTableInstance.getDefault().getTable("Velocity").getEntry("BRDriveVelocity")
                .setDouble(SwerveDrive.getBRVelocity());
        NetworkTableInstance.getDefault().getTable("Velocity").getEntry("BlDriveVelocity")
                .setDouble(SwerveDrive.getBLVelocity());
        NetworkTableInstance.getDefault().getTable("Velocity").getEntry("FrPosition")
                .setDouble(SwerveDrive.getFRPosition());
        NetworkTableInstance.getDefault().getTable("Position").getEntry("X position").setDouble(position);

        // 50.9047 + 75 arm, 203.2 wrist

        // DigitalInput limitSwitchArmUpper = RobotContainer.getLimitSwitchUper();
        // DigitalInput limitSwitchLower = RobotContainer.getLimitSwitchLower();
        // DigitalInput limitSwitchWrist = RobotContainer.getlimitSwitchWrist();

        /**************************************************************************************************************************************/

        // if(Math.abs(m_robotContainer.getController2().getRightY()) >= .05 ){
        // m_robotContainer.getWrist().setWristVoltage(5 *
        // -(m_robotContainer.getController2().getRightY()) * wristDampener);
        // }else{
        // m_robotContainer.getWrist().setWristVoltage(0);
        // }
        if (Math.abs(m_robotContainer.getController2().getLeftY()) >= .05) {
            m_robotContainer.getArm()
                    .setArmVoltage((5 * -(m_robotContainer.getController2().getLeftY()) * armDampener));

        } else {
            m_robotContainer.getArm().setArmVoltage(0);
        }

        if (Math.abs(m_robotContainer.getController2().getRightY()) >= .05) {
            m_robotContainer.getWrist()
                    .setWristVoltage((5 * -(m_robotContainer.getController2().getRightY()) * wristDampener));

        } else {
            m_robotContainer.getWrist().setWristVoltage(0);
        }

        // if (Math.abs(m_robotContainer.getController2().getRightY()) >= .05) {
        //     m_robotContainer.getWrist()
        //             .setWristVoltage((5 * -(m_robotContainer.getController2().getRightY()) * wristDampener));

        // } else {
        //     m_robotContainer.getWrist().setWristVoltage(0);
        // }

        /**************************************************************************************************************************************/

        // Arm Stuff

        // NetworkTableInstance.getDefault().getTable("ARM").getEntry("ArmEncoder").setDouble(Arm1.getArmEncoder());}
        // XboxController Control1 = m_robotContainer.getController1();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    // This function is called periodically during test mode.
    @Override
    public void testPeriodic() {
        // m_robotContainer.getSwerve().setChassisSpeeds(.7, 0, 0);

        // System.out.println(5 * -m_robotContainer.getController2().getLeftY());
        // System.out.println(5 * -m_robotContainer.getController2().getRightY());
        // m_robotContainer.getArm().setArmVoltage((1 * armDampener));
        // m_robotContainer.getArm().setWristVoltage((1 * wristDampener));

        // TODO Fix wrist setVoltage below to setWristAngle and remove Dampner
        // multiplier
        // m_robotContainer.getWrist().setWristVoltage(5 *
        // -(m_robotContainer.getController2().getRightY()) * wristDampener);
        // m_robotContainer.getArm().setArmVoltage((5 * -(m_robotContainer.getController2().getLeftY()) * armDampener));

    }

}