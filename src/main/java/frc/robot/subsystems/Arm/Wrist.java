package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Comms3140;

import java.util.function.Supplier;

public class Wrist extends SubsystemBase {
  // ~.89 in cycles, 
  private final double minAngleWrist = 0;
  private final double maxAngleWrist = 180;

  // Wrist Absolute Encoder
  public final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(1);
  public final CANSparkMax wristSparkMax;
  private double wristP;
  private double wristD;
  private PIDController wristPidController = new PIDController(wristP, 0.001, wristD);
  private double WristAngle;
  private double WristAngleSetPt = getWristAngle();
  // out of 12V
  private double maxVoltage = 7;
  private double motorVoltage = 0;

  double wristLength = Units.feetToMeters(23.75 / 12);
  DigitalInput limitSwitchWrist = RobotContainer.getlimitSwitchWrist();

  // TODO: we should implement these controls
  double maxAngle=170;
  double minAngle=15;
  //double maxUpVoltage = 7.0;
  //double minDownVoltage = 7.0;
  
  // TODO: This did not work remove it
  //LinearFilter angle_filter = LinearFilter.singlePoleIIR(3.0,0.02 );


  Comms3140 comms = Comms3140.getInstance();


  public Wrist(int WristID) {
    wristSparkMax = new CANSparkMax(WristID, MotorType.kBrushless);

    wristSparkMax.restoreFactoryDefaults();
    wristSparkMax.setSmartCurrentLimit(20);
    wristSparkMax.setIdleMode(IdleMode.kBrake);
    wristSparkMax.burnFlash();

    wristSparkMax.getEncoder().setPosition(0.0);

    wristPidController.enableContinuousInput(0, 360);

    // Initial P and D values for the PID Controller
// NetworkTableInstance.getDefault().getTable("Arm").getEntry("Wrist P").setDouble(0.027074);
    //.03
    //NetworkTableInstance.getDefault().getTable("Arm").getEntry("Wrist D").setDouble(0.015);
    //.02

      registerSettings();

  }

  private void registerSettings(){
    comms.registerDoubleSetting("Wrist", "Controler P", wristPidController::getP, wristPidController::setP,0.027);
    comms.registerDoubleSetting("Wrist", "Controler I", wristPidController::getI, wristPidController::setI,0.0);
    comms.registerDoubleSetting("Wrist", "Controler D", wristPidController::getD, wristPidController::setD,0.015);
    comms.registerDoubleSetting("Wrist", "Set Point", wristPidController::getSetpoint, wristPidController::setSetpoint,0.0);
  }

  public void setWristVoltage(double wristVoltage) {
  //System.out.println(getWristAngle());
    if (Math.abs(motorVoltage) > maxVoltage) {
      motorVoltage = Math.signum(motorVoltage) * maxVoltage;
    }
    if (!limitSwitchWrist.get()) {
    wristSparkMax.setVoltage(Math.min(wristVoltage, 0));
    } else if (getWristAngle() < 300 && getWristAngle() > 210) {//|| 
    wristSparkMax.setVoltage(Math.max(wristVoltage, 0));
    } else {
    wristSparkMax.setVoltage(wristVoltage);
    }
  }
  double kS = .082436; 
  double kV = .027445;
  double kA = .0062396;
  double kG = .086783;
  ArmFeedforward feedForward = new ArmFeedforward(kS, kG, kV, kA);
  double lastSetPoint;
  public void moveWrist(){
    if (Math.abs(RobotContainer.getInstance().getController2().getRightY()) >= .05) {
      setWristVoltage((5 * -(RobotContainer.getInstance().getController2().getRightY())));
      lastSetPoint = getWristAngle();
    } else if(lastSetPoint - getWristAngle() > 5) {    
      setWristVoltage(
        wristPidController.calculate(getWristAngle(), lastSetPoint)
      );
    }
  
    }

  @Override
  public void periodic() {
    
    double angle = getWristAngle();
    // double filtered_angle = angle_filter.calculate(angle);
    
    comms.sendDoubleTelemetry("Wrist", "Angle",angle);
    comms.sendDoubleTelemetry("Wrist", "Set Point",wristPidController.getSetpoint());
    //comms.sendDoubleTelemetry("Wrist", "Angle Filtered",filtered_angle);

    // TODO: Remove network table code.  Replaced by Comms3140
    //NetworkTableInstance.getDefault().getTable("Arm").getEntry("WristAngle").setDouble(getWristAngle());
    //NetworkTableInstance.getDefault().getTable("Arm").getEntry("WristAngleSetPt").setDouble(WristAngleSetPt);
    //NetworkTableInstance.getDefault().getTable("Arm").getEntry("Wrist PID Output").setDouble(wristPidController.calculate(WristAngle));
    //NetworkTableInstance.getDefault().getTable("Arm").getEntry("Wrist Voltage").setDouble(motorVoltage);
    //wristP = NetworkTableInstance.getDefault().getTable("Arm").getEntry("Wrist P").getDouble(0.0);
    //wristD = NetworkTableInstance.getDefault().getTable("Arm").getEntry("Wrist D").getDouble(0.0);

    //if (wristP != wristPidController.getP() || wristD != wristPidController.getD()) {
    //  wristPidController.setP(wristP);
    //  wristPidController.setD(wristD);
    //}

  
    motorVoltage = wristPidController.calculate(angle);

    comms.sendDoubleTelemetry("Wrist", "Voltage Raw", motorVoltage);
    //iss
    // cap output to +/- maxvoltage
    

    comms.sendDoubleTelemetry("Wrist", "Voltage", motorVoltage);
    //setWristVoltage(motorVoltage);
  }

  // Getter methods
  public DutyCycleEncoder getWristEncoder() {
    return wristEncoder;
  }

  public double getWristAngle() {
    // the .892239 is correction factor that aligns the encoder to have 0 at the
    // minimum angle
    double a = ((getWristEncoder().getAbsolutePosition() - .892239)) * 360;
    return a < 0 ? a + 360 : a;
  }

  // Setter methods
  public void setWristAngle(double angle) {
    WristAngleSetPt = Math.min(Math.max(angle, minAngleWrist), maxAngleWrist);
    wristPidController.setTolerance(3);
    WristAngle = getWristAngle();

    wristPidController.setSetpoint(WristAngleSetPt);
    motorVoltage = wristPidController.calculate(WristAngle);
    //cap output to +/- maxvoltage
    if (Math.abs(motorVoltage) > maxVoltage) {
     motorVoltage = Math.signum(motorVoltage) * maxVoltage;
    }

   // setWristVoltage(motorVoltage);

  }
}
