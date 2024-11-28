// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDSubsystem.ChooseColor;
import frc.robot.subsystems.LEDSubsystem.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.RobotContainer;


public class CANSparkFlexSubsystem extends SubsystemBase {

  public SparkMax myMotor;
  public RelativeEncoder myEncoder;
  public SparkClosedLoopController myPIDController;
  // public final LEDSubsystem motorLED = new LEDSubsystem();

  public DigitalInput mechanicalSwitch; 
  public DigitalInput magnetSwitch;
 
  double kP = 0.0003;
  double kI = 0.000001;
  double kD = 0.00001;
  double kIz = 0;

  double kMinOutput = -1;
  double kMaxOutput = 1;

  double kFF = 0.0001;

  double setPointRPM = 1427; 
  

  /** Creates a new CANSparkFlex. */
  public CANSparkFlexSubsystem() {

    myMotor = new SparkMax(52, MotorType.kBrushless);

    ClosedLoopConfig controllerConfig = new ClosedLoopConfig();
    controllerConfig.p(kP);
    controllerConfig.i(kI);
    controllerConfig.d(kD);

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.smartCurrentLimit(0, 15, 0);
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.apply(controllerConfig);
    myMotor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    myEncoder = myMotor.getEncoder(); // getEncoder returns a relative encoder
    myEncoder.setPosition(0);
    myPIDController = myMotor.getClosedLoopController();

    // motorLED = new LEDSubsystem();

    mechanicalSwitch = new DigitalInput(9);
    magnetSwitch = new DigitalInput(8);

    Preferences.initDouble(Constants.PIDConstants.pKey, kP);
    Preferences.initDouble(Constants.PIDConstants.iKey, kI);
    Preferences.initDouble(Constants.PIDConstants.dKey, kD);
    Preferences.initDouble(Constants.PIDConstants.ffKey, kFF);
    Preferences.initDouble(Constants.PIDConstants.spKey, setPointRPM);

  }

  public Command turnMotor(DoubleSupplier speed) {
    return new FunctionalCommand(
    
      // ** INIT
      () -> {},

      // ** EXECUTE
      ()-> { 
        double s = speed.getAsDouble();
        System.out.println(s);
        if ( (s < 0 && s > -0.1) || (s > 0 && s < 0.1) ) { // setting deadband range
          s = 0;
        }
        SmartDashboard.putNumber("Motor Speed", s);

        if (s > 0) {
          myMotor.set(s);
          System.out.println("motor turning clockwise");
          RobotContainer.getLED().changeColor(Color.PURPLE, 1);
        }
        if (s < 0) {
          myMotor.set(s);
          System.out.println("motor turning counterclockwise");
          RobotContainer.getLED().changeColor(Color.CYAN, 1);

        }        
      },
   
      // ** ON INTERRUPTED
      interrupted -> {
        myMotor.set(0);
        System.out.println("motor stopped");
        RobotContainer.getLED().changeColor(Color.COLORLESS, 1);
      },
   
      // ** END CONDITION
      ()-> false,

      // ** REQUIREMENTS
    this);
  }

  public Command turnMotorOnToggle() {
    return new FunctionalCommand(

      // init
      () -> {},

      // execute
      () -> {
        System.out.println(mechanicalSwitch.get());
        if (mechanicalSwitch.get() == true ) {
          myMotor.set(0.07);
        } else {
          myMotor.set(0);
        }
      },

      interrupted -> 
      {
        myMotor.set(0);
      },

      () -> false, // needs to be false because it gets interrupted

    this);
  }

  public Command turnAtSetPoint() {
    return new FunctionalCommand (

      // init
      () -> {},

      // execute
      () -> {

        // Sets the error tolerance to 5, and the error derivative tolerance to 10 per second
        // myPIDController.setTolerance(0, 0.1);

        // Returns true if the error is less than 5 units, and the
        // error derivative is less than 10 units
        // myPIDController.atSetpoint();

        System.out.println("Button pressed, motor turning");
        SmartDashboard.putNumber(Constants.PIDConstants.pKey, kP);
        SmartDashboard.putNumber(Constants.PIDConstants.iKey, kI);
        SmartDashboard.putNumber(Constants.PIDConstants.dKey, kD);
        SmartDashboard.putNumber("Setpoint RPM", setPointRPM);

        myPIDController.setReference(setPointRPM, SparkMax.ControlType.kVelocity);

        SmartDashboard.putNumber("Motor Speed", myEncoder.getVelocity());

        double speed = myMotor.get();

        SmartDashboard.putNumber("SPEED", speed);


        System.out.println("Set point velocity set");

        loadPreferences(); // supposed to update values based off shuffleboard

        System.out.println("New prefs loaded");

      },

      interrupted -> {
  
        myPIDController.setReference(0, SparkMax.ControlType.kVelocity);
      },

      () -> false,

    this);
  }

  public void loadPreferences() {
    // Load Preferences for kP
    kP = Preferences.getDouble(Constants.PIDConstants.pKey, kP);
    if (kP != Preferences.getDouble(Constants.PIDConstants.pKey, kP)) {
      kP = Preferences.getDouble(Constants.PIDConstants.pKey, kP);
      myPIDController.setReference(kP, SparkMax.ControlType.kVelocity);
    }
    // Load Preferences for kI
    kI = Preferences.getDouble(Constants.PIDConstants.iKey, kI);
    if (kI != Preferences.getDouble(Constants.PIDConstants.iKey, kI)) {
      kI = Preferences.getDouble(Constants.PIDConstants.iKey, kI);
      myPIDController.setReference(kI, SparkMax.ControlType.kVelocity);
    }
    // Load Preferences for kD
    kD = Preferences.getDouble(Constants.PIDConstants.dKey, kD);
    if (kD != Preferences.getDouble(Constants.PIDConstants.dKey, kD)) {
      kD = Preferences.getDouble(Constants.PIDConstants.dKey, kD);
      myPIDController.setReference(kD, SparkMax.ControlType.kVelocity);
    }
    // Load Preferences for kFF
    kFF = Preferences.getDouble(Constants.PIDConstants.ffKey, kFF);
    if (kFF != Preferences.getDouble(Constants.PIDConstants.ffKey, kFF)) {
      kFF = Preferences.getDouble(Constants.PIDConstants.ffKey, kFF);
      myPIDController.setReference(kFF, SparkMax.ControlType.kVelocity);
    }
    // Load Preferences for set point
    setPointRPM = Preferences.getDouble(Constants.PIDConstants.spKey, setPointRPM);
    if (setPointRPM != Preferences.getDouble(Constants.PIDConstants.spKey, setPointRPM)) {
      setPointRPM = Preferences.getDouble(Constants.PIDConstants.spKey, setPointRPM);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
