// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;


public class SwerveSubsystem extends SubsystemBase {


  /**
   * Double finals for HALF the length and width of the chassis 
   * 
   * Currently, no neasurements so as seen below, they will be declared to a random number
   */
  final double L;
  final double W;

  /*
   * Drive motor and turning motor (Talon FX motors) 
   * 
   * Turning encoder (for position) (CANcoder)
   * 
   * Turning encoder reads the value from the turning motor only and 
   * uses the PID control to regulate the values
   */

  public TalonFX driveMotor;
  public TalonFX turningMotor;
  public CANcoder turningEncoder;
  public PIDController turningPIDControl;

  public SimpleMotorFeedforward driveFF;

  /*
   * Declaration of PID variables
   */
  private double kP;
  private double kI;
  private double kD;

  // Meant solely for PID tuning command (in order to tune the turning motor)
  private double targetPosition;

  private double driveSpeed;
  private double turnAngle;
  private double turnEncoderPosition;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {

    /*
     * For now, L and W are declared to random variables as there are no measurements
     */
    L = 6;
    W = 6;

    /*
     * Sets motors and encoders to the proper ID
     * 
     * In this case, this module is coding for wheel #4, so tens digit is 4
     * 
     * Ones digit is as follows:
     * 1 - Encoder
     * 2 - Steering
     * 3 - Drive
     */
    driveMotor = new TalonFX(43);
    turningMotor = new TalonFX(42);
    turningEncoder = new CANcoder(41);
    
    //kP = 0.0001;
    //kI = 0;
    //kD = 0;
   
    // Initializes the PID values from before

    turningPIDControl = new PIDController(kP, kI, kD);

    /* Tells the PID controller that 180 and -180 are at the same place
     * 
     */
    turningPIDControl.enableContinuousInput(-180, 180); 
    
  }

  /**
    * Configures the drive motor
    
  public void configDriveMotor(){
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = 110;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Audio.AllowMusicDurDisable = true;

    driveMotor.getConfigurator().apply(config);
  }

*/

  /**
    * Configures the turn motor
    
  public void configTurnMotor(){
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Audio.AllowMusicDurDisable = true;

    turningMotor.getConfigurator().apply(config);
  }

  */

  /**
     * Configures the CANcoder, this involves giving it an offset
     * 
     * @param offset the offset to give the CANcoder
     */
    public void configCANcoder(double offset){
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cancoderConfigs.MagnetSensor.MagnetOffset = offset;
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        turningEncoder.getConfigurator().apply(cancoderConfigs);
    }

    public TalonFX getDriveMotor(){return driveMotor;}
    public TalonFX getTurnMotor(){return turningMotor;}
    public CANcoder getTurnEncoder(){return turningEncoder;}


  /**
   * Swerve drive method.
   * 
   * @param x1 // This represents strafe value on the left x joystick
   * @param y1 // This represents forward value on the left y joystick
   * @param x2 // This represents cw/ccw (rotation) value on the right x joystick
   * @return
   */
  public Command drive (DoubleSupplier x1, DoubleSupplier y1, DoubleSupplier x2) {

    return new FunctionalCommand(
      
    () -> {},

    () -> {
      
      // Strafe
      double xSTR = x1.getAsDouble();

      // Forward
      double yFWD = y1.getAsDouble() * -1; 

      // Rotation
      double xRCW = x2.getAsDouble();

      // Hypotenuse (r value)
      double r = Math.sqrt (Math.pow(L, 2)+Math.pow(W, 2)); 
      
      /*
       * Calculating RCW values.
       * 
       */
      
      double RCW_FWD = xRCW * (L/r); 
      double RCW_STR = xRCW * (W/r); 

      /*
       * Calculating FWD and STR values.
       *       
       * The sign in between the two values is the following based on the quadrant:
       *
       *  Quadrant 1: 
       *  FWD1 - 
       *  STR1 +
       * 
       * Quadrant 2: 
       *  FWD2 -
       *  STR2 -
       * 
       * Quadrant 3:
       *  FWD3 +
       *  STR3 +
       * 
       * Quadrant 4:
       *  FWD4 +
       *  STR4 -
       * 
       * In this case, it is the 4th quadrant, hence the signs
       * 
       */
      double FWD4 = yFWD + RCW_FWD;
      double STR4 = xSTR - RCW_STR;
    
      /*
       * Computes the driveSpeed
       */
      driveSpeed = Math.sqrt(FWD4 * FWD4 + STR4 * STR4);
      
      /*
       * Computes the turnAngle, a number between -180 and 180 
       *      STILL NEED TO TEST THIS!!! (turn off any drive commands and view shuffleboard)
       */
      turnAngle = Math.toDegrees(Math.atan2(FWD4, STR4));

      // THE FOLLOWING IS NO LONGER NEEDED BECAUSE ATAN WAS SWAPPED FOR ATAN2
      // (but it's just here in case)

      /*
      // meant to match up the input angle with the encoder angle
      if (xSTR > 0) {
        if (yFWD > 0) {
          backRightAngle *= -1; // converts calculated angle to encoder value
        } else if (yFWD < 0) {
          backRightAngle = -180 - backRightAngle;
        }
      } else if (xSTR < 0) {
        if (yFWD < 0) {
          backRightAngle = 180 - backRightAngle;
        } else if (yFWD > 0) {
          backRightAngle *= -1;
        }
      }
      */

      // Following code calls to actually send these computed vaulues to the motors.

      // Drive speed has been set.
      driveMotor.set(driveSpeed);

      /* 
       * Retrieves position of turning motor from the encoder
       * 
       * Value is returned in DEGREES, between the values of -180 and 180
       */
      turnEncoderPosition = turningEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI * (180/Math.PI);
      
      // Uses PID control to bring the turning motor to the ideal position
      // Compares actual position by reading from encoder to the setpoint angle
      turningMotor.set(turningPIDControl.calculate(turnEncoderPosition, turnAngle));
      
    },

    interrupted -> {},

    () -> false,


    this);
   

  }

  /*
   * This command allows for PID tuning of the Talon FX motors.
   * 
   * Meant purely for testing purposes
   * 
   * Precondition : drive method is commented out, this method is set as default method
   * 
   */
  public Command PIDTest () {
    return new FunctionalCommand(
      
    () -> {

      // Updates values changed from Shuffleboard.
      
      loadPreferences();

    },
    
    () -> {

      turnEncoderPosition = turningEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI * (180/Math.PI);

      turningMotor.set(turningPIDControl.calculate(turnEncoderPosition, targetPosition));

    }, 
    
    interrupted -> {}, 
    
    () -> false, 
    
    this);
  }

  /*
   * Helper method for relaying inputted PID values from shuffleboard to WPILIB
   */
  public void loadPreferences() {
    // Load Preferences for kP
    kP = Preferences.getDouble(Constants.PIDConstants.pKey, kP);
    if (kP != Preferences.getDouble(Constants.PIDConstants.pKey, kP)) {
      kP = Preferences.getDouble(Constants.PIDConstants.pKey, kP);
      turningPIDControl.setP(kP);
    }

    // Load Preferences for kI
    kI = Preferences.getDouble(Constants.PIDConstants.iKey, kI);
    if (kI != Preferences.getDouble(Constants.PIDConstants.iKey, kI)) {
      kI = Preferences.getDouble(Constants.PIDConstants.iKey, kI);
      turningPIDControl.setI(kI);
    }

    // Load Preferences for kD
    kD = Preferences.getDouble(Constants.PIDConstants.dKey, kD);
    if (kD != Preferences.getDouble(Constants.PIDConstants.dKey, kD)) {
      kD = Preferences.getDouble(Constants.PIDConstants.dKey, kD);
      turningPIDControl.setD(kD);
    }
    
    // Load Preferences for set point
    targetPosition = Preferences.getDouble(Constants.PIDConstants.tpKey, targetPosition);
    if (targetPosition != Preferences.getDouble(Constants.PIDConstants.tpKey, targetPosition)) {
      targetPosition = Preferences.getDouble(Constants.PIDConstants.tpKey, targetPosition);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Following three values return values pertaining to the turning angle
    SmartDashboard.putNumber("(DEGREES) Module 4 Turning Motor Position from CANcoder", turningEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI * (180/Math.PI));

    SmartDashboard.putNumber("Module 4 PID Output", turningPIDControl.calculate(turnEncoderPosition, turnAngle));
          
    SmartDashboard.putNumber("(DEGREES) Module 4 Setpoint Angle", turnAngle);
    
    // Both the following return speed of drive motor (2 different ways)
    SmartDashboard.putNumber("Module 4 Velocity", driveMotor.getVelocity().getValueAsDouble());

    SmartDashboard.putNumber("Module 4 Speed", driveSpeed);

    // The following put values for PID control testing
    // THESE VALUES ARE ALSO EDITABLE!!
    SmartDashboard.putNumber(Constants.PIDConstants.pKey, kP);
    SmartDashboard.putNumber(Constants.PIDConstants.iKey, kI);
    SmartDashboard.putNumber(Constants.PIDConstants.dKey, kD);
    SmartDashboard.putNumber(Constants.PIDConstants.tpKey, targetPosition);
  }

}

