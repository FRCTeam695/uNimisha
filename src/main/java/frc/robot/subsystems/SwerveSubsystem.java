// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;


public class SwerveSubsystem extends SubsystemBase {


  /**
   * Double finals for HALF the length and width of the chassis 
   * 
   * Currently, no measurements so as seen below, they will be declared to a random number
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
  public TalonFXConfiguration configDrive;

  public TalonFX turningMotor;
  public TalonFXConfiguration configTurn;

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

  // Calculated version of angle for Talon FX relative encoder
  private double relativeAngle;

  private double xSTR;
  private double yFWD;
  private double xRCW;

  // Orchestra
  private Orchestra orchestra;

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

    configDrive = new TalonFXConfiguration();
    configDrive.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    turningMotor = new TalonFX(42);

    configTurn = new TalonFXConfiguration();
    configTurn.MotorOutput.NeutralMode = NeutralModeValue.Brake; 

    turningEncoder = new CANcoder(41);
    
    kP = 0.005;
    kI = 0;
    kD = 0;
   
    // Initializes the PID values from before

    turningPIDControl = new PIDController(kP, kI, kD);

    /* Tells the PID controller that 180 and -180 are at the same place
     * 
     */
    turningPIDControl.enableContinuousInput(-180, 180); 


    // CANcoder range is -0.5 to 0.5
    // 0.5/180 * 17 degrees (position to 0) 
    // configCANcoder(-0.04722222222);

    orchestra = new Orchestra();
    orchestra.addInstrument(driveMotor);
    
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


  /**
   * Swerve drive method.
   * 
   * @param x1 // This represents strafe value on the left x joystick
   * @param y1 // This represents forward value on the left y joystick
   * @param x2 // This represents cw/ccw (rotation) value on the right x joystick
   * @return
   */
  public Command drive (DoubleSupplier x1, DoubleSupplier y1, DoubleSupplier x2) {

    return new RunCommand(
   
    () -> {

      double strafe = x1.getAsDouble() * -1;
      double forward = y1.getAsDouble() * -1;
      double rotation = x2.getAsDouble() * -1;

      SmartDashboard.putNumber("Post deadband x1", strafe);
      SmartDashboard.putNumber("Post deadband y1", forward);
      SmartDashboard.putNumber("Post deadband x2", rotation);

      // If any value is being changed it means control input is being given and will
      // therefore do the swerve calculations.
      // Command will NOT execute if all 3 controls are let go.
      // Requires at least one of the joystick axes to be pressed
      if (Math.abs(strafe) > 0.05 || Math.abs(forward) > 0.05 || Math.abs(rotation) > 0.05) {
        calculate(strafe, forward, rotation);
        drive();
      } else {
        driveMotor.set(0);  // Stop drive motor
        turningMotor.set(0);
        // turningMotor.set(turningPIDControl.calculate(turnEncoderPosition, turnAngle));
      }
      

    },

    this);
   
  }
  
  public void calculate (double str, double fwd, double rcw) {
    
    // Strafe
    xSTR = str;

    // Forward
    yFWD = fwd;

    // Rotation
    xRCW = rcw;

    /* 
    if (xSTR == 0 && yFWD == 0 && xRCW == 0) {
        driveMotor.set(0);
    } else {
    */

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
      * 
      * But, in this case, a number between -177 and 183
      * 
      */
    turnAngle = Math.toDegrees(Math.atan2(STR4, FWD4)) + 3;

    SmartDashboard.putNumber("FWD4", FWD4);
    SmartDashboard.putNumber("STR4", STR4);




      
  }

  /**
   * Calls to send computer values to the motors
   */
  public void drive() {
      
      /* 
       * Retrieves position of turning motor from the encoder
       * 
       * Value is returned in DEGREES, between the values of -180 and 180
       */
      turnEncoderPosition = turningEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI * (180/Math.PI);

      double err = Math.abs(turnEncoderPosition - turnAngle - 3);

      if (err >= 90) {
        turnAngle -= 90;
        driveSpeed *= -1;
      }

      SmartDashboard.putNumber("error", err);


      // Motor has 4096 pulses per revolution, and 360/4096 = 0.087890625.
      // Dividing the angle by 0.087890625 will give the position meant for the relative encoder.
      // turningMotor.set(turningPIDControl.calculate(turningMotor.getPosition().getValueAsDouble(), turnAngle/0.087890625));
      // relativeAngle = turnAngle/0.087890625;

      // Drive speed has been set.
      driveMotor.set(driveSpeed);

      // Uses PID control to bring the turning motor to the ideal position
      // Compares actual position by reading from encoder to the setpoint angle
      turningMotor.set(turningPIDControl.calculate(turnEncoderPosition, turnAngle));
  }

  public Command faceForward () {
    return new FunctionalCommand(
      () -> {}, 
      
      () -> {

        turnEncoderPosition = turningEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI * (180/Math.PI);

        turningMotor.set(turningPIDControl.calculate(turnEncoderPosition, 3));

        // turningMotor.setPosition(0);

      },
      
      interrupted -> {}, 
      
      () -> false,
      
      this);
  }

  public void playNote() {
    
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

    },
    
    () -> {

      // Updates values changed from Shuffleboard.
      
      // loadPreferences();

      targetPosition = 0;

      turnEncoderPosition = turningEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI * (180/Math.PI);

      System.out.println("Motor set to target speed");

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

    // Relative encoder on the Talon FX position
    SmartDashboard.putNumber("Turn Motor Relative Encoder Position", turningMotor.getPosition().getValueAsDouble());

    // Relative encoder set point for Talon FX
    SmartDashboard.putNumber("Relative encoder set point", relativeAngle);

    SmartDashboard.putNumber("xSTR", xSTR);
    SmartDashboard.putNumber("yFWD", yFWD);
    SmartDashboard.putNumber("xRCW", xRCW);

    // Input values
    SmartDashboard.putNumber("x1", RobotContainer.m_driverController.getRawAxis(0));
    SmartDashboard.putNumber("y1", RobotContainer.m_driverController.getRawAxis(1));
    SmartDashboard.putNumber("x2", RobotContainer.m_driverController.getRawAxis(4));


  }

  public TalonFX getDriveMotor(){return driveMotor;}
  public TalonFX getTurnMotor(){return turningMotor;}
  public CANcoder getTurnEncoder(){return turningEncoder;}
}

