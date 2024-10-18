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

  final double L;
  final double W;

  public TalonFX driveMotor;
  public TalonFX turningMotor;
  public CANcoder turningEncoder;

  public PIDController turningPIDControl;

  public SimpleMotorFeedforward driveFF;


  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {

    L = 6;
    W = 6;

    driveMotor = new TalonFX(43);
    turningMotor = new TalonFX(42);
    turningEncoder = new CANcoder(41);
    
    double kP = 0.1;
    double kI = 0;
    double kD = 0;
   
    turningPIDControl = new PIDController(kP, kI, kD);

    turningPIDControl.enableContinuousInput(-180, 180); // Tells the PID controller that 180 and -180 are at the same place
    
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
     * Configures the CANcoder,  this involves giving it an offset
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


  public Command drive (DoubleSupplier x1, DoubleSupplier y1, DoubleSupplier x2) {

    return new FunctionalCommand(
      
    () -> {},

    () -> {
      
      double xSTR = x1.getAsDouble();
      double yFWD = y1.getAsDouble() * -1;
      double xRCW = x2.getAsDouble();

      double r = Math.sqrt ((L*L)+(W*W)); // side length of triangle created by width and length
      
      double RCW_fwd = yFWD - xRCW * (L/r); // going forward
      double RCW_bkwd = yFWD + xRCW * (L/r); // going backward
      double RCW_rightStr = xSTR + xRCW * (W/r); // going right
      double RCW_leftStr = xSTR - xRCW * (W/r); // going left
    
      // only one swerve drive for now in quadrant 4
      double backRightSpeed = Math.sqrt(RCW_bkwd * RCW_bkwd + RCW_rightStr * RCW_rightStr);
      
      double backRightAngle = Math.toDegrees(Math.atan(RCW_rightStr/RCW_bkwd));

      if (RCW_rightStr > 0) {
        backRightAngle -= 90;
      } else if (RCW_rightStr < 0) {
        backRightAngle += 90;
      }

      // driveMotor.set(backRightSpeed);

      turningMotor.getPosition();

      double encoderPosition = turningEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI * (180/Math.PI);
      
      // turningMotor.set(turningPIDControl.calculate(encoderPosition, backRightAngle));

      SmartDashboard.putNumber("Module 4 PID Output", turningPIDControl.calculate(encoderPosition, backRightAngle));

      SmartDashboard.putNumber("Module 4 Setpoint Angle", backRightAngle);

      SmartDashboard.putNumber("Module 4 Velocity", driveMotor.getVelocity().getValueAsDouble());

      SmartDashboard.putNumber("Module 4 Setpoint Speed", backRightSpeed);
    },

    interrupted -> {},

    () -> false,


    this);
   

  }

  public Command motorTest () {
    return new FunctionalCommand(
      
    () -> {},
    
    () -> {
      driveMotor.set(0.3);
      System.out.println("Speed adjusted");
    }, 
    
    interrupted -> {}, 
    
    () -> false, 
    
    this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
          SmartDashboard.putNumber("CANCoder Value", turningEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI);

          SmartDashboard.putNumber("1 CANcoder value", turningEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI * (180/Math.PI));

  }

}

