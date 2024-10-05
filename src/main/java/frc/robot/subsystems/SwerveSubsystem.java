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

    L = 0;
    W = 0;

    driveMotor = new TalonFX(23);
    turningMotor = new TalonFX(22);
    turningEncoder = new CANcoder(21);
    
    double kP = 0.0003;
    double kI = 0.000001;
    double kD = 0.00001;
   
    turningPIDControl = new PIDController(kP, kI, kP);

    turningPIDControl.enableContinuousInput(-180, 180); //Tells the PID controller that 180 and -180 are at the same place

  }

  /**
    * Configures the drive motor
    */
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

  /**
    * Configures the turn motor
    */
  public void configTurnMotor(){
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Audio.AllowMusicDurDisable = true;

    turningMotor.getConfigurator().apply(config);
  }

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

    return new RunCommand(
      
    () -> {
      
      double xSTR = x1.getAsDouble();
      double yFWD = y1.getAsDouble(); // *1;
      double xRCW = x2.getAsDouble();

      double r = Math.sqrt ((L*L)+(W*W)); // side length of triangle created by width and length
      
      double fwd = yFWD - xRCW * (L/r); // going forward
      double bkwd = yFWD + xRCW * (L/r); // going backward
      double rightStr = xSTR + xRCW * (W/r); // going right
      double leftStr = xSTR - xRCW * (W/r); // going left
    
      // only one swerve drive thingy thing for now in quadrant 1
      double frontLeftSpeed = Math.sqrt(fwd * fwd + leftStr * leftStr);
      
      double frontLeftAngle = Math.atan(leftStr/fwd);

      driveMotor.set(frontLeftSpeed);

      turningMotor.setPosition(turningPIDControl.calculate((turningEncoder.getAbsolutePosition()).getValueAsDouble() * 2 * Math.PI), frontLeftAngle);

      // rotPIDController.setReference(frontLeftAngle, CANSparkMax.ControlType.kPosition);

      SmartDashboard.putNumber("Module 2 PID Output", turningPIDControl.calculate((turningEncoder.getAbsolutePosition()).getValueAsDouble() * 2 * Math.PI));

      SmartDashboard.putNumber("Module 2 Velocity", driveMotor.getVelocity().getValueAsDouble());

    }, 
    
    this);
   

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}

