// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem.ChooseColor;
import frc.robot.subsystems.LEDSubsystem.Color;
import frc.robot.subsystems.LEDSubsystem;

public class CANSparkFlexSubsystem extends SubsystemBase {

  public CANSparkFlex myMotor;
  public RelativeEncoder myEncoder;
  public SparkPIDController myPIDController;
  private final LEDSubsystem motorLED = new LEDSubsystem();

  public DigitalInput mechanicalSwitch; 
  public DigitalInput magnetSwitch;

  /** Creates a new CANSparkFlex. */
  public CANSparkFlexSubsystem() {

    myMotor = new CANSparkFlex(52, CANSparkLowLevel.MotorType.kBrushless);
    myEncoder = myMotor.getEncoder(); // getEncoder returns a relative encoder

    myMotor.restoreFactoryDefaults();
    myEncoder.setPosition(0);
    
    myPIDController = myMotor.getPIDController();

    // motorLED = new LEDSubsystem();

    mechanicalSwitch = new DigitalInput(9);
    magnetSwitch = new DigitalInput(8);

    //myPIDController = new PIDController(Constants.PIDConstants.kP, Constants.PIDConstants.kI, Constants.PIDConstants.kD);

    //myPIDController.setReference(Constants.PIDConstants.setPoint, CANSparkBase.ControlType.kVelocity);
 
    // Set the P value of the PID controller on slot 0 to 0.25
    myPIDController.setP(Constants.PIDConstants.kP);
    myPIDController.setI(Constants.PIDConstants.kI);
    myPIDController.setD(Constants.PIDConstants.kD);
    myPIDController.setIZone(Constants.PIDConstants.kIz);

    // Set the minimum and maximum outputs of the motor [-1, 1]
    myPIDController.setOutputRange(Constants.PIDConstants.kMinOutput, Constants.PIDConstants.kMaxOutput);

    // Set kFF
    myPIDController.setFF(Constants.PIDConstants.kFF);
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

          // Set color to purple
          motorLED.changeColor(Color.PURPLE, s);
          System.out.println("color is purple");
        }
        if (s < 0) {
          myMotor.set(s);
          System.out.println("motor turning counterclockwise");

          // cyan
          motorLED.changeColor(Color.CYAN, s);
          System.out.println("color is cyan");
        }
      },
   
      // ** ON INTERRUPTED
      interrupted -> {
        myMotor.set(0);
        System.out.println("motor stopped");
        motorLED.changeColor(Color.COLORLESS, 0);
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
        SmartDashboard.putNumber("kP value", Constants.PIDConstants.kP);
        SmartDashboard.putNumber("kI value", Constants.PIDConstants.kI);
        SmartDashboard.putNumber("kD value", Constants.PIDConstants.kD);
        SmartDashboard.putNumber("Setpoint RPM", Constants.PIDConstants.setPointRPM);

        myPIDController.setReference(Constants.PIDConstants.setPointRPM, CANSparkMax.ControlType.kVelocity);

        SmartDashboard.putNumber("Motor Speed", myEncoder.getVelocity());

        double speed = myMotor.get();

        SmartDashboard.putNumber("SPEED", speed);


        System.out.println("Set point velocity set");
      },

      interrupted -> {
  
        myPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
      },

      () -> false,

    this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
