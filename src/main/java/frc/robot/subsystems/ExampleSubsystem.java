// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.hal.simulation.I2CDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.AutoCloseable;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  /*private AddressableLED myLed;
  public AddressableLEDBuffer myLedBuffer;
  private Servo myServo;

  public CANSparkFlex myMotor;
  public RelativeEncoder myEncoder;
  public SparkPIDController myPIDController;
  */

  //public DigitalInput magnetSwitch;
  //public DigitalInput mechanicalSwitch;

  public ExampleSubsystem() {
    /*
    // Get the default instance of NetworkTables that was created automatically
    // when the robot program starts
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // Get the table within that instance that contains the data. There can
    // be as many tables as you like and exist to make it easier to organize
    // your data. In this case, it's a table called datatable.
    NetworkTable table = inst.getTable("datatable");

    // Start publishing topics within that table that correspond to the X and Y values
    // for some operation in your program.
    // The topic names are actually "/datatable/x" and "/datatable/y".
    xPub = table.getDoubleTopic("x").publish();
    yPub = table.getDoubleTopic("y").publish();
    */

    /*
    // Initialize LED and buffer
    myLed = new AddressableLED(1);
    myLedBuffer = new AddressableLEDBuffer(5);
    myLed.setLength(myLedBuffer.getLength());
    myLed.setData(myLedBuffer);
    myLed.start();

    // Initialize Servo
    myServo = new Servo(0);

    myMotor = new CANSparkFlex(52, CANSparkLowLevel.MotorType.kBrushless);
    myEncoder = myMotor.getEncoder(); // getEncoder returns a relative encoder

    myMotor.restoreFactoryDefaults();
    myEncoder.setPosition(0);
    
    myPIDController = myMotor.getPIDController();

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

    // Resetting everything
   
    */

    // magnetSwitch = new DigitalInput(8);
    // mechanicalSwitch = new DigitalInput(9);

  }


  //MOVED TO LED SUBSYSTEM
  /* 
  public int rv;
  public int gv;
  public int bv;

  public enum Color {
    BLUE,
    CYAN,
    GREEN,
    YELLOW,
    ORANGE,
    RED,
    PINK,
    PURPLE,
    COLORLESS
  }

  public class ChooseColor {
    
    Color myColor;

    public ChooseColor (Color color) {
      myColor = color;
    }

    public void setColor() {
      switch(myColor) {
        case BLUE: 
          rv = 0;
          gv = 0;
          bv = 255;
          break;
        
        case CYAN:
          rv = 0;
          gv = 255;
          bv = 255;
          break;

        case GREEN:
          rv = 0;
          gv = 255;
          bv = 0;
          break;
        
        case YELLOW:
          rv = 255;
          gv = 255;
          bv = 0;
          break;
        
        case ORANGE:
          rv = 255;
          gv = 128;
          bv = 13;
          break;
        
        case RED:
          rv = 255;
          gv = 0;
          bv = 0;
          break;
        
        case PINK:
          rv = 255;
          gv = 192;
          bv = 203;
          break;

        case PURPLE:
          rv = 156;
          gv = 81;
          bv = 182;
          break;

        case COLORLESS:
          rv = 0;
          gv = 0;
          bv = 0;
          break;
      }
    }
  }


  public Command changeColorCommand (Color choice) {
    return new FunctionalCommand(

      // INIT
      () -> {},

      // EXECUTE
      () -> {
        ChooseColor color = new ChooseColor(choice);
        color.setColor();
        for (int i = 0; i < myLedBuffer.getLength(); i++) {
          myLedBuffer.setRGB(i, rv, gv, bv);
        }
        myLed.setData(myLedBuffer);
      },

      interrupted -> {},

      () -> false,

    this);

  }

  public void changeColor(Color choice) {
    ChooseColor color = new ChooseColor(choice);
    color.setColor();
    for (int i = 0; i < myLedBuffer.getLength(); i++) {
      myLedBuffer.setRGB(i, rv, gv, bv);
    }
    myLed.setData(myLedBuffer);
  }

*/

  // MOVED TO AUTOSENDABLECHOOSER.JAVA
  /*public Command pinkAuto() {
    return new FunctionalCommand(
      () -> {
        System.out.println("pinkAuto initialized");
      },

      () -> {
        changeColor(Color.PINK);
      },

      interrupted -> {},

      () -> true, 
    
    this);
  }*/

  /*public Command blueAuto() {
    return new FunctionalCommand(
      () -> {
        System.out.println("blueAuto initialized");
      },

      () -> {
        changeColor(Color.BLUE);
      },

      interrupted -> {},

      () -> true, 
    
    this);
  }*/

  /*
  public Command flashColors() {
    return new FunctionalCommand(
    
    // INIT
    () -> {},

    // EXECUTE
    () -> {},

    interrupted -> {},

    // END
    () -> false,

    this);
  }
  */

  /*public Command servoOff() {
    return new FunctionalCommand(

    // ** INIT

    () -> {},

    // ** EXECUTE

    () -> { 
      myServo.set(0.5);
    },

    // ** ON INTERRUPTED

    interrupted -> {},

    // ** END CONDITION
      
    () -> true,
    
    this);
  }*/


  /*
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

        // purple
        ChooseColor purple = new ChooseColor(Color.PURPLE);
        purple.setColor();
        for (var i = 0; i < myLedBuffer.getLength(); i++) {
          myLedBuffer.setRGB(i, ((int)(Math.abs(s)*rv)), ((int)(Math.abs(s))*gv), ((int)((Math.abs(s))*bv)));
        } 
        myLed.setData(myLedBuffer);
        System.out.println("color is purple");
      }
      if (s < 0) {
        myMotor.set(s);
        System.out.println("motor turning counterclockwise");

         // cyan
        ChooseColor cyan = new ChooseColor(Color.CYAN);
        cyan.setColor();
        for (var i = 0; i < myLedBuffer.getLength(); i++) {
          myLedBuffer.setRGB(i, ((int)(Math.abs(s)*rv)), ((int)(Math.abs(s))*gv), ((int)((Math.abs(s))*bv)));
        } 
        myLed.setData(myLedBuffer);
        System.out.println("color is cyan");
      }
    },
   
    // ** ON INTERRUPTED
    interrupted -> {
      myMotor.set(0);
      System.out.println("motor stopped");
      for (var i = 0; i < myLedBuffer.getLength(); i++) {
        myLedBuffer.setRGB(i, 0, 0, 0); // off
      } 
      myLed.setData(myLedBuffer);
    },
   
    // ** END CONDITION
    ()-> false,

    // ** REQUIREMENTS
    this);
  }

  */


  // MOVED TO SERVOSUBSYSTEM
  /* 
  public Command controlSpeed(DoubleSupplier targetValue) 
  {
    return new FunctionalCommand (
      // ** INIT
      () -> {},
   
      // ** EXECUTE
      () -> { 
        System.out.println(targetValue.getAsDouble());
        double t = targetValue.getAsDouble();
        if ( (t < 0 && t > -0.1) || (t > 0 && t < 0.1) ) { // setting deadband range
          t = 0;
        }
        myServo.set((t / -2.0)+0.5);

        for (var i = 0; i < myLedBuffer.getLength(); i++) 
        {
          if (t < 0) { // counterclockwise; joystick value between -1 and 0
            ChooseColor red = new ChooseColor(Color.RED);
            red.setColor();
            myLedBuffer.setRGB(i, (int)(Math.abs(t)*rv), gv, bv); // red color
            System.out.println("color is red; turning ccw");

          } else { // clockwise; joystick value between 0 and 1
            ChooseColor green = new ChooseColor(Color.GREEN);
            green.setColor();
            myLedBuffer.setRGB(i, rv, (int)(Math.abs(t)*gv), bv); // green color
            System.out.println("color is green; turning cw");
          }
        } 
          myLed.setData(myLedBuffer);
      },
   
      // ** ON INTERRUPTED
      interrupted -> { myServo.set(0.5); },
   
      // ** END CONDITION
      () -> false,

      // ** REQUIREMENTS
      this);
  }

  public Command turnServoClockwise () {
    return new FunctionalCommand (
    
    // ** INIT
    () -> {},
   
    // ** EXECUTE
    () -> { 
      myServo.set(0);
      System.out.println("servo turning clockwise");
      changeColor(Color.GREEN);
    },
   
    // ** ON INTERRUPTED
    interrupted -> { 
      myServo.set(0.5); 
      changeColor(Color.COLORLESS);
    },
   
    // ** END CONDITION
    () -> false, // needs to be false when doing while true

    // ** REQUIREMENTS
    this);
  } 

  public Command turnServoCounterClockwise () {
    return new FunctionalCommand(
    
    // ** INIT
    () -> {},
   
    // ** EXECUTE
    () -> { 
      myServo.set(1);
      System.out.println("servo turning counterclockwise");
      changeColor(Color.RED);
    },
   
    // ** ON INTERRUPTED
    interrupted -> { 
      myServo.set(0.5);
      changeColor(Color.COLORLESS);
    },
   
    // ** END CONDITION
    () -> false,

    // ** REQUIREMENTS
    this);
  } */

  /*
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

    () -> false, // needs to be false because thing gets interrupted

    this);
  }

  */

  /*

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

  */

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public void teleopPeriodic() {
    // Publish values that are constantly increasing.
   
  }

  @Override
  public void periodic() {

    // SmartDashboard.putNumber("MOTOR SPEED VALUE", myMotor.get());
 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
