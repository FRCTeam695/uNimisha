// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.Color;
import edu.wpi.first.wpilibj.Preferences;


public class ServoSubsystem extends SubsystemBase {

    private Servo myServo;
    //private final LEDSubsystem servoLED = new LEDSubsystem();

    public ServoSubsystem() {

        myServo = new Servo(0);

    }

    public Command servoOff() {
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
    }

    public Command controlSpeed(DoubleSupplier targetValue) {
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

                if (t < 0) { // counterclockwise; joystick value between -1 and 0
                   // servoLED.changeColor(Color.RED, t);
                    System.out.println("color is red; turning ccw");
                } else { // clockwise; joystick value between 0 and 1
                   // servoLED.changeColor(Color.GREEN, t);
                    System.out.println("color is green; turning cw");
                }
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
      //servoLED.changeColor(Color.GREEN, 1);
    },
   
    // ** ON INTERRUPTED
    interrupted -> { 
      myServo.set(0.5); 
      //servoLED.changeColor(Color.COLORLESS, 0);
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
      //servoLED.changeColor(Color.RED, 1);
    },
   
    // ** ON INTERRUPTED
    interrupted -> { 
      myServo.set(0.5);
      //servoLED.changeColor(Color.COLORLESS, 0);
    },
   
    // ** END CONDITION
    () -> false,

    // ** REQUIREMENTS
    this);
  } 
}
