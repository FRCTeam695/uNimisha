// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.CANSparkFlexSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ServoSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LEDSubsystem.Color;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public  DriveTrainSubsystem DT = new DriveTrainSubsystem();

  // public  ExampleSubsystem nimishaSubsystem = new ExampleSubsystem();

  public ServoSubsystem Servo = new ServoSubsystem();

  public CANSparkFlexSubsystem CSP = new CANSparkFlexSubsystem();

  public static LEDSubsystem LED = new LEDSubsystem();

  public SwerveSubsystem Swerve = new SwerveSubsystem();

  final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // public final AddressableLED myLed = new AddressableLED(1);
  // public final AddressableLEDBuffer myLedBuffer = new AddressableLEDBuffer(5);

  //npublic final DrivetrainCommand m_driveCommand = new DrivetrainCommand(m_exampleSubsystem); 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static CommandXboxController m_driverController =
      new CommandXboxController(0); 

  public static CommandJoystick m_leftJoystick = new CommandJoystick(0);
  public static CommandJoystick m_rightJoystick = new CommandJoystick(1);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    System.out.println("ROBOTCONTAINER INSTALLED");

    // Configure the trigger bindings
    configureBindings();

    m_chooser.addOption("Pink Auto", LED.changeColorCommand(Color.PINK, 1));
    System.out.println("Pink option added to chooser");
    m_chooser.addOption("Blue Auto", LED.changeColorCommand(Color.BLUE, 1));
    System.out.println("Blue option added to chooser");
    SmartDashboard.putData(m_chooser);
    System.out.println("Autonomous options added to chooser");
    
  }

  public static LEDSubsystem getLED() {return LED;}

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi..wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // m_exampleSubsystem.setDefaultCommand(m_exampleSubsystem.arcadeDrive( () -> m_leftJoystick.getRawAxis(1), () -> (m_rightJoystick.getRawAxis(0))));

    // nimishaSubsystem.setDefaultCommand(nimishaSubsystem.turnAtSetPoint());

    
    Swerve.setDefaultCommand(Swerve.drive(

      () -> (deadband(m_driverController.getRawAxis(0), 0, 0)),

      () -> (deadband(m_driverController.getRawAxis(1), 0, -0.0008)),

      () -> (deadband(m_driverController.getRawAxis(4), 0, -0.024))

    ));
  

    //Swerve.setDefaultCommand(Swerve.PIDTest());

    // Swerve.setDefaultCommand(Swerve.motorTest());

    System.out.println("configuring button bindings");

    m_driverController.a().whileTrue(Swerve.faceForward());
    
    //m_driverController.x().onTrue(nimishaSubsystem.changeColorCommand(Color.BLUE));
    m_driverController.x().whileTrue(CSP.turnMotorOnToggle());
    m_driverController.y().onTrue(LED.changeColorCommand(Color.YELLOW, 1));
    // m_driverController.a().whileTrue(CSP.turnAtSetPoint());
    m_driverController.b().onTrue(
      LED.changeColorCommand(Color.RED, 1)
      //.andThen(new WaitCommand(1))
      //.andThen(m_exampleSubsystem.turnServoClockwise())
      //.andThen(new WaitCommand(1))
      //.andThen(m_exampleSubsystem.servoOff())
    );
    m_driverController.back().whileTrue(
      CSP.turnMotor(() -> m_driverController.getRawAxis(0))
    );
    m_driverController.start().whileTrue(Servo.controlSpeed(() -> m_driverController.getRawAxis(1)));
    m_driverController.leftTrigger().whileTrue(
      // new WaitCommand(1.0)
      //.andThen
      (Servo.turnServoCounterClockwise())
    );
    m_driverController.rightTrigger().whileTrue(
      // new WaitCommand(1.0)
      //.andThen
      (Servo.turnServoClockwise())
    );

  }

  /**
   * Sets deadband values.
   * 
   * @param input // input by the axis (number between -1 and 1)
   * @param upper // positive deadband range
   * @param lower // negative deadband range
   * @return
   */
  static double deadband (double input, double upper, double lower) {
    if (input < 0) {
      return -(input - lower) / -1 - lower;
    } else if (input > 0) {
      return input + upper / 1 - upper;
    }
    return 0;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);

    //return m_exampleSubsystem.arcadeDrive();

    System.out.println("Option selected");

    return m_chooser.getSelected();
    
  }
}

  

