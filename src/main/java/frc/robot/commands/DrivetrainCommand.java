// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DrivetrainCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrainSubsystem m_drivetrain;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DrivetrainCommand (DriveTrainSubsystem subsystem) {
    m_drivetrain = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting joystick command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // when forwardSpeed is getleftX and turningSpeed is getrightX, rightX turns even 
    // if leftX isnt in use

    // when forwardSpeed is leftY and turningspeed is rightX, neither of them work
    // when forwardSpeed is getRawAxis(1) and turning speed is rightX, neither of them work

    double forwardSpeed = RobotContainer.m_driverController.getLeftX();
    double turningSpeed = RobotContainer.m_driverController.getRightX();
    m_drivetrain.arcadeDrive(forwardSpeed, turningSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
}
