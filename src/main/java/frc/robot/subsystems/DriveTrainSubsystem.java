

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.AutoCloseable;

import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase{

    SparkMax leftFrontMotor = new SparkMax(Constants.DrivetrainConstants.leftFrontCANID, MotorType.kBrushless);
    SparkMax leftBackMotor = new SparkMax(Constants.DrivetrainConstants.leftBackCANID, MotorType.kBrushless);
    SparkMax rightFrontMotor = new SparkMax(Constants.DrivetrainConstants.rightFrontCANID, MotorType.kBrushless);
    SparkMax rightBackMotor = new SparkMax(Constants.DrivetrainConstants.rightBackCANID, MotorType.kBrushless);

    RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
    RelativeEncoder rightEncoder = rightFrontMotor.getEncoder();

    // need to create a config class for all 4 motors?
    
    MotorController leftMotors = new MotorController() {
        public void set (double speed) {
            leftFrontMotor.set(speed);
            leftBackMotor.set(speed);
        }

        @Override
        public double get() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'get'");
        }

        @Override
        public void setInverted(boolean isInverted) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'setInverted'");
        }

        @Override
        public boolean getInverted() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'getInverted'");
        }

        @Override
        public void disable() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'disable'");
        }

        @Override
        public void stopMotor() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'stopMotor'");
        }
    };

    MotorController rightMotors = new MotorController() {
        public void set(double speed) {
            rightFrontMotor.set(speed);
            rightBackMotor.set(speed);
        }

        @Override
        public double get() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'get'");
        }

        @Override
        public void setInverted(boolean isInverted) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'setInverted'");
        }

        @Override
        public boolean getInverted() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'getInverted'");
        }

        @Override
        public void disable() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'disable'");
        }

        @Override
        public void stopMotor() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'stopMotor'");
        }
    };
    
    DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
    
    public DriveTrainSubsystem() {

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        // How to set two motors as followers?

        //leftFrontMotor.setInverted(true);
    }

    public void arcadeDrive (double forward, double rotation) {
        differentialDrive.arcadeDrive(forward, rotation);
    }

    public void tankDrive (double left, double right) {
        differentialDrive.tankDrive(left, right);
    } 

    /*public Command tankDrive (double left) {
        differentialDrive.tankDrive(left, right);
    }
    */

    public Command arcadeDrive (DoubleSupplier forwardVal, DoubleSupplier rotationVal) {
        return new FunctionalCommand(

        // INIT
        () -> {
            System.out.println("Starting joystick command for arcade drive");
        },

        // EXECUTE
        () -> {
            // System.out.println("Executing arcade drive command");
            double forwardSpeed = forwardVal.getAsDouble();
            SmartDashboard.putNumber("Forward Speed", forwardSpeed);
            double turningSpeed = rotationVal.getAsDouble();
            SmartDashboard.putNumber("Turning Speed", turningSpeed);
            arcadeDrive(turningSpeed, forwardSpeed);
        },

        interrupted -> {
            // arcadeDrive(0.0, 0.0);
        },

        () -> false,

        this);
    }

    public Command tankDriveMethod (DoubleSupplier leftVal, DoubleSupplier rightVal) {
        return new FunctionalCommand(

        // INIT
        () -> {
            System.out.println("Starting joystick command for tank drive");
        },

        // EXECUTE
        () -> {
            System.out.println("Executing tank drive command");
            double leftSpeed = leftVal.getAsDouble();
            SmartDashboard.putNumber("Left speed", leftSpeed);
            double rightSpeed = rightVal.getAsDouble();
            SmartDashboard.putNumber("Right speed", rightSpeed);
            tankDrive(leftSpeed, rightSpeed);
        },

        interrupted -> {
            // arcadeDrive(0.0, 0.0);
        },

        () -> false,

        this);
    }
}

