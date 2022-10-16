package frc.robot;

// import frc.robot.commands.*;
// import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



//import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

//import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/**
 *
 */
public class DriveTrain extends SubsystemBase {

    public static WPI_VictorSPX leftFrontMotorController;
    private static WPI_VictorSPX rightFrontMotorController;
    private static WPI_VictorSPX leftBackMotorController;
    private static WPI_VictorSPX rightBackMotorController; 
    public static double outputSpeed;

    /**
    *
    */
    final ShuffleboardTab tab = Shuffleboard.getTab("Motor Diag");

    public DriveTrain() {

        leftFrontMotorController = new WPI_VictorSPX(DriveConstants.LEFT_FRONT_MOTOR_ID);
        rightFrontMotorController = new WPI_VictorSPX(DriveConstants.RIGHT_FRONT_MOTOR_ID);
        leftBackMotorController = new WPI_VictorSPX(DriveConstants.LEFT_BACK_MOTOR_ID);
        rightBackMotorController = new WPI_VictorSPX(DriveConstants.RIGHT_BACK_MOTOR_ID);

        // Make wheels go in same direction
        leftFrontMotorController.setInverted(true);
        rightFrontMotorController.setInverted(false);

        //sets motor controllers following leaders
        //leftBackMotorController.follow(leftFrontMotorController);
        rightBackMotorController.follow(rightFrontMotorController);

        }

    @Override
    public void periodic() {

    }



    @Override
    public void simulationPeriodic() {

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setMotors(double leftSpeed, double rightSpeed) {

        leftFrontMotorController.set(-leftSpeed);
        rightFrontMotorController.set(-rightSpeed);
        leftBackMotorController.set(leftSpeed);
        System.out.println(leftSpeed);
        System.out.println(rightSpeed);
    }

    public static void teleopInit()
    {
        leftFrontMotorController.setNeutralMode(NeutralMode.Brake);
        rightFrontMotorController.setNeutralMode(NeutralMode.Brake);        
        leftBackMotorController.setNeutralMode(NeutralMode.Brake);
        rightBackMotorController.setNeutralMode(NeutralMode.Brake);
    }
}
