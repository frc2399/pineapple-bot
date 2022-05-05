package frc.robot;

// import frc.robot.commands.*;
// import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



//import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

//import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.util.NavX;
// import edu.wpi.first.wpilibj.simulation.EncoderSim;
// import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

import com.ctre.phoenix.motorcontrol.NeutralMode;
//import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;






/**
 *
 */
public class DriveTrain extends SubsystemBase {

    public static WPI_VictorSPX leftFrontMotorController;
    private static WPI_VictorSPX rightFrontMotorController;
    private static WPI_VictorSPX leftBackMotorController;
    private static WPI_VictorSPX rightBackMotorController;

   
    public static PIDController turnController;
    //public static DifferentialDrive myRobot;
    //double rotateToAngleRate;


    public final double kP = 0.06;
    public final double kPSim = 0.5;
    static final double kI = 0;
    static final double kD = 0;
    static final double kF = 0;

    //private static double currentAngle = 0;
    //private static double targetAngle = 0;
    public static double outputSpeed;

    
    //static double kToleranceDegrees = 2.0f;
    /**
    *
    */

    private DifferentialDriveOdometry odometry;


    private DifferentialDrivetrainSim driveSim;
    private Field2d field = new Field2d();


    final ShuffleboardTab tab = Shuffleboard.getTab("Motor Diag");
    public static final NetworkTableEntry angleErrorTolerance = Shuffleboard.getTab("Params").addPersistent("Angle Err Tol", 2).getEntry();
    public static final NetworkTableEntry distanceErrorTolerance = Shuffleboard.getTab("Params").addPersistent("Distance Err Tol", 5).getEntry();
    public static final NetworkTableEntry robotAngle = Shuffleboard.getTab("Driver").add("Angle of Robot", 0).getEntry();
    public static final NetworkTableEntry angleErrorPValue = Shuffleboard.getTab("Params").add("angle err p", 0.01).getEntry();
    public static final NetworkTableEntry encoderTickLeft = Shuffleboard.getTab("Testing").add("tick left", 0).getEntry();
    public static final NetworkTableEntry encoderTickRight = Shuffleboard.getTab("Testing").add("tick right", 0).getEntry();




    public DriveTrain() {

        leftFrontMotorController = new WPI_VictorSPX(DriveConstants.LEFT_FRONT_MOTOR_ID);
        rightFrontMotorController = new WPI_VictorSPX(DriveConstants.RIGHT_FRONT_MOTOR_ID);
        leftBackMotorController = new WPI_VictorSPX(DriveConstants.LEFT_BACK_MOTOR_ID);
        rightBackMotorController = new WPI_VictorSPX(DriveConstants.RIGHT_BACK_MOTOR_ID);

        // Set motors to coast mode
        //teleopInit();

        // Make wheels go in same direction
        leftFrontMotorController.setInverted(true);
        rightFrontMotorController.setInverted(false);

        //sets motor controllers following leaders
        leftBackMotorController.follow(leftFrontMotorController);
        rightBackMotorController.follow(rightFrontMotorController);

        //myRobot = new DifferentialDrive(leftFrontMotorController, rightFrontMotorControllefr);

        turnController = new PIDController(kP, kI, kD);
        turnController.enableContinuousInput(-180.0f, 180.0f);



        // this code is instantiating the simulated sensors and actuators when the robot is in simulation
       
            field = new Field2d();

            // Ethan is suspicious and thinks we need to re-enable this but it doesn't matter
            SmartDashboard.putData("Field", field);

            field.setRobotPose(new Pose2d(9, 6.5, new Rotation2d(3.14/2)));
        }

    

    

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // currentAngle = ahrs.getAngle();
        // targetAngle = Robot.targetAngle;

        // double error = targetAngle - currentAngle;

        // outputSpeed = kP * error;

        // robotAngle.setDouble(ahrs.getAngle());
        // SmartDashboard.putNumber("Angle", ahrs.getAngle());


// 
        // System.out.println("drive train periodic");

        //SmartDashboard.putNumber("target angle", RobotContainer.m_turnToNAngle.targetAngle);

        // outputSpeed = MathUtil.clamp(outputSpeed, -0.5, 0.5);
        // setMotors(-outputSpeed, outputSpeed);


        // This will get the simulated sensor readings that we set
        // in the previous article while in simulation, but will use
        // real values on the robot itself.
        // finds the position and angle of the robot given gyro and encoders


    }



    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
        // Set the inputs to the system. Note that we need to convert
        // the [-1, 1] PWM signal to voltage by multiplying it by the
        // robot controller voltage.

       
        field.setRobotPose(odometry.getPoseMeters());

       
    

        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        driveSim.update(0.02);

        // Update all of our sensors.
        

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setMotors(double leftSpeed, double rightSpeed) {

        leftFrontMotorController.set(leftSpeed);
        rightFrontMotorController.set(rightSpeed);

        // SmartDashboard.putNumber("outputSpeed", leftSpeed);
    }

    public static void teleopInit()
    {
        leftFrontMotorController.setNeutralMode(NeutralMode.Coast);
        rightFrontMotorController.setNeutralMode(NeutralMode.Coast);        
        leftBackMotorController.setNeutralMode(NeutralMode.Coast);
        rightBackMotorController.setNeutralMode(NeutralMode.Coast);
    }


    // public static void teleopInit()
    // {
    //     leftFrontMotorController.setIdleMode(CANSparkMax.IdleMode.kCoast);
    //     rightFrontMotorController.setIdleMode(CANSparkMax.IdleMode.kCoast);
    //     leftBackMotorController.setIdleMode(CANSparkMax.IdleMode.kCoast);
    //     rightBackMotorController.setIdleMode(CANSparkMax.IdleMode.kCoast);
    // }

    // public static void autonomousInit()
    // {
    //     leftFrontMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
    //     rightFrontMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
    //     leftBackMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
    //     rightBackMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
    // }


}
