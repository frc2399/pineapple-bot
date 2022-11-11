package frc.robot;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants.DriveConstants;
import frc.robot.SimGyro;

import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;


/**
 here's the drive train class that operates the drive train
 *
 */
public class DriveTrain extends SubsystemBase {

    public static WPI_VictorSPX leftFrontMotorController;
    private static WPI_VictorSPX rightFrontMotorController;
    private static WPI_VictorSPX leftBackMotorController;
    private static WPI_VictorSPX rightBackMotorController;
    public static PIDController turnController;
    public final double kP = 0.06;
    public final double kPSim = 0.5;
    static final double kI = 0;
    static final double kD = 0;
    static final double kF = 0;
    public static double outputSpeed;

    
    //static double kToleranceDegrees = 2.0f;
    /**
    *
    */

    private DifferentialDriveOdometry odometry;
    private DifferentialDrivetrainSim driveSim;
    private SimVisionSystem simVision;
    public SimEncoder leftEncoderSim;
    public SimEncoder rightEncoderSim;
    public SimGyro gyroSim;
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
        leftFrontMotorController.setInverted(true);
        rightFrontMotorController.setInverted(false);


        //sets motor controllers following leaders
        leftBackMotorController.follow(leftFrontMotorController);
        rightBackMotorController.follow(rightFrontMotorController);

        turnController = new PIDController(kP, kI, kD);
        turnController.enableContinuousInput(-180.0f, 180.0f);



        // this code is instantiating the simulated sensors and actuators when the robot is in simulation
       
            // Ethan is suspicious and thinks we need to re-enable this but it doesn't matter

            if (RobotBase.isSimulation())
            {
                leftEncoderSim = new SimEncoder("Left Drive");
                rightEncoderSim = new SimEncoder("Right Drive");
                gyroSim = new SimGyro("NavX");
                odometry = new DifferentialDriveOdometry(gyroSim.getAngle(), new Pose2d(9, 6.5, new Rotation2d(Math.PI/2)));

                driveSim = new DifferentialDrivetrainSim(
                    DCMotor.getNEO(3), 
                    8, 
                    6, 
                    Units.lbsToKilograms(140), 
                    Units.inchesToMeters(2.1), 
                    Units.inchesToMeters(27.811), 
                    VecBuilder.fill(0, 0, 0, 0, 0, 0, 0));

                    simVision = new SimVisionSystem(" Limelight", 75.0, 15.0, new Transform2d(), 0.85, 20, 640, 480, 10);
                    double targetWidth= Units.inchesToMeters(41.3)-Units.inchesToMeters(6.7);
                    double targetHeight = Units.inchesToMeters(98.19)-Units.inchesToMeters(81.19);
                    double tgtXPos = Units.feetToMeters(54);
                    double tgtyPos= Units.feetToMeters(27/2)- Units.inchesToMeters(43.75) -  Units.inchesToMeters(48/2);
                    Pose2d farTargetPose = new Pose2d(new Translation2d( tgtXPos, tgtyPos), new Rotation2d(0));
                    double tgtyPos1= Units.feetToMeters(27/2) + Units.inchesToMeters(43.75) +  Units.inchesToMeters(48/2);
                    Pose2d farTargetPose1 = new Pose2d(new Translation2d( tgtXPos, tgtyPos1), new Rotation2d(0));
                    double TARGET_HEIGHT_METERS = Units.inchesToMeters(81.19);
            
                    field = new Field2d();

                    field.setRobotPose(new Pose2d(9, 6.5, new Rotation2d(Math.PI/2)));
                    simVision.addSimVisionTarget(new SimVisionTarget(farTargetPose, TARGET_HEIGHT_METERS, targetWidth, targetHeight));
                    simVision.addSimVisionTarget(new SimVisionTarget(farTargetPose1, TARGET_HEIGHT_METERS, targetWidth, targetHeight));

                    FieldObject2d target = field.getObject("target 0");
                    FieldObject2d target1 = field.getObject("target 1");
                    target.setPose(farTargetPose);
                    target1.setPose(farTargetPose1);
                    SmartDashboard.putData("Field", field);
            }
        }

    

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
    }



    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
        // Set the inputs to the system. Note that we need to convert
        // the [-1, 1] PWM signal to voltage by multiplying it by the
        // robot controller voltage.



        odometry.update(
            new Rotation2d(-gyroSim.getAngle().getRadians()), 
            leftEncoderSim.getDistance(), 
            rightEncoderSim.getDistance());
       
            driveSim.setInputs(
                leftFrontMotorController.get() * RobotController.getInputVoltage(), 
                rightFrontMotorController.get() * RobotController.getInputVoltage());

                driveSim.update(0.02);

                leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
                leftEncoderSim.setSpeed(driveSim.getLeftVelocityMetersPerSecond());
                rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
                rightEncoderSim.setSpeed(driveSim.getRightVelocityMetersPerSecond());

                gyroSim.setAngle(new Rotation2d(-driveSim.getHeading().getRadians()));
                simVision.processFrame(odometry.getPoseMeters());
                
        
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

        leftFrontMotorController.set(-leftSpeed);
        rightFrontMotorController.set(-rightSpeed);
        leftBackMotorController.set(leftSpeed);
       

        
    }

    public static void teleopInit()
    {
        leftFrontMotorController.setNeutralMode(NeutralMode.Brake);
        rightFrontMotorController.setNeutralMode(NeutralMode.Brake);        
        leftBackMotorController.setNeutralMode(NeutralMode.Brake);
        rightBackMotorController.setNeutralMode(NeutralMode.Brake);
    }



}
