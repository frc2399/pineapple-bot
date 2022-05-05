package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.*;
import frc.robot.Constants.XboxConstants;
// import frc.robot.commands.robot.PointAndShoot;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */

public class RobotContainer {

    // The robot's subsystems
    public final static DriveTrain m_driveTrain = new DriveTrain();


    // Joysticks
    public static Joystick JOYSTICK = new Joystick(JoystickConstants.JOYSTICK_PORT);
    public static Joystick XBOX = new Joystick(XboxConstants.XBOX_PORT);

    private DriveTurnControls driveTurnControls = new DriveTurnControls(XBOX);

   
    // Robot
    private static InstantCommand killCommand = new InstantCommand(() -> CommandScheduler.getInstance().cancelAll());




    // TODO: Change speed


        


    // private static Command m_pointAndShoot = new PointAndShoot(m_driveTrain, m_shooter, m_indexer);


    public static NetworkTableEntry a_value = Shuffleboard.getTab("Params")
        .addPersistent("Stick Sensitivity", 0.0).getEntry();

    // A chooser for autonomous commands
    static SendableChooser < Command > m_chooser = new SendableChooser < > ();
    public final static ComplexWidget autonChooser = Shuffleboard.getTab("Driver")
    .add("Choose Auton", m_chooser).withWidget(BuiltInWidgets.kSplitButtonChooser).withPosition(0, 4).withSize(9, 1);

    PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);

    // ALWAYS put this last!!!!
    private static RobotContainer m_robotContainer = new RobotContainer();

    private RobotContainer() {

        // PortForwarder.add(5800, "photovision.local", 5800);

        PortForwarder.add(5800, "10.23.99.11", 5800);
        PortForwarder.add(5801, "10.23.99.11", 5801);
        PortForwarder.add(5802, "10.23.99.11", 5802);
        PortForwarder.add(5803, "10.23.99.11", 5803);
        PortForwarder.add(5804, "10.23.99.11", 5804);
        PortForwarder.add(5805, "10.23.99.11", 5805);
        
        // camera not in simulator to make it not crash
        if (RobotBase.isReal())
        {
            CameraServer.startAutomaticCapture();
        }


        // Smart Dashboard
        // Smartdashboard Subsystems
        // SmartDashboard.putData(m_driveTrain);
        // SmartDashboard.putData(m_intake);
        // SmartDashboard.putData(m_shooter);
        // SmartDashboard.putData(m_indexer);

        // Shuffleboard.getTab("SmartDashboard").add("Follow Target", new
        // FollowTarget(m_driveTrain));
        // Shuffleboard.getTab("SmartDashboard").add("Buttery Follow Target", new
        // ButterySmoothFollowTarget(m_driveTrain));

        // //Shuffleboard.getTab("Robot").add("Index and Shoot", new
        // SequentialCommandGroup(
        // new InstantCommand(
        // () -> m_shooter.setSpeedWithPID(ShooterConstants.TOP_SETPOINT,
        // ShooterConstants.BOTTOM_SETPOINT),
        // m_shooter),
        // new WaitUntilCommand(() -> m_shooter.correctSpeed()),
        // new IndexerCmdForGivenTime(m_indexer, 0.5, 2)));

     



        SmartDashboard.putNumber("drive slew", XboxConstants.DRIVE_SLEW_RATE);
        SmartDashboard.putNumber("turn slew", XboxConstants.TURN_SLEW_RATE);

        // SmartDashboard.putNumber("a value", XboxConstants.JOYSTICK_SENSITIVITY);

        // Changing the "a" value on shuffleboard to alter joystick drive sensitivity
        // Shuffleboard.getTab("Drive")
        // .add("a value", 1)
        // .withWidget(BuiltInWidgets.kNumberSlider)
        // .withProperties(Map.of("min", 0, "max", 1))
        // .getEntry();

        // SmartDashboard.putNumber("Joystick Sensitivity",
        // XboxConstants.JOYSTICK_SENSITIVITY);

        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // m_driveTrain.setDefaultCommand(
        // new ArcadeDriveCmd(m_driveTrain,
        // () -> -XBOX.getRawAxis(XboxConstants.ARCADE_DRIVE_SPEED_AXIS),
        // () -> XBOX.getRawAxis(XboxConstants.ARCADE_DRIVE_TURN_AXIS)));
        m_driveTrain.setDefaultCommand(
            new ArcadeDriveCmd(m_driveTrain,
                () -> -driveTurnControls.getDrive(),
                () -> driveTurnControls.getTurn()));


       
        // Shuffleboard.getTab("Robot").add("Index and Shoot", new
        // SequentialCommandGroup(
        // new InstantCommand(
        // () -> m_shooter.setSpeedWithPID(ShooterConstants.TOP_SETPOINT,
        // ShooterConstants.BOTTOM_SETPOINT),
        // m_shooter),
        // new WaitUntilCommand(() -> m_shooter.correctSpeed()),
        // new IndexerCmdForGivenTime(m_indexer, 0.5, 2)));

    }

    public static RobotContainer getInstance() {
        return m_robotContainer;
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */

    private void configureButtonBindings() {
        // Robot
        new JoystickButton(XBOX, XboxMappingToJoystick.Y_BUTTON).whenPressed(killCommand);



        // new JoystickButton(XBOX, XboxConstants.TURN_RIGHT).whenPressed(m_turnRight);
        // new JoystickButton(XBOX, XboxConstants.TURN_LEFT).whenPressed(m_turnLeft);
        // new JoystickButton(XBOX, XboxConstants.TURN_180).whenPressed(m_turn180);

        // Intake
        // new JoystickButton(JOYSTICK, JoystickConstants.INTAKE_BACK).whileHeld(intakeBackCmd);
        // Sets the intake command to the left trigger
        
        

        // new JoystickButton(XBOX, XboxMappingToJoystick.LEFT_STICK_PUSH).whenPressed(collectBall);
        // new JoystickButton(XBOX, XboxMappingToJoystick.RIGHT_STICK_PUSH).whenPressed(noCollectBall);

        //new JoystickButton(XBOX, XboxMappingToJoystick.A_BUTTON).whenPressed(pointAndShootCmd);

        // new JoystickButton(JOYSTICK, JoystickConstants.INTAKE_ARM_EXTEND).whenPressed(extendIntakeArm);
        // new JoystickButton(JOYSTICK, JoystickConstants.INTAKE_ARM_RETRACT).whenPressed(retractIntakeArm);

       
        // TODO: Need to create these commands
        // new JoystickButton(XBOX, XboxConstants.INDEXER_AND_SHOOT).whileHeld();
        // new JoystickButton(XBOX, XboxConstants.POINT_AND_SHOOT).whenPressed(m_pointAndShoot);

        //turning
        // new JoystickButton(XBOX, XboxConstants.TURN_RIGHT_90_CCW).whenPressed(m_turnRight);
        // new JoystickButton(XBOX, XboxConstants.TURN_RIGHT_90_CW).whenPressed(m_turnLeft);
        // new JoystickButton(XBOX, XboxConstants.TURN_180).whenPressed(m_turn180);


        

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */

    public Command getAutonomousCommand() {
        // The selected command will be run in autonomous
        System.out.println("Autonomous command!" + m_chooser.getSelected());
        return m_chooser.getSelected();
    }

}
