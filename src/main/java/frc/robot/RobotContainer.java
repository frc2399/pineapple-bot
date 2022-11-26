package frc.robot;

//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    private static MoveForwardForTime moveForwardForTime= new MoveForwardForTime(1, 0.3, 0.3, m_driveTrain);
    private static SequentialCommandGroup dance = new SequentialCommandGroup(
        new MoveForwardForTime(3, 0.4, 0.4, m_driveTrain), 
        new MoveForwardForTime(3, -0.4, -0.4, m_driveTrain),
        new MoveForwardForTime(0.6, -0.6, 0.6, m_driveTrain),
        new MoveForwardForTime(3, -0.4, 1, m_driveTrain), // Spin right//
        new MoveForwardForTime(1, -0.7, -0.7, m_driveTrain), //move backwqards//
        new MoveForwardForTime(2, 0.4, 0.4, m_driveTrain), //move forward//
        new MoveForwardForTime(2, 0.4, -0.4, m_driveTrain), //
        new MoveForwardForTime(0.5, 0.7, -0.4, m_driveTrain), //
        new MoveForwardForTime(1.2, 0.58, -0.2, m_driveTrain),//
        new MoveForwardForTime(0.4, 1.6, -0.4, m_driveTrain), //
        new MoveForwardForTime(0.8, 0.6, -0.6, m_driveTrain) //
    );



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

       
      
        SmartDashboard.putData(pdp);



        SmartDashboard.putNumber("drive slew", XboxConstants.DRIVE_SLEW_RATE);
        SmartDashboard.putNumber("turn slew", XboxConstants.TURN_SLEW_RATE);
        

       
        configureButtonBindings();

        
        m_driveTrain.setDefaultCommand(
            new ArcadeDriveCmd(m_driveTrain,
                () -> -driveTurnControls.getDrive(),
                () -> driveTurnControls.getTurn()));


       
        

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
        //new JoystickButton(XBOX, XboxMappingToJoystick.A_BUTTON).whenPressed(moveForwardForTime);
        //new JoystickButton(XBOX, XboxMappingToJoystick.X_BUTTON).whenPressed(dance);

        // new JoystickButton(XBOX, XboxConstants.TURN_RIGHT).whenPressed(m_turnRight);
    
        // Intake
        // new JoystickButton(JOYSTICK, JoystickConstants.INTAKE_BACK).whileHeld(intakeBackCmd);
        

        

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
