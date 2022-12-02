package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.RobotContainer;
// import frc.robot.Constants.JoystickConstants;
// import frc.robot.Constants.XboxConstants;

public class ArcadeDriveCmd extends CommandBase {

    public static final NetworkTableEntry leftSpeed = Shuffleboard.getTab("Motor Diag").add("Left Speed", 0).getEntry();
    public static final NetworkTableEntry rightSpeed = Shuffleboard.getTab("Motor Diag").add("Right Speed", 0).getEntry();

    private final DriveTrain driveSubsystem;
    private final Supplier<Double> speedFunction, turnFunction;

    public ArcadeDriveCmd(DriveTrain driveSubsystem, //
            Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
        this.speedFunction = speedFunction;
        this.turnFunction = turnFunction;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ArcadeDriveCmd started!");
    }

    @Override
    public void execute() {
        double realTimeSpeed;
        double realTimeTurn;

        // // have deadband to prevent joystick drifting
        // if (Math.abs(speedFunction.get()) <= XboxConstants.FORWARD_DEADBAND) {
        //     realTimeSpeed = 0;
        // } 
        // else {
        //     realTimeSpeed = speedFunction.get() * JoystickConstants.FORWARD_JOYSTICK_INVERT;
        // }

        // // have deadband to prevent joystick drifting
        // if (Math.abs(turnFunction.get()) <= XboxConstants.TURN_DEADBAND) {
        //     realTimeTurn = 0;
        // } 
        // else {
        //     realTimeTurn = turnFunction.get() * JoystickConstants.TURN_JOYSTICK_INVERT;
        // }


        // double a = RobotContainer.a_value.getDouble(0.0);

        // // altering driving joystick sensitivity
        // realTimeSpeed = ((1 - a) * realTimeSpeed) + (a * Math.pow(realTimeSpeed, 3));

        realTimeSpeed = speedFunction.get();
        realTimeTurn = turnFunction.get();

        double left = realTimeSpeed + realTimeTurn;
        double right = realTimeSpeed - realTimeTurn;
       
        leftSpeed.setNumber(left);
        rightSpeed.setNumber(right);
       // SmartDashboard.putNumber("left speed", left);
       // SmartDashboard.putNumber("right speed", right);

       SmartDashboard.putNumber("realTimeSpeed", realTimeSpeed);
       SmartDashboard.putNumber("realTimeTurn", realTimeTurn);

        this.driveSubsystem.setMotors(left, right);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ArcadeDriveCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
