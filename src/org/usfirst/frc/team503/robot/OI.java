package org.usfirst.frc.team503.robot;

import org.usfirst.frc.team503.robot.subsystems.CyberShooterSubsystem.Pressure;

import cyber_commands.CyberPressurizeCommand;
import cyber_commands.CyberShootCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);
    
    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.
    
    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:
    
    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());
    
    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());
    
    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());
	
	private static Joystick joystick = new Joystick(0);
	
	private static JoystickButton shootButton = new JoystickButton(joystick, 1);
	private static JoystickButton pressureButton = new JoystickButton(joystick, 2);
	private static JoystickButton safety1 = new JoystickButton(joystick, 5);
	private static JoystickButton safety2 = new JoystickButton(joystick, 6);
	
	public static void initialize(){
		shootButton.whenPressed(new CyberShootCommand());
		//pressureButton.whenPressed(new CyberPressurizeCommand(Pressure.MID));
	}
	
	public static double getLeftYValue(){
		return joystick.getRawAxis(1);
	}
	public static double getLeftXValue(){
		return joystick.getRawAxis(0);
	}
	public static double getRightYValue(){
		return joystick.getRawAxis(5);
	}
	public static double getRightXValue(){
		return joystick.getRawAxis(4);
	}
	
	public static boolean getSafety1(){
		return safety1.get();
	}
	public static boolean getSafety2(){
		return safety2.get();
	}
	
}

