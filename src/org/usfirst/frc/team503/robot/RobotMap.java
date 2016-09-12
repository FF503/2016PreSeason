package org.usfirst.frc.team503.robot;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.
    // public static int leftMotor = 1;
    // public static int rightMotor = 2;
    
    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static int rangefinderPort = 1;
    // public static int rangefinderModule = 1;
	public static class Cyber{
		public static int driveCounter = 0;
		public static double driveValue = 0;
		public static final boolean DRIVE_SENSITIVITY = false;
		
		public static double currentPressure = 0;
		public static final double PRESSURE_TOLERANCE = 50;	
		public static final double PRESSURE_MOTOR_SPEED = .5;
		
		public static double pressurePIDOutput = 0;
		public static final double PRESSURE_P = 0;
		public static final double PRESSURE_I = 0;
		public static final double PRESSURE_D = 0;
		public static final double PRESSURE_F = 0;
		
		public static double currentAngle = 0;
		public static final double ARM_TOLERANCE = 3;
		
		public static double armPIDOutput = 0;
		public static final double ARM_P = 0;
		public static final double ARM_I = 0;
		public static final double ARM_D = 0;
		public static final double ARM_F = 0;
	}
}