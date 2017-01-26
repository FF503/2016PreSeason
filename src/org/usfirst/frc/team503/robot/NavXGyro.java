package org.usfirst.frc.team503.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

public class NavXGyro{  
    // Abstracted methods for Nav X Gyro
    private static AHRS ahrs;
  

    public NavXGyro() {
    	try { 
    		ahrs = new AHRS(SPI.Port.kMXP); 
    	} 
    	catch (RuntimeException ex ) {
    		DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    	}  	
    }

    public void reset() {
    	ahrs.reset();
    }
    public double getAngle() {
    	return ahrs.getAngle();
    }
    public double getHeading() {
    	return ahrs.getCompassHeading();
    }
}

