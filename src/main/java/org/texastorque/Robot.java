// skeleton of Robot.java
package org.texastorque;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;

import java.lang.Thread;

import javax.swing.tree.AbstractLayoutCache.NodeDimensions;

import org.texastorque.torquelib.base.TorqueIterative;
import org.texastorque.torquelib.component.TorqueEncoder;
import org.texastorque.torquelib.component.TorqueMotor;
import org.texastorque.torquelib.controlLoop.ScheduledPID;
import org.texastorque.torquelib.util.GenericController;
import org.texastorque.util.TorqueTimer;
import org.texastorque.util.Vector;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Talon;

// change the rotation change of direction so that its not just adding or subtracting

// http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/Phoenix-latest.json

public class Robot extends TorqueIterative {

  	GenericController driver = new GenericController(0, 0.1);
  
	private AHRS NX_gyro;
	private double yaw = 0;
	  
	private ScheduledPID rotationalPID0; // MAKE THIS FINAL IN THE ACTUAL THING
 	private double rotSpeed0 = 0;
 	private double currentPos0 = 0;
	private double setpoint0 = 0;
	private double prevSetpoint0 = 0;
 
	private ScheduledPID rotationalPID1; // MAKE THIS FINAL IN THE ACTUAL THING
 	private double rotSpeed1 = 0;
 	private double currentPos1 = 0;
	private double setpoint1 = 0;
	private double prevSetpoint1 = 0;

	private ScheduledPID rotationalPID2; // MAKE THIS FINAL IN THE ACTUAL THING
 	private double rotSpeed2 = 0;
 	private double currentPos2 = 0;
	private double setpoint2 = 0;
	private double prevSetpoint2 = 0;

	private ScheduledPID rotationalPID3; // MAKE THIS FINAL IN THE ACTUAL THING
 	private double rotSpeed3 = 0;
 	private double currentPos3 = 0;
	private double setpoint3 = 0;
	private double prevSetpoint3 = 0;

	private TorqueMotor rotMot0;
	private CANSparkMax transMot0;
 	private TorqueMotor rotMot1;
	private CANSparkMax transMot1;
	private TorqueMotor rotMot2;
	private CANSparkMax transMot2;
	private TorqueMotor rotMot3;
	private CANSparkMax transMot3;

	private TorqueEncoder DB_rot0;
	private CANEncoder DB_trans0;
	private TorqueEncoder DB_rot1;
	private CANEncoder DB_trans1;
	private TorqueEncoder DB_rot2;
	private CANEncoder DB_trans2;
	private TorqueEncoder DB_rot3;
	private CANEncoder DB_trans3;

	private double encoderRotAngle0 = 0;
	private double encoderRotAngle1 = 0;
	private double encoderRotAngle2 = 0;
	private double encoderRotAngle3 = 0;

	private double transX = 0;
	private double transY = 0;
	private double transMag = 0;

	private double transTheta0 = 0;
	private double transTheta1 = 0;
	private double transTheta2 = 0;
	private double transTheta3 = 0;

	private double resultMag0 = 0;
	private double resultMag1 = 0;
	private double resultMag2 = 0;
	private double resultMag3 = 0;
	private double rotMag = 0;
	
	private double rotMagSplit;

	private double dtheta0 = 0;
	private double dtheta1 = 0;
	private double dtheta2 = 0;
	private double dtheta3 = 0;

	private Vector translation;
	private Vector rotation;

	private Vector vector0;
	private Vector vector1;
	private Vector vector2;
	private Vector vector3;

	public void robotInit() {

		// module 0 - rotMot 0 - trans 1 - encoder 0,1
		// module 1 - rotMot 1 - trans 2 - encoder 2,3
		// module 2 - rotMot 2 - trans 3 - encoder 4,5
		// module 3 - rotMot 3 - trans 4 - encoder 6,7
		
		rotMot0 = new TorqueMotor(new VictorSP(0), false);
		transMot0 = new CANSparkMax(1,MotorType.kBrushless);
		rotMot1 = new TorqueMotor(new VictorSP(1), true);
		transMot1 = new CANSparkMax(2, MotorType.kBrushless);
		rotMot2 = new TorqueMotor(new VictorSP(2), false);
		transMot2 = new CANSparkMax(3, MotorType.kBrushless);
		rotMot3 = new TorqueMotor(new VictorSP(3), false);
		transMot3 = new CANSparkMax(4, MotorType.kBrushless);

		DB_trans0 = transMot0.getEncoder(EncoderType.kHallSensor, 4096);
		DB_rot0 = new TorqueEncoder(0,1,true,EncodingType.k4X);
		DB_trans1 = transMot1.getEncoder(EncoderType.kHallSensor, 4096);
		DB_rot1 = new TorqueEncoder(2,3,true,EncodingType.k4X);
		DB_trans2 = transMot2.getEncoder(EncoderType.kHallSensor, 4096);
		DB_rot2 = new TorqueEncoder(4,5,true,EncodingType.k4X);
		DB_trans3 = transMot3.getEncoder(EncoderType.kHallSensor, 4096);
		DB_rot3 = new TorqueEncoder(6,7,true,EncodingType.k4X);

		NX_gyro = new AHRS(SPI.Port.kMXP);

		// initialize reset
		DB_rot0.reset();
		DB_rot1.reset();
		DB_rot2.reset();
		DB_rot3.reset();
		
		rotMot0.set(0);
		transMot0.set(0);
		rotMot1.set(0);
		transMot1.set(0);
		rotMot2.set(0);
		transMot2.set(0);
		rotMot3.set(0);
		transMot3.set(0);
		
		
		NX_gyro.reset();
		NX_gyro.zeroYaw();
		System.out.println(NX_gyro.isCalibrating() + "robotInit");
		try{
			Thread.sleep(2000);
		} catch(InterruptedException e) {

		}
		

		this.rotationalPID0 = new ScheduledPID.Builder(setpoint0, -.8, .8, 1)
			.setPGains(0.02)
			// .setIGains(0)
			// .setDGains(0)
			.build();
		
		this.rotationalPID1 = new ScheduledPID.Builder(setpoint1, -.8, .8, 1)
			.setPGains(0.02)
			// .setIGains(0)
			// .setDGains(0)
			.build();
		
		this.rotationalPID2 = new ScheduledPID.Builder(setpoint2, -.8, .8, 1)
		.setPGains(0.02)
		// .setIGains(0)
		// .setDGains(0)
		.build();

		this.rotationalPID3 = new ScheduledPID.Builder(setpoint3, -.8, .8, 1)
		.setPGains(0.02)
		// .setIGains(0)
		// .setDGains(0)
		.build();
	} // robot is enabled

	private void initSubsystems() {
		rotMot0.set(0);
		transMot0.set(0);
		rotMot1.set(0);
		transMot1.set(0);
		rotMot2.set(0);
		transMot2.set(0);
		rotMot3.set(0);
		transMot3.set(0);
	} // initialize subsystems 

	@Override
	public void autoInit() {
		rotMot0.set(0);
		transMot0.set(0);
		rotMot1.set(0);
		transMot1.set(0);
		rotMot2.set(0);
		transMot2.set(0);
		rotMot3.set(0);
		transMot3.set(0);
	} // auton start
	
	@Override
	public void teleopInit() {
		// talonTest.set(0);
		rotMot0.set(0);
		transMot0.set(0);
		rotMot1.set(0);
		transMot1.set(0);
		rotMot2.set(0);
		transMot2.set(0);
		rotMot3.set(0);
		transMot3.set(0);
		
		NX_gyro.reset();
		NX_gyro.zeroYaw();
		System.out.println(NX_gyro.isCalibrating() + "teleopInit");
		try{
			Thread.sleep(2000);
		} catch(InterruptedException e) {
		}
	} // teleop start

	@Override
	public void disabledInit() {
		// rotMot0.set(0);
		// transMot0.set(0);
		// rotMot1.set(0);
		// transMot1.set(0);
		// rotMot2.set(0);
		// transMot2.set(0);
		// rotMot3.set(0);
		// transMot3.set(0);
	} // disabled robot

	@Override
	public void autoContinuous() {
		rotMot0.set(0);
		transMot0.set(0);
		rotMot1.set(0);
		transMot1.set(0);
		rotMot2.set(0);
		transMot2.set(0);
		rotMot3.set(0);
		transMot3.set(0);
	} // run at all times in state auto 

	@Override
	public void teleopContinuous() {
		// talonTest.set(driver.getLeftXAxis()*.3);

		SmartDashboard.putBoolean("Is Calibrating", NX_gyro.isCalibrating());

		// ---- input -----
		transX = driver.getLeftXAxis();
		transY = -driver.getLeftYAxis();

		rotMag = driver.getRightXAxis();
		rotMagSplit = (Math.sqrt(2)/2)*rotMag;

		translation = new Vector(transX, transY);
		rotation = new Vector(rotMagSplit, rotMagSplit);
		// ----- feedback -------
		runEncoders();

		// ----- calculations -----
		translation.yawOffset(yaw);
		
		vector0 = translation.addVectorDirections(rotation, 1, 1);
		resultMag0 = vector0.getMag();
		transTheta0 = toBearing(vector0.getTheta());

		vector1 = translation.addVectorDirections(rotation, 1, -1);
		resultMag1 = vector1.getMag();
		transTheta1 = toBearing(vector1.getTheta());

		vector2 = translation.addVectorDirections(rotation, -1, 1);
		resultMag2 = vector2.getMag();
		transTheta2 = toBearing(vector2.getTheta());

		vector3 = translation.addVectorDirections(rotation, -1, -1);
		resultMag3 = vector3.getMag();
		transTheta3 = toBearing(vector3.getTheta());

		if (resultMag0 > 1 ||  resultMag1 > 1 || resultMag2 > 1 || resultMag3 >1){
			double max = resultMag0;
			if (resultMag1 > max){max = resultMag1;}
			if (resultMag2 > max){max = resultMag2;}
			if (resultMag3 > max){max = resultMag3;}
			resultMag0 /= max;
			resultMag1 /= max;
			resultMag2 /= max;
			resultMag3 /= max;
		} // if there is a translational output of more than one, scale them all back to the maximum equalling one
		
		// plots to the smaller path for travel, and flips direction of wheel spin
		if (needOptimization(encoderRotAngle0, transTheta0)){
			resultMag0 *= -1;
			if (transTheta0 > 0){
				transTheta0 -= 180;
			} else {
				transTheta0 += 180;
			}
		}
		if (needOptimization(encoderRotAngle1, transTheta1)){
			resultMag1 *= -1;
			if (transTheta1 > 0){
				transTheta1 -= 180;
			} else {
				transTheta1 += 180;
			}
		}
		if (needOptimization(encoderRotAngle2, transTheta2)){
			resultMag2 *= -1;
			if (transTheta2 > 0){
				transTheta2 -= 180;
			} else {
				transTheta2 += 180;
			}
		}
		if (needOptimization(encoderRotAngle3, transTheta3)){
			resultMag3 *= -1;
			if (transTheta3 > 0){
				transTheta3 -= 180;
			} else {
				transTheta3 += 180;
			}
		}

		// ------ output --------
		runRotationalPID0();
		runRotationalPID1();
		runRotationalPID2();
		runRotationalPID3();


		rotMot0.set(rotSpeed0);
		transMot0.set(-resultMag0/2);
		rotMot1.set(rotSpeed1);
		transMot1.set(-resultMag1/2);
		rotMot2.set(rotSpeed2);
		transMot2.set(-resultMag2/2);
		rotMot3.set(rotSpeed3);
		transMot3.set(-resultMag3/2);
	} // run at all times in state teleop

  public void runEncoders(){
    DB_rot0.calc();
    encoderRotAngle0 = DB_rot0.get()/8.5; // EXPERIMENTALLY DERIVED!!! NEED TO GET A MORE ACCURATE ONE
	// encoderTransSpeed0 = DB_trans0.getVelocity()*0.225; // NEED TO CHANGE (POTENTIALLY) COULD BE WRONG
	DB_rot1.calc();
    encoderRotAngle1 = DB_rot1.get()/8.5; // EXPERIMENTALLY DERIVED!!! NEED TO GET A MORE ACCURATE ONE
	// encoderTransSpeed1 = DB_trans1.getVelocity()*0.225; // NEED TO CHANGE (POTENTIALLY) COULD BE WRONG
	DB_rot2.calc();
    encoderRotAngle2 = DB_rot2.get()/8.5; // EXPERIMENTALLY DERIVED!!! NEED TO GET A MORE ACCURATE ONE
	// encoderTransSpeed2 = DB_trans2.getVelocity()*0.225; // NEED TO CHANGE (POTENTIALLY) COULD BE WRONG
	DB_rot3.calc();
    encoderRotAngle3 = DB_rot3.get()/8.5; // EXPERIMENTALLY DERIVED!!! NEED TO GET A MORE ACCURATE ONE
	// encoderTransSpeed3 = DB_trans3.getVelocity()*0.225; // NEED TO CHANGE (POTENTIALLY) COULD BE WRONG

	yaw = NX_gyro.getYaw();
  } // calculate encoder turn 

	@Override
	public void disabledContinuous() {
		rotMot0.set(0);
		transMot0.set(0);
		rotMot1.set(0);
		transMot1.set(0);
		rotMot2.set(0);
		transMot2.set(0);
		rotMot3.set(0);
		transMot3.set(0);
	} // run at all times when disabled

	@Override
	public void alwaysContinuous() {
		smartDashboard();
	} // run at all times when robot has power

	// ------- METHODS TO USE IN STUFF ----------
	public double toBearing(double angle) { // to use with input from controller // 1_comment
        double bearing = 0;
		angle = angle % 360;
		if (transX == 0 && transY == 0 && rotMag == 0){
			return 1477;
			// return 0;
		} 
		if ((angle <= 180 && angle >= 0 ) || (angle < 0 && angle >= -90)) {
			bearing = 90 - angle;
		} else {
			bearing = 90 - angle -360;
		}
        return bearing;
	} // change value to a bearing

	public boolean needOptimization(double encoderRotAngle, double transTheta){
		double dtheta = 0;
		if (encoderRotAngle >= -180 && encoderRotAngle < -90){ // Quadrant 3
			dtheta = encoderRotAngle + 90;
			if (-dtheta < transTheta && transTheta < (180 - dtheta)){
				return true;
			}
			return false;
		} else if (encoderRotAngle >= -90 && encoderRotAngle < 0) { // Quadrant 2
			dtheta = -encoderRotAngle;
			if (((90 - dtheta) < transTheta && transTheta < 180) || ((-90-dtheta) > dtheta && dtheta > -180)) {
				return true;
			}
			return false;
		} else if (encoderRotAngle >= 0 && encoderRotAngle < 90){ // Quadrant 1
			dtheta = encoderRotAngle;
			if ((-180 < transTheta && transTheta < (-90+dtheta)) || ((90+dtheta) < transTheta && transTheta < 180) ) {
				return true;
			}
			return false;
		} else if (encoderRotAngle >= 90 && encoderRotAngle <=180) {// Quadrant 4
			dtheta = 180 - dtheta;
			if (((-90-dtheta) < transTheta && transTheta < 0) || (0 < transTheta && transTheta < (90-dtheta))){
				return true;
			}
			return false;
		}
		return false;
	}

	public void runRotationalPID0(){
		if (transTheta0 <= 180 && transTheta0 >= -180){
			setpoint0 = transTheta0;
			SmartDashboard.putNumber("setpoint0", setpoint0);
		}
		currentPos0 = encoderRotAngle0;
		if (setpoint0 != prevSetpoint0){
			SmartDashboard.putNumber("difference0", Math.abs(setpoint0-currentPos0));
			rotationalPID0.changeSetpoint(setpoint0);
			prevSetpoint0 = setpoint0;
		}
		rotSpeed0 = rotationalPID0.calculate(currentPos0);
	}
	
	public void runRotationalPID1(){
		if (transTheta1 <= 180 && transTheta1 >= -180){
			setpoint1 = transTheta1;
			SmartDashboard.putNumber("setpoint1", setpoint1);
		}
		currentPos1 = encoderRotAngle1;
		if (setpoint1 != prevSetpoint1){
			SmartDashboard.putNumber("difference1", Math.abs(setpoint1-currentPos1));
			rotationalPID1.changeSetpoint(setpoint1);
			prevSetpoint1 = setpoint1;
		}
		rotSpeed1 = rotationalPID1.calculate(currentPos1);
	}
	
	public void runRotationalPID2(){
		if (transTheta2 <= 180 && transTheta2 >= -180){
			setpoint2 = transTheta2;
			SmartDashboard.putNumber("setpoint2", setpoint2);
		}
		currentPos2 = encoderRotAngle2;
		if (setpoint2 != prevSetpoint2){
			SmartDashboard.putNumber("difference2", Math.abs(setpoint2-currentPos2));
			rotationalPID2.changeSetpoint(setpoint2);
			prevSetpoint2 = setpoint2;
		}
		rotSpeed2 = rotationalPID2.calculate(currentPos2);
	}
	
	public void runRotationalPID3(){
		if (transTheta3 <= 180 && transTheta3 >= -180){
			setpoint3 = transTheta3;
			SmartDashboard.putNumber("setpoint3", setpoint3);
		}
		currentPos3 = encoderRotAngle3;
		if (setpoint3 != prevSetpoint3){
			SmartDashboard.putNumber("difference3", Math.abs(setpoint3-currentPos3));
			rotationalPID3.changeSetpoint(setpoint3);
			prevSetpoint3 = setpoint3;
		}
		rotSpeed3 = rotationalPID3.calculate(currentPos3);
	}

	public void smartDashboard(){
		SmartDashboard.putNumber("Yaw", yaw);
	} // put stuff to smart dashboard
} // Robot