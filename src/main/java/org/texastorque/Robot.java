// skeleton of Robot.java
package org.texastorque;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;

import org.texastorque.torquelib.base.TorqueIterative;
import org.texastorque.torquelib.component.TorqueEncoder;
import org.texastorque.torquelib.component.TorqueMotor;
import org.texastorque.torquelib.controlLoop.ScheduledPID;
import org.texastorque.torquelib.util.GenericController;
import org.texastorque.util.VectorUtils;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// change the rotation change of direction so that its not just adding or subtracting

// http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/Phoenix-latest.json

public class Robot extends TorqueIterative {

  	GenericController driver = new GenericController(0, 0.1);
  
	private AHRS NX_gyro;
	private double pitch = 0;
	private double roll = 0;
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
	private double encoderTransSpeed0 = 0;
	private double encoderRotAngle1 = 0;
	private double encoderTransSpeed1 = 0;
	private double encoderRotAngle2 = 0;
	private double encoderTransSpeed2 = 0;
	private double encoderRotAngle3 = 0;
	private double encoderTransSpeed3 = 0;

	private double transX = 0;
	private double transY = 0;
	private double transMag = 0;
	private double transThetaRaw = 0;
	private double transTheta = 0;

	private double transTheta0 = 0;
	private double transTheta1 = 0;
	private double transTheta2 = 0;
	private double transTheta3 = 0;

	private double resultMag = 0;

	private double resultMag0 = 0;
	private double resultMag1 = 0;
	private double resultMag2 = 0;
	private double resultMag3 = 0;

	private double diff0 = 0;
	private double diff1 = 0;
	private double diff2 = 0;
	private double diff3 = 0;

	private double rotMag = 0;

	private double s_c_45;

	private double transSpeed0 = 0;
	private double transSpeed1 = 0;
	private double transSpeed2 = 0;
	private double transSpeed3 = 0;

	private double transRotTempX0 = 0;
	private double transRotTempX1 = 0;
	private double transRotTempX2 = 0;
	private double transRotTempX3 = 0;
	private double transRotTempY0 = 0;
	private double transRotTempY1 = 0;
	private double transRotTempY2 = 0;
	private double transRotTempY3 = 0;

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
		rotMot0.set(0);
		transMot0.set(0);
		rotMot1.set(0);
		transMot1.set(0);
		rotMot2.set(0);
		transMot2.set(0);
		rotMot3.set(0);
		transMot3.set(0);
	} // teleop start

	@Override
	public void disabledInit() {
		rotMot0.set(0);
		transMot0.set(0);
		rotMot1.set(0);
		transMot1.set(0);
		rotMot2.set(0);
		transMot2.set(0);
		rotMot3.set(0);
		transMot3.set(0);
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
		// ---- input -----
		transX = driver.getLeftXAxis();
		transY = -driver.getLeftYAxis();
		// rotMag = Math.abs(driver.getRightXAxis());
		rotMag = driver.getRightXAxis();
		
		// ----- feedback -------
		runEncoders();

		// ----- calculations -----
		// transMag = Math.hypot(transX, transY); // square value // second
		transThetaRaw = Math.toDegrees(Math.atan2(transY, transX));
		double transThetaTemp = toBearing(transThetaRaw);
		// resultMag = VectorUtils.vectorAddition2DMagnitude(transX, transY, Math.abs(rotMag), 0);
		resultMag = Math.hypot(transX, transY);//VectorUtils.vectorAddition2DMagnitude(transX, transY, 0, 0);
		transTheta = transThetaTemp - (yaw);

		s_c_45 = (Math.sin(Math.toRadians(45)));//Math.toDegrees(Math.sin(Math.toRadians(45)));
		SmartDashboard.putNumber("transThetaTemp", transThetaTemp);
		SmartDashboard.putNumber("transTheta", transTheta);

		if (rotMag > 0 && (transY != 0 || transX != 0) && (transThetaTemp >= 0 && transThetaTemp <= 90)){ 
			if (transThetaTemp >= 0 && transThetaTemp < 45){
				// module 0 between 0 and 45 degrees bearing
				transRotTempX0 = transX + rotMag*s_c_45;
				transRotTempY0 = transY + rotMag*s_c_45;
				resultMag0 = Math.hypot(transRotTempX0, transRotTempY0);
				double thetaMid0 = Math.asin((rotMag/resultMag0)*Math.sin(Math.toRadians(135 + transThetaTemp)));
				transTheta0 = toBearing(transTheta + thetaMid0);

				// module 3 between 0 and 45 degrees bearing
				transRotTempX3 = transX - rotMag*s_c_45;
				transRotTempY3 = transY - rotMag*s_c_45;
				resultMag3 = Math.hypot(transRotTempX3, transRotTempY3);
				double thetaMid3 = Math.asin((rotMag/resultMag3)*Math.sin(Math.toRadians(45 - transThetaTemp)));
				transTheta3 = toBearing(transTheta - thetaMid3);
			} // for the first half of the first quadrant
			else if (transThetaTemp > 45 && transThetaTemp <= 90){
				// module 0 between 45 and 90 degrees bearing
				transRotTempX0 = transX + rotMag*s_c_45;
				transRotTempY0 = transY + rotMag*s_c_45;
				resultMag0 = Math.hypot(transRotTempX0, transRotTempY0);
				double thetaMid0 = Math.asin((rotMag/resultMag0)*Math.sin(Math.toRadians(225 - transThetaTemp)));
				transTheta0 = toBearing(transTheta - thetaMid0);

				// module 3 between 45 and 90 degrees bearing
				transRotTempX3 = transX - rotMag*s_c_45;
				transRotTempY3 = transY - rotMag*s_c_45;
				resultMag3 = Math.hypot(transRotTempX3, transRotTempY3);
				double thetaMid3 = Math.asin((rotMag/resultMag3)*Math.sin(Math.toRadians(transThetaTemp - 45)));
				transTheta3 = toBearing(transTheta + thetaMid3);
			} // for the second half of the first quadrant
			else if (transThetaTemp == 45){
				// module 0 at 45 degrees bearing
				transRotTempX0 = transX + rotMag*s_c_45;
				transRotTempY0 = transY + rotMag*s_c_45;
				resultMag0 = Math.hypot(transRotTempX0, transRotTempY0);
				transTheta0 = transTheta;

				// module 3 at 45 degrees bearing
				transRotTempX3 = transX - rotMag*s_c_45;
				transRotTempY3 = transY - rotMag*s_c_45;
				resultMag3 = Math.hypot(transRotTempX3, transRotTempY3);
				transTheta3 = transTheta;
			} // for the middle of the first quadrant

			transRotTempX1 = transX + rotMag*s_c_45;
			transRotTempY1 = transY - rotMag*s_c_45;
			resultMag1 = Math.hypot(transRotTempX1, transRotTempY1);
			double thetaMid1 = Math.asin((rotMag/resultMag1)*Math.sin(Math.toRadians(45 + transThetaTemp)));
			transTheta1 = toBearing(transTheta + thetaMid1);

			transRotTempX2 = transX - rotMag*s_c_45;
			transRotTempY2 = transY + rotMag*s_c_45;
			resultMag2 = Math.hypot(transRotTempX2, transRotTempY2);
			double thetaMid2 = Math.asin((rotMag/resultMag2)*Math.sin(Math.toRadians(135-transThetaTemp)));
			transTheta2 = toBearing(transTheta - thetaMid2);
			
			SmartDashboard.putNumber("transTheta0", transTheta0);
			SmartDashboard.putNumber("transTheta1", transTheta1);
			SmartDashboard.putNumber("transTheta2", transTheta2);
			SmartDashboard.putNumber("transTheta3", transTheta3);
			SmartDashboard.putNumber("resultMag0", resultMag0);
			SmartDashboard.putNumber("resultMag1", resultMag1);
			SmartDashboard.putNumber("resultMag2", resultMag2);
			SmartDashboard.putNumber("resultMag3", resultMag3);
		} else if (rotMag > 0 && transX == 0 && transY == 0){
			transRotTempX0 = transX + rotMag*Math.cos(Math.PI/4);
			transRotTempY0 = transY + rotMag*Math.sin(Math.PI/4);
			transTheta0 = Math.toDegrees(Math.atan2(transRotTempY0, transRotTempX0));
			resultMag0 = Math.hypot(transRotTempX0, transRotTempY0);

			transRotTempX1 = transX + rotMag*Math.cos(Math.PI/4);
			transRotTempY1 = transY - rotMag*Math.sin(Math.PI/4);
			transTheta1 = Math.toDegrees(Math.atan2(transRotTempY1, transRotTempX1));
			resultMag1 = -Math.hypot(transRotTempX1, transRotTempY1);
			
			transRotTempX2 = transX - rotMag*Math.cos(Math.PI/4);
			transRotTempY2 = transY + rotMag*Math.sin(Math.PI/4);
			transTheta2 = Math.toDegrees(Math.atan2(transRotTempY2, transRotTempX2));
			resultMag2 = -Math.hypot(transRotTempX2, transRotTempY2);

			transRotTempX3 = transX - rotMag*Math.cos(Math.PI/4);
			transRotTempY3 = transY - rotMag*Math.sin(Math.PI/4);
			transTheta3 = Math.toDegrees(Math.atan2(transRotTempY3, transRotTempX3));
			resultMag3 = Math.hypot(transRotTempX3, transRotTempY3);
		} else if (rotMag < 0 && transX == 0 && transY == 0){
			transRotTempX0 = transX - rotMag*Math.cos(Math.PI/4);
			transRotTempY0 = transY - rotMag*Math.sin(Math.PI/4);
			transTheta0 = Math.toDegrees(Math.atan2(transRotTempY0, transRotTempX0));
			resultMag0 = -Math.hypot(transRotTempX0, transRotTempY0);

			transRotTempX1 = transX - rotMag*Math.cos(Math.PI/4);
			transRotTempY1 = transY + rotMag*Math.sin(Math.PI/4);
			transTheta1 = Math.toDegrees(Math.atan2(transRotTempY1, transRotTempX1));
			resultMag1 = Math.hypot(transRotTempX1, transRotTempY1);
			
			transRotTempX2 = transX + rotMag*Math.cos(Math.PI/4);
			transRotTempY2 = transY - rotMag*Math.sin(Math.PI/4);
			transTheta2 = Math.toDegrees(Math.atan2(transRotTempY2, transRotTempX2));
			resultMag2 = Math.hypot(transRotTempX2, transRotTempY2);

			transRotTempX3 = transX + rotMag*Math.cos(Math.PI/4);
			transRotTempY3 = transY + rotMag*Math.sin(Math.PI/4);
			transTheta3 = Math.toDegrees(Math.atan2(transRotTempY3, transRotTempX3));
			resultMag3 = -Math.hypot(transRotTempX3, transRotTempY3);
			
			SmartDashboard.putNumber("resultMag0", resultMag0);
			SmartDashboard.putNumber("resultMag1", resultMag1);
			SmartDashboard.putNumber("resultMag2", resultMag2);
			SmartDashboard.putNumber("resultMag3", resultMag3);
		}
		
		if (rotMag == 0){ 
			transTheta0 = transTheta;
			transTheta1 = transTheta;
			transTheta2 = transTheta;
			transTheta3 = transTheta;
			resultMag0 = resultMag;
			resultMag1 = resultMag;
			resultMag2 = resultMag;
			resultMag3 = resultMag;
		}
		
//START TODAY
		// diff0 = Math.abs(transTheta0 - encoderRotAngle0);
		// diff1 = Math.abs(transTheta1 - encoderRotAngle1);
		// diff2 = Math.abs(transTheta2 - encoderRotAngle2);
		// diff3 = Math.abs(transTheta3 - encoderRotAngle3);

		// if (diff0 > 180){
		// 	diff0 = 360 - diff0;
		// }
		// if (diff1 > 180){
		// 	diff1 = 360 - diff1;
		// }
		// if (diff2 > 180){
		// 	diff2 = 360 - diff2;
		// }
		// if (diff3 > 180){
		// 	diff3 = 360 - diff3;
		// }

		// // today
		// // plots to the smaller path for travel, and flips direction of wheel spin
		// if (diff0 > 90){
		// 	resultMag0 *= -1;
		// 	if (transTheta0 > 0){
		// 		transTheta0 -= 180;
		// 	} else {
		// 		transTheta0 += 180;
		// 	}
		// }
		// if (diff1 > 90){
		// 	resultMag1 *= -1;
		// 	if (transTheta1 > 0){
		// 		transTheta1 -= 180;
		// 	} else {
		// 		transTheta1 += 180;
		// 	}
		// }
		// if (diff2 > 90){
		// 	resultMag2 *= -1;
		// 	if (transTheta2 > 0){
		// 		transTheta2 -= 180;
		// 	} else {
		// 		transTheta2 += 180;
		// 	}
		// }
		// if (diff3 > 90){
		// 	resultMag3 *= -1;
		// 	if (transTheta3 > 0){
		// 		transTheta3 -= 180;
		// 	} else {
		// 		transTheta3 += 180;
		// 	}
		// }
// END TODAY
		// if ((Math.abs(transTheta0-encoderRotAngle0) > 90) && (Math.abs(encoderRotAngle0+transTheta0) > 90)){
		// 	if(encoderRotAngle0 < 0){
		// 		transTheta0+= 180;
		// 	}
		// 	else {
		// 		transTheta0 -= 180;
		// 	}
		// 	resultMag0 *= -1;
		// } // least distance traveled / drive only displacement

		// if ((Math.abs(transTheta1-encoderRotAngle1) > 90) && (Math.abs(encoderRotAngle1+transTheta1) > 90)){
		// 	if(encoderRotAngle1 < 0){
		// 		transTheta1+= 180;
		// 	}
		// 	else {
		// 		transTheta1 -= 180;
		// 	}
		// 	resultMag1 *= -1;
		// } // least distance traveled / drive only displacement

		// if ((Math.abs(transTheta2-encoderRotAngle2) > 90) && (Math.abs(encoderRotAngle2+transTheta2) > 90)){
		// 	if(encoderRotAngle2 < 0){
		// 		transTheta2+= 180;
		// 	}
		// 	else {
		// 		transTheta2 -= 180;
		// 	}
		// 	resultMag2 *= -1;
		// } // least distance traveled / drive only displacement

		// if ((Math.abs(transTheta3-encoderRotAngle3) > 90) && (Math.abs(encoderRotAngle3+transTheta3) > 90)){
		// 	if(encoderRotAngle3 < 0){
		// 		transTheta3 += 180;
		// 	}
		// 	else {
		// 		transTheta3 -= 180;
		// 	}
		// 	resultMag3 *= -1;
		// } // least distance traveled / drive only displacement
		

		// ------ output --------
		runRotationalPID0();
		runRotationalPID1();
		runRotationalPID2();
		runRotationalPID3();
		rotMot0.set(rotSpeed0);
		transMot0.set(-resultMag0);
		rotMot1.set(rotSpeed1);
		transMot1.set(-resultMag1);
		rotMot2.set(rotSpeed2);
		transMot2.set(-resultMag2);
		rotMot3.set(rotSpeed3);
		transMot3.set(-resultMag3);
	} // run at all times in state teleop

  public void runEncoders(){
    DB_rot0.calc();
    encoderRotAngle0 = DB_rot0.get()/8.5; // EXPERIMENTALLY DERIVED!!! NEED TO GET A MORE ACCURATE ONE
	encoderTransSpeed0 = DB_trans0.getVelocity()*0.225; // NEED TO CHANGE (POTENTIALLY) COULD BE WRONG
	DB_rot1.calc();
    encoderRotAngle1 = DB_rot1.get()/8.5; // EXPERIMENTALLY DERIVED!!! NEED TO GET A MORE ACCURATE ONE
	encoderTransSpeed1 = DB_trans1.getVelocity()*0.225; // NEED TO CHANGE (POTENTIALLY) COULD BE WRONG
	DB_rot2.calc();
    encoderRotAngle2 = DB_rot2.get()/8.5; // EXPERIMENTALLY DERIVED!!! NEED TO GET A MORE ACCURATE ONE
	encoderTransSpeed2 = DB_trans2.getVelocity()*0.225; // NEED TO CHANGE (POTENTIALLY) COULD BE WRONG
	DB_rot3.calc();
    encoderRotAngle3 = DB_rot3.get()/8.5; // EXPERIMENTALLY DERIVED!!! NEED TO GET A MORE ACCURATE ONE
	encoderTransSpeed3 = DB_trans3.getVelocity()*0.225; // NEED TO CHANGE (POTENTIALLY) COULD BE WRONG

	pitch = NX_gyro.getPitch();
	roll = NX_gyro.getRoll();
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
		if (transX == 0 && transY == 0){
			// return 1477;
			return 0;
		} 
		if ((angle <= 180 && angle >= 0 ) || (angle < 0 && angle >= -90)) {
			bearing = 90 - angle;
		} else {
			bearing = 90 - angle -360;
		}
        return bearing;
	} // change value to a bearing

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
		// SmartDashboard.putNumber("Rotation Angle 0", encoderRotAngle0);
		// SmartDashboard.putNumber("Trans Speed 0", encoderTransSpeed0);
		// SmartDashboard.putNumber("Rotation Angle 0", encoderRotAngle0);
		// SmartDashboard.putNumber("Trans Speed 0", encoderTransSpeed0);
		// SmartDashboard.putNumber("Translation Theta0", transTheta0); // 1_comment
		// SmartDashboard.putNumber("Rotation Angle 1", encoderRotAngle1);
		// SmartDashboard.putNumber("Trans Speed 1", encoderTransSpeed1);
		// SmartDashboard.putNumber("Rotation Angle 1", encoderRotAngle1);
		// SmartDashboard.putNumber("Trans Speed 1", encoderTransSpeed1);
		// SmartDashboard.putNumber("Translation Theta1", transTheta1); // 1_comment
		// SmartDashboard.putNumber("TransMag", transMag);
		// SmartDashboard.putNumber("RotMag", rotMag);
		// SmartDashboard.putNumber("ResultMag0", resultMag0);
		// SmartDashboard.putNumber("RotSpeed0", rotSpeed0);
		SmartDashboard.putNumber("Yaw", yaw);
		// SmartDashboard.putNumber("encoder0", encoderRotAngle0);
		// SmartDashboard.putNumber("encoder1", encoderRotAngle1);
		// SmartDashboard.putNumber("encoder2", encoderRotAngle2);
		// SmartDashboard.putNumber("encoder3", encoderRotAngle3);
	} // put stuff to smart dashboard
} // Robot


// CODE FOR CW ROTATION AND TRANSLATION ARCHIVE
// 			transRotTempX0 = transX + rotMag*s_c_45;
// 			transRotTempY0 = transY + rotMag*s_c_45;
// 			transTheta0 = toBearing(Math.toDegrees(Math.atan2(transRotTempY0, transRotTempX0)) - yaw);
// 			resultMag0 = Math.hypot(transRotTempX0, transRotTempY0);
// 			// if (transRotTempX0 < 0 || transRotTempY0 < 0){
// 			// 	resultMag0 *= -1;
// 			// }
			
// 			transRotTempX1 = transX + rotMag*s_c_45;
// 			transRotTempY1 = - transX + rotMag*s_c_45;
// 			transTheta1 = toBearing(Math.toDegrees(Math.atan2(transRotTempY1, transRotTempX1)) - yaw);
// 			resultMag1 = Math.hypot(transRotTempX1, transRotTempY1);
// 			// if (transRotTempX1 < 0 || transRotTempY1 < 0){
// 			// 	resultMag1 *= -1;
// 			// }

// 			transRotTempX2 = - transX + rotMag*s_c_45;
// 			transRotTempY2 = transX + rotMag*s_c_45;
// 			transTheta2 = toBearing(Math.toDegrees(Math.atan2(transRotTempY2, transRotTempX2)) - yaw);
// 			resultMag2 = Math.hypot(transRotTempX2, transRotTempY2);
// 			// if (transRotTempX1 < 0 || transRotTempY1 < 0){
// 			// 	resultMag1 *= -1;
// 			// }

// 			transRotTempX3 = - transX + rotMag*s_c_45;
// 			transRotTempY3 = - transX + rotMag*s_c_45;
// 			transTheta3 = toBearing(Math.toDegrees(Math.atan2(transRotTempY3, transRotTempX3)) - yaw);
// 			resultMag3 = Math.hypot(transRotTempX3, transRotTempY3);
// 			// if (transRotTempX1 < 0 || transRotTempY1 < 0){
// 			// 	resultMag1 *= -1;
// 			// }
