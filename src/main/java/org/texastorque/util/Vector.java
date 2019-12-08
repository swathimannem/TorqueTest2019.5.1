/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.texastorque.util;

public class Vector {
    private double vectorX;
    private double vectorY;
    private double polarR;
    private double polarTheta;

    public Vector(double vectorX, double vectorY){
        this.vectorX = vectorX;
        this.vectorY = vectorY;
        makePolar();
    }

    public double getMag(){
        return polarR;
    }

    public double getTheta(){
        return polarTheta;
    }

    public double getX(){
        return vectorX;
    }

    public double getY(){
        return vectorY;
    }

    public void yawOffset(double yawOffset){
        makePolar();
        polarTheta -= yawOffset;
        vectorX = polarR*Math.toDegrees(Math.cos(Math.toRadians(polarTheta)));
        vectorY = polarR*Math.toDegrees(Math.sin(Math.toRadians(polarTheta)));
        makePolar();
    }

    public void makePolar(){
        polarR = Math.hypot(vectorX, vectorY);
        polarTheta = Math.toDegrees(Math.atan2(vectorY, vectorX));
    }

    public void setVectorValues(double vectorX, double vectorY){
        this.vectorX = vectorX;
        this.vectorY = vectorY;
    }

    public double addVectorMag(Vector vector2){
        return Math.hypot((vectorX + vector2.getX()), (vectorY + vector2.getY()));
    } // add two vectors to get a magnitude

    public Vector addVectorDirections(Vector vector2, int directionX, int directionY){
        double tempX = vectorX + vector2.getX()*directionX;
        double tempY = vectorY + vector2.getY()*directionY;
        Vector tempVector = new Vector(tempX, tempY);
        return tempVector;
    }
} // vector class
