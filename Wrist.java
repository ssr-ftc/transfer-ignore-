package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Wrist {
    public Servo liftServo;
    public Servo rotateServo;

    double position_to_angle;

    double angleGoal;
    double heightGoal;
    ElapsedTime sensorTimeout;

    public Wrist(Servo s1, Servo s2) {
        liftServo = s1;
        rotateServo = s2;

        position_to_angle = -0.00454;

        //colorSensor = sensor;

        sensorTimeout = new ElapsedTime();


    }


    public void goToAngle(double degrees) {
        liftServo.setPosition(degrees*(1/position_to_angle));
    }


    public void smartAngleMove(double armAngle){
        double angleToGoTo =  angleGoal- armAngle;
        goToAngle(angleToGoTo);
    }


    public void setPositionofLiftServo(double pos){
        liftServo.setPosition(pos);
    }
    public void setPositionofturnServo(double pos){
        rotateServo.setPosition(pos);
    }

    public void setGoals(double desiredAngle, double desiredHeight){
        if (!(desiredAngle == Double.NaN)){
            angleGoal = desiredAngle;
        }
        if (!(desiredHeight == Double.NaN)) {
            heightGoal = desiredHeight;
        }
    }


}