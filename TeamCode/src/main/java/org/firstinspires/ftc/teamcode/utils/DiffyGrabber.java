package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DiffyGrabber {
    Servo leftServo;
    Servo rightServo;
    AnalogInput leftEncoder;
    AnalogInput rightEncoder;

    private double leftTarget;
    private double rightTarget;
    private double minimum;
    private double maximum;
    public DiffyGrabber(HardwareMap hmap){
        leftServo = hmap.servo.get("leftDiffy");
        rightServo = hmap.servo.get("rightDiffy");
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftTarget = 0.0;
        rightTarget = 0.0;
        minimum = 0;
        maximum = 1;
    }

    public void setLeftPosition(double position){
        if (position>maximum) leftTarget = maximum;
        else if (position<minimum) leftTarget = minimum;
        else leftTarget=position;
        leftServo.setPosition(leftTarget);
    }

    public void setRightPosition(double position){
        if (position>maximum) rightTarget = maximum;
        else if (position<minimum) rightTarget = minimum;
        else rightTarget=position;
        rightServo.setPosition(rightTarget);
    }
    public void changeLeftPosition(double change){
        if ((leftTarget+change)>=minimum && (leftTarget+change)<=maximum) {
            leftTarget+=change;
            leftServo.setPosition(leftTarget);
        }
    }
    public void changeRightPosition(double change){
        if ((rightTarget+change)>=minimum && (rightTarget+change)<=maximum) {
            rightTarget+=change;
            rightServo.setPosition(rightTarget);
        }
    }
/*
    public void updatePosition() {
        double leftAngle = leftEncoder.getVoltage() / 3.3 * 360.0;
        double rightAngle = rightEncoder.getVoltage() / 3.3 * 360.0;

        leftCurrentAngle = leftAngle;
        rightCurrentAngle = rightAngle;

        double leftAngleDifference = leftCurrentAngle - leftPreviousAngle;
        if (leftAngleDifference < -180) {
            leftAngleDifference += 360;
        } else if (leftAngleDifference > 180) {
            leftAngleDifference -= 360;
        }

        double rightAngleDifference = rightCurrentAngle - rightPreviousAngle;
        if (rightAngleDifference < -180) {
            rightAngleDifference += 360;
        } else if (rightAngleDifference > 180) {
            rightAngleDifference -= 360;
        }

        leftTotalRotation += leftAngleDifference;
        rightTotalRotation += rightAngleDifference;

        leftPreviousAngle = leftCurrentAngle;
        rightPreviousAngle = rightCurrentAngle;
    }*/

    public double getLeftPosition() {
        return leftServo.getPosition();
    }
    public double getRightPosition() {
        return rightServo.getPosition();
    }
    public double getRightTarget(){
        return rightTarget;
    }
    public double getLeftTarget(){
        return leftTarget;
    }
}
