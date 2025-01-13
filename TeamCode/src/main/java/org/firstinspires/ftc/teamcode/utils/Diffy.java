package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Diffy {
    private final CRServo leftServo;
    private final CRServo rightServo;
    private final AnalogInput leftEncoder;
    private final AnalogInput rightEncoder;

    private double leftLocalAngle;
    private double leftPreviousLocalAngle;
    private double leftTotalRotation;
    private double rightLocalAngle;
    private double rightPreviousLocalAngle;
    private double rightTotalRotation;
    private double rightStartingAngle;
    private double leftStartingAngle;
    public Diffy(HardwareMap hmap){
        leftServo = hmap.crservo.get("leftDiffy");
        rightServo = hmap.crservo.get("rightDiffy");
        leftEncoder = hmap.get(AnalogInput.class, "leftDiffyEncoder");
        rightEncoder = hmap.get(AnalogInput.class, "rightDiffyEncoder");
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftStartingAngle = leftEncoder.getVoltage() / 3.3 * 360.0;
        rightStartingAngle = rightEncoder.getVoltage() / 3.3 * 360.0;
        rightTotalRotation = -rightStartingAngle;
        leftTotalRotation = -leftStartingAngle;
    }

    public void setLeftPower(double power) {
        leftServo.setPower(power);
    }
    public void setRightPower(double power) {
        rightServo.setPower(power);
    }
    public void updatePosition() {
        leftLocalAngle = leftEncoder.getVoltage() / 3.3 * 360.0;
        rightLocalAngle = rightEncoder.getVoltage() / 3.3 * 360.0;
        double leftAngleDifference = leftLocalAngle - leftPreviousLocalAngle;
        if (leftAngleDifference < -180) {
            leftAngleDifference += 360;
        } else if (leftAngleDifference > 180) {
            leftAngleDifference -= 360;
        }

        double rightAngleDifference = rightLocalAngle - rightPreviousLocalAngle;
        if (rightAngleDifference < -180) {
            rightAngleDifference += 360;
        } else if (rightAngleDifference > 180) {
            rightAngleDifference -= 360;
        }

        leftTotalRotation += leftAngleDifference;
        rightTotalRotation += rightAngleDifference;

        leftPreviousLocalAngle = leftLocalAngle;
        rightPreviousLocalAngle = rightLocalAngle;
    }

    public double getLeftLocalAngle() {
        return leftLocalAngle;
    }
    public double getRightLocalAngle() {
        return rightLocalAngle;
    }
    public double getRightTotalRotation(){
        return rightTotalRotation;
    }
    public double getLeftTotalRotation(){
        return leftTotalRotation;
    }
    public double getRightStartingAngle(){return rightStartingAngle;}
    public double getLeftStartingAngle(){return leftStartingAngle;}
}
