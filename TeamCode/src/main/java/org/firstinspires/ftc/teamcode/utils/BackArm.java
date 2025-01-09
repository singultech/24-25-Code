package org.firstinspires.ftc.teamcode.utils;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BackArm {
    private final CRServo leftServo;
    private final CRServo rightServo;
    private final double upPosition;
    private final double downPosition;
    AnalogInput rightEncoder;

    private double currentAngle = 0.0;
    private double previousAngle = 0.0;
    private double totalRotation = 0.0;

    public BackArm(double upPos, double downPos, HardwareMap hmap){
        leftServo = hmap.crservo.get("leftFlip");
        rightServo = hmap.crservo.get("rightFlip");
        rightEncoder = hmap.get(AnalogInput.class, "rightArmEncoder");
        upPosition = upPos;
        downPosition = downPos;
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void updatePosition(){
        double currentAngle = (rightEncoder.getVoltage() / 3.3 * 360)/3;
        double angleDifference = currentAngle - previousAngle;

        if (angleDifference < -180) {
            angleDifference += 360;
        } else if (angleDifference > 180) {
            angleDifference -= 360;
        }

        totalRotation += angleDifference;

        previousAngle = currentAngle;
    }
    public double getAngle(){
        return currentAngle;
    }
    public double getPosition(){
        return totalRotation;
    }
    public void setPower(double power){
        leftServo.setPower(power);
        rightServo.setPower(power);
    }
    public double getRawOut() {return rightEncoder.getVoltage();}



}