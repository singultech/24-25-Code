package org.firstinspires.ftc.teamcode.utils;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BackArm {
    private final CRServo leftServo;
    private final CRServo rightServo;
    AnalogInput rightEncoder;

    private double currentAngle = 0.0;
    private double previousAngle = 0.0;
    private double totalRotation = 0.0;
    private double targetRotation;
    private double startingRotation;

    public BackArm(double upPos, double downPos, HardwareMap hmap){
        leftServo = hmap.crservo.get("leftFlip");
        rightServo = hmap.crservo.get("rightFlip");
        rightEncoder = hmap.get(AnalogInput.class, "rightArmEncoder");
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        targetRotation = 0;
        startingRotation = ((rightEncoder.getVoltage() / 3.3) * 360)/3;

    }
    public void update(){
        double leftAngle = (rightEncoder.getVoltage() / 3.3 * 360)/3;

        currentAngle = leftAngle-startingRotation;

        double angleDifference = currentAngle - previousAngle;

        if (angleDifference < -180) {
            angleDifference += 360;
        } else if (angleDifference > 180) {
            angleDifference -= 360;
        }

        totalRotation += angleDifference;

        previousAngle = currentAngle;
    }
    public void setTargetRotation(double target){
        targetRotation = target;
    }
    public double getTargetRotation(){return targetRotation;}
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