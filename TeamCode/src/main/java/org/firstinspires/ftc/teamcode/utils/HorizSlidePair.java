package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HorizSlidePair {
    CRServo rightSlide;
    CRServo leftSlide;
    AnalogInput leftSlideEncoder;
    AnalogInput rightSlideEncoder;
    private double currentAngle = 0.0;
    private double previousAngle = 0.0;
    private double totalRotation = 0.0;
    private double targetRotation = 0.0;
    private final double maxExtend;
    private final double minExtend;
    private final double startPosition;

    public HorizSlidePair(HardwareMap hmap){
        rightSlide = hmap.get(CRServo.class, "rightHorizSlide");
        leftSlide = hmap.get(CRServo.class, "leftHorizSlide");
        rightSlideEncoder = hmap.get(AnalogInput.class, "rightHorizSlideEncoder");
        leftSlideEncoder = hmap.get(AnalogInput.class, "leftHorizSlideEncoder");
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        maxExtend = 450;
        minExtend = 0;
        startPosition = leftSlideEncoder.getVoltage() / 3.3 * 360;
    }
    public double getCurrentPosition() { return currentAngle; }

    public void update(){
        double leftAngle = leftSlideEncoder.getVoltage() / 3.3 * 360;

        currentAngle = leftAngle-startPosition;

        double angleDifference = currentAngle - previousAngle;

        if (angleDifference < -180) {
            angleDifference += 360;
        } else if (angleDifference > 180) {
            angleDifference -= 360;
        }

        totalRotation += angleDifference;

        previousAngle = currentAngle;

        if (Math.abs(targetRotation-totalRotation)<10) {
            setPower(0);
            return;
        }
        else if (totalRotation<targetRotation) setPower(0.3);
        else if (totalRotation>targetRotation) {
            setPower(-0.3);
        }
    }

    public double getTotalRotation() {
        return totalRotation;
    }

    public void setPower(double power){
        rightSlide.setPower(power);
        leftSlide.setPower(power);
    }
    public void setTargetRotation(double target){
        if (target <= maxExtend && target >= minExtend)
            targetRotation = target;
    }
    public double getTargetRotation(){return targetRotation;}
    public double getRawEncoders(){
        return leftSlideEncoder.getVoltage();
    }
}
