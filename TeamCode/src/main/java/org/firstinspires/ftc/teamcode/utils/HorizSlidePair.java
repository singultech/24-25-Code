package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HorizSlidePair {
    CRServo rightSlide;
    CRServo leftSlide;
    AnalogInput rightSlideEncoder;
    private double localAngle = 0.0;
    private double previousAngle = 0.0;
    private double totalRotation = 0.0;
    private double targetRotation = 0.0;
    private final double maxExtend;
    private final double minExtend;
    private final double startPosition;
    private final boolean runToPosition;

    public HorizSlidePair(HardwareMap hmap, boolean shouldRtp){
        rightSlide = hmap.get(CRServo.class, "rightHorizSlide");
        leftSlide = hmap.get(CRServo.class, "leftHorizSlide");
        rightSlideEncoder = hmap.get(AnalogInput.class, "rightHorizSlideEncoder");
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        runToPosition = shouldRtp;
        maxExtend = 450;
        minExtend = 0;
        startPosition = rightSlideEncoder.getVoltage() / 3.3 * 360;
        totalRotation = -startPosition;
    }
    public double getCurrentPosition() { return localAngle; }

    public void update(){
        localAngle = rightSlideEncoder.getVoltage() / 3.3 * 360;

        double angleDifference = localAngle - previousAngle;

        if (angleDifference < -180) {
            angleDifference += 360;
        } else if (angleDifference > 180) {
            angleDifference -= 360;
        }

        totalRotation += angleDifference;

        previousAngle = localAngle;
        if (!runToPosition) return;
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
        return rightSlideEncoder.getVoltage();
    }
}
