package org.firstinspires.ftc.teamcode.utils;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BackArm {
    private final CRServo leftServo;
    private final CRServo rightServo;
    AnalogInput rightEncoder;
    private double localAngle;
    private double previousAngle;
    private double totalRotation;
    private double targetRotation;
    private final double startingRotation;
    private final boolean runToPosition;

    public BackArm(double upPos, double downPos, HardwareMap hmap, boolean shouldRtp){
        runToPosition = shouldRtp;
        leftServo = hmap.crservo.get("leftFlip");
        rightServo = hmap.crservo.get("rightFlip");
        rightEncoder = hmap.get(AnalogInput.class, "rightArmEncoder");
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        startingRotation = ((rightEncoder.getVoltage() / 3.3) * 360)/3;
        totalRotation = -startingRotation;

    }
    public void update(){
        localAngle = (rightEncoder.getVoltage() / 3.3 * 360)/3;

        double angleDifference = localAngle - previousAngle;

        if (angleDifference < -180) {
            angleDifference += 360;
        } else if (angleDifference > 180) {
            angleDifference -= 360;
        }

        totalRotation += angleDifference;

        previousAngle = localAngle;
        if (!runToPosition) return;
        double power = 0.4;
        if (Math.abs(targetRotation-totalRotation)<10) {
            setPower(0);
            return;
        }
        else if (totalRotation<targetRotation) setPower(power);
        else if (totalRotation>targetRotation) {
            setPower(-power);
        }
    }
    public void setTargetRotation(double target){
        targetRotation = target;
    }
    public double getTargetRotation(){return targetRotation;}
    public double getAngle(){
        return localAngle;
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