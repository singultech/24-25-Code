package org.firstinspires.ftc.teamcode.subsystems;


import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class CRAxon {
    private final AnalogInput servoEncoder;
    private final CRServo servo;
    private boolean rtp;
    private double power;
    private final DcMotorSimple.Direction direction;
    private double previousAngle;
    private double totalRotation;
    private double targetRotation;
    private final double startingAngle;

    //region constructors
    public CRAxon(CRServo s) {
        rtp = false;
        servo = s;
        servoEncoder = null;
        direction = DcMotorSimple.Direction.FORWARD;
        startingAngle = 0.0;
    }
    public CRAxon(CRServo servo, AnalogInput encoder) {
        rtp = true;
        this.servo = servo;
        servoEncoder = encoder;
        direction = DcMotorSimple.Direction.FORWARD;
        startingAngle = getCurrentAngle();
        previousAngle = getCurrentAngle();
        totalRotation = 0;
    }
    public CRAxon(CRServo s, DcMotorSimple.Direction direction) {
        rtp = false;
        servo = s;
        servoEncoder = null;
        this.direction = direction;
        startingAngle = 0.0;
    }
    public CRAxon(CRServo servo, AnalogInput encoder, DcMotorSimple.Direction direction) {
        rtp = true;
        this.servo = servo;
        servoEncoder = encoder;
        this.direction = direction;
        startingAngle = getCurrentAngle();
        previousAngle = getCurrentAngle();
        totalRotation = 0;
    }
    //endregion

    public void setPower(double power) {
        this.power = power;
        if (!rtp) servo.setPower(this.power);
    }
    public double getPower(){
        return power;
    }
    public void setRtp(boolean rtp){
        this.rtp = rtp;
    }
    public boolean getRtp(){
        return rtp;
    }
    //region rtp stuff
    private double getCurrentAngle() {
        if (servoEncoder == null) return 0;
        return (servoEncoder.getVoltage() / 3.3) * (direction.equals(DcMotorSimple.Direction.REVERSE) ? -360 : 360);
    }
    private double getTotalRotation(){
        return totalRotation;
    }
    private double getTargetRotation(){
        return targetRotation;
    }
    public void update() {
        if (!rtp) return;
        double angleDifference = getCurrentAngle() - previousAngle;

        if (angleDifference > 180) {
            angleDifference -= 360;
        } else if (angleDifference < -180) {
            angleDifference += 360;
        }

        totalRotation += angleDifference;
        previousAngle = getCurrentAngle();


//        double maxPower = 0.25;
//        double kP = 0.015;
//        double error = targetRotation - totalRotation;
//
//        if (Math.abs(error) > 1) {
//            double power = Math.min(maxPower, Math.abs(error * kP)) * Math.signum(error);
//            setPower(power);
//        } else {
//            setPower(0);
//        }
    }
    //endregion
    @SuppressLint("DefaultLocale")
    public String log(){
        return String.format("Current Angle: %f\nTotal Rotation: %f\nTarget Rotation: %f", getCurrentAngle(), totalRotation, targetRotation);
    }
}
