package org.firstinspires.ftc.teamcode.subsystems;


import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;


public class RTPAxon {
    private final AnalogInput servoEncoder;
    private final CRServo servo;
    private boolean rtp;
    private double power;
    private double maxPower;
    private Direction direction;
    private double previousAngle;
    private double totalRotation;
    private double targetRotation;
    private double kP;

    public enum Direction{
        FORWARD,
        REVERSE
    }

    //region constructors
    public RTPAxon(CRServo servo, AnalogInput encoder) {
        rtp = true;
        this.servo = servo;
        servoEncoder = encoder;
        direction = Direction.FORWARD;
        initialize();
    }
    public RTPAxon(CRServo servo, AnalogInput encoder, Direction direction) {
        rtp = true;
        this.servo = servo;
        servoEncoder = encoder;
        this.direction = direction;
        initialize();
    }
    private void initialize(){
        previousAngle = getCurrentAngle();
        totalRotation = 0;
        kP = 0.015;
        maxPower = 0.25;
    }
    //endregion


    public void setDirection(Direction direction){
        this.direction = direction;
    }
    public void setPower(double power) {
        this.power = Math.max(-maxPower, Math.min(maxPower, power));
        servo.setPower(this.power * (direction == Direction.REVERSE ? -1 : 1));
    }
    public double getPower(){
        return power;
    }
    public void setMaxPower(double maxPower){
        this.maxPower = maxPower;
    }
    public double getMaxPower(){
        return maxPower;
    }
    public void setRtp(boolean rtp){
        this.rtp = rtp;
    }
    public boolean getRtp(){
        return rtp;
    }
    public void setK(double k){
        kP = k;
    }
    public double getK() {
        return kP;
    }
    //region rtp stuff
    private double getCurrentAngle() {
        if (servoEncoder == null) return 0;
        return (servoEncoder.getVoltage() / 3.3) * (direction.equals(Direction.REVERSE) ? -360 : 360);
    }
    private double getTotalRotation(){
        return totalRotation;
    }
    private double getTargetRotation(){
        return targetRotation;
    }

    public void changeTargetRotation(double change){
        targetRotation += change;
    }
    public void setTargetRotation(double target){
        targetRotation = target;
    }

    public synchronized void update() {

        double angleDifference = getCurrentAngle() - previousAngle;

        if (angleDifference > 180) {
            angleDifference -= 360;
        } else if (angleDifference < -180) {
            angleDifference += 360;
        }

        totalRotation += angleDifference;
        previousAngle = getCurrentAngle();
        double error = targetRotation - totalRotation;
        if (!rtp) return;
        double DEADZONE = 0.5;
        if (Math.abs(error) > DEADZONE) {
            double power = Math.min(maxPower, Math.abs(error * kP)) * Math.signum(error);
            setPower(-power);
        } else {
            setPower(0);
        }
    }
    //endregion
    @SuppressLint("DefaultLocale")
    public String log(){
        return String.format("Current Angle: %f\nTotal Rotation: %f\nTarget Rotation: %f\nCurrent Power: %f", getCurrentAngle(), totalRotation, targetRotation, power);
    }
}
