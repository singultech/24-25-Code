package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Diffy {
    private final RTPAxon leftServo;
    private final RTPAxon rightServo;
    private double grabberTwist;
    private double grabberRotation;

    public enum Side {
        LEFT,
        RIGHT
    }

    public Diffy(HardwareMap hmap) {
        leftServo = new RTPAxon(hmap.crservo.get("leftDiffy"), hmap.get(AnalogInput.class, "leftDiffyEncoder"), RTPAxon.Direction.REVERSE);
        rightServo = new RTPAxon(hmap.crservo.get("rightDiffy"), hmap.get(AnalogInput.class, "rightDiffyEncoder"), RTPAxon.Direction.FORWARD);
        grabberTwist = 0;
        grabberRotation = 315;
    }

    public void setLeftPower(double power) {
        leftServo.setPower(power);
    }

    public void setRightPower(double power) {
        rightServo.setPower(power);
    }

    public void update() {
        leftServo.update();
        rightServo.update();
    }

    public void setManualMode(boolean manual) {
        leftServo.setRtp(!manual);
        rightServo.setRtp(!manual);
    }

    public boolean isManualMode() {
        return !leftServo.getRtp();
    }

    public void setMaxPower(double power) {
        leftServo.setMaxPower(power);
        rightServo.setMaxPower(power);
    }

    public double getRotation(Side side) {
        if (side == Side.LEFT) return leftServo.getTotalRotation();
        return rightServo.getTotalRotation();
    }

    public void setTargetRotation(Side side, double target) {
        if (side == Side.LEFT) leftServo.setTargetRotation(target);
        else rightServo.setTargetRotation(target);
    }

    public void setTargetRotation(double leftTarget, double rightTarget) {
        setTargetRotation(Side.LEFT, leftTarget);
        setTargetRotation(Side.RIGHT, rightTarget);
    }

    public double getTargetRotation(Side side) {
        if (side == Side.LEFT) return leftServo.getTargetRotation();
        return rightServo.getTargetRotation();
    }

    public void changeTargetRotation(Side side, double amount) {
        if (side == Side.LEFT) leftServo.changeTargetRotation(amount);
        else rightServo.changeTargetRotation(amount);
    }

    public void changeTargetRotation(double leftAmount, double rightAmount) {
        changeTargetRotation(Side.LEFT, leftAmount);
        changeTargetRotation(Side.RIGHT, rightAmount);
    }



    public void twistGrabber(double degrees) {
        double formattedRotation = degrees * 0.55555555;
        if (grabberTwist + degrees <= 90 && grabberTwist + degrees >= -90) {
            grabberTwist += degrees;
            changeTargetRotation(formattedRotation, -formattedRotation);
        }
    }

    public double getGrabberTwist() {
        return grabberTwist;
    }

    public void rotateGrabber(double degrees) {
        if (grabberRotation + degrees <= 315 && grabberRotation + degrees >= 0) {
            grabberRotation += degrees;
            changeTargetRotation(-degrees, -degrees);
        }
    }

    public double getGrabberRotation() {
        return grabberRotation;
    }

    @SuppressLint("DefaultLocale")
    public String log() {
        return String.format("Left Servo: %s\nRight Servo: %s\nGrabber Twist: %f\nGrabber Rotation: %f",
                leftServo.log(),
                rightServo.log(),
                grabberTwist,
                grabberRotation);
    }
}
