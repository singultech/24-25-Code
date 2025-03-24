package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Diffy {
    private final RTPAxon leftServo;
    private final RTPAxon rightServo;
    private double grabberRoll;
    private double grabberPitch;

    public enum Side {
        LEFT,
        RIGHT
    }

    public Diffy(HardwareMap hmap) {
        leftServo = new RTPAxon(hmap.crservo.get("leftDiffy"), hmap.get(AnalogInput.class, "leftDiffyEncoder"), RTPAxon.Direction.REVERSE);
        rightServo = new RTPAxon(hmap.crservo.get("rightDiffy"), hmap.get(AnalogInput.class, "rightDiffyEncoder"), RTPAxon.Direction.FORWARD);
        grabberRoll = 0;
        grabberPitch = 315;
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



    public void rollGrabber(double degrees) {
        double formattedRotation = degrees * 0.55555555;
        if (grabberRoll + degrees <= 90 && grabberRoll + degrees >= -90) {
            grabberRoll += degrees;
            changeTargetRotation(formattedRotation, -formattedRotation);
        }
    }

    public double getGrabberRoll() {
        return grabberRoll;
    }

    public void pitchGrabber(double degrees) {
        if (grabberPitch + degrees <= 315 && grabberPitch + degrees >= 0) {
            grabberPitch += degrees;
            changeTargetRotation(-degrees, -degrees);
        }
    }

    public double getGrabberPitch() {
        return grabberPitch;
    }

    @SuppressLint("DefaultLocale")
    public String log() {
        return String.format("Left Servo: %s\nRight Servo: %s\nGrabber Roll: %f\nGrabber Pitch: %f",
                leftServo.log(),
                rightServo.log(),
                grabberRoll,
                grabberPitch);
    }
}
