package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Diffy {
    private final RTPAxon leftServo;
    private final RTPAxon rightServo;
    private double grabberTwist;
    private double grabberRotation;

    public Diffy(HardwareMap hmap) {
        CRServo lServo = hmap.crservo.get("leftDiffy");
        CRServo rServo = hmap.crservo.get("rightDiffy");
        AnalogInput lEncoder = hmap.get(AnalogInput.class, "leftDiffyEncoder");
        AnalogInput rEncoder = hmap.get(AnalogInput.class, "rightDiffyEncoder");
        leftServo = new RTPAxon(lServo, lEncoder, RTPAxon.Direction.REVERSE);
        rightServo = new RTPAxon(rServo, rEncoder, RTPAxon.Direction.FORWARD);
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

    public double getLeftRotation() {
        return leftServo.getTotalRotation();
    }

    public double getRightRotation() {
        return rightServo.getTotalRotation();
    }

    public void setLeftTargetRotation(double target) {
        leftServo.setTargetRotation(target);
    }

    public void setRightTargetRotation(double target) {
        rightServo.setTargetRotation(target);
    }

    public double getLeftTargetRotation() {
        return leftServo.getTargetRotation();
    }

    public double getRightTargetRotation() {
        return rightServo.getTargetRotation();
    }

    public void changeLeftTargetRotation(double amount) {
        leftServo.changeTargetRotation(amount);
    }

    public void changeRightTargetRotation(double amount) {
        rightServo.changeTargetRotation(amount);
    }

    public void twistGrabber(double degrees) {
        double formattedRotation = degrees * 0.55555555;
        if (grabberTwist + degrees <= 90 && grabberTwist + degrees >= -90) {
            grabberTwist += degrees;
            changeLeftTargetRotation(formattedRotation);
            changeRightTargetRotation(-formattedRotation);
        }
    }

    public double getGrabberTwist() {
        return grabberTwist;
    }

    public void rotateGrabber(double degrees) {
        if (grabberRotation + degrees <= 315 && grabberRotation + degrees >= 0) {
            grabberRotation += degrees;
            changeLeftTargetRotation(-degrees);
            changeRightTargetRotation(-degrees);
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
