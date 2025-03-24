package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HorizSlidePair {
    private final CRServo leftSlide;
    private final RTPAxon rightSlide;
    private boolean manualMode = false;

    public HorizSlidePair(HardwareMap hmap) {
        leftSlide = hmap.crservo.get("leftHorizSlide");
        rightSlide = new RTPAxon(hmap.crservo.get("rightHorizSlide"), hmap.get(AnalogInput.class, "rightHorizSlideEncoder"), RTPAxon.Direction.REVERSE);
        //leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update() {
        rightSlide.update();
        if (!manualMode) {
            leftSlide.setPower(rightSlide.getPower());
        }
    }

    public void setManualMode(boolean manual) {
        manualMode = manual;
        rightSlide.setRtp(!manual);
    }

    public boolean isManualMode() {
        return manualMode;
    }

    public void setManualPower(double power) {
        if (!manualMode) throw new IllegalStateException("Not in manual mode");

        double limitedPower = Math.max(-rightSlide.getMaxPower(),
                Math.min(rightSlide.getMaxPower(), power));
        rightSlide.setPower(limitedPower);
        leftSlide.setPower(limitedPower);
    }

    public void setTargetRotation(double target) {
        if (!manualMode) {
            rightSlide.setTargetRotation(target);
        }
    }
    public void changeTargetRotation(double delta) {
        if (!manualMode) {
            rightSlide.changeTargetRotation(delta);
        }
    }

    public double getTargetRotation() {
        return rightSlide.getTargetRotation();
    }

    public double getRotation() {
        return rightSlide.getTotalRotation();
    }

    public void setMaxPower(double power) {
        rightSlide.setMaxPower(power);
    }

    @SuppressLint("DefaultLocale")
    public String log(){
        return String.format("%s\nManual Mode: %b\nTotal Rotation",
                rightSlide.log(),
                manualMode);
    }
}