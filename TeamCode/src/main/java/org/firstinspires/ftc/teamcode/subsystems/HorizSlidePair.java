package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HorizSlidePair {
    private final CRServo leftSlides;
    private final RTPAxon rightSlides;
    private boolean manualMode = false;
    private final double GEAR_RATIO = 3; // 3:1

    public HorizSlidePair(HardwareMap hmap) {
        leftSlides = hmap.crservo.get("leftFlip");
        CRServo rServo = hmap.crservo.get("rightFlip");
        AnalogInput rightEncoder = hmap.get(AnalogInput.class, "rightArmEncoder");
        rightSlides = new RTPAxon(rServo, rightEncoder);
        leftSlides.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update() {
        rightSlides.update();
        if (!manualMode) {
            leftSlides.setPower(rightSlides.getPower());
        }
    }

    public void setManualMode(boolean manual) {
        manualMode = manual;
        rightSlides.setRtp(!manual);
    }

    public boolean isManualMode() {
        return manualMode;
    }

    public void setManualPower(double power) {
        if (!manualMode) throw new IllegalStateException("Not in manual mode");

        double limitedPower = Math.max(-rightSlides.getMaxPower(),
                Math.min(rightSlides.getMaxPower(), power));
        rightSlides.setPower(limitedPower);
        leftSlides.setPower(limitedPower);
    }

    public void setTargetRotation(double target) {
        if (!manualMode) {
            rightSlides.setTargetRotation(target*GEAR_RATIO);
        }
    }
    public void changeTargetRotation(double delta) {
        if (!manualMode) {
            rightSlides.changeTargetRotation(delta*GEAR_RATIO);
        }
    }

    public double getTargetRotation() {
        return rightSlides.getTargetRotation()/GEAR_RATIO;
    }

    public double getRotation() {
        return rightSlides.getTotalRotation()/GEAR_RATIO;
    }

    public void setMaxPower(double power) {
        rightSlides.setMaxPower(power);
    }

    @SuppressLint("DefaultLocale")
    public String log(){
        return String.format("%s\nManual Mode: %b\nTotal Rotation w/ Gear ratio %f",
                rightSlides.log(),
                manualMode, getRotation());
    }
}