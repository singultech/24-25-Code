package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HorizSlidePair {
    private final CRServo leftSlide;
    private final RTPAxon rightSlide;
    private boolean manualMode = false;

    public HorizSlidePair(HardwareMap hmap) {
        leftSlide = hmap.crservo.get("rightHorizSlide");
        rightSlide = new RTPAxon(hmap.crservo.get("leftHorizSlide"), hmap.get(AnalogInput.class, "rightHorizSlideEncoder"), RTPAxon.Direction.REVERSE);
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

    @TeleOp(name = "Horiz Slides Test", group = "test")
    public static class HorizontalSlidesTest extends LinearOpMode {
        public static int DEGREE_INCREMENT = 15;
        public static double MAX_MANUAL_POWER = 0.5;
        @Override
        public void runOpMode() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            HorizSlidePair slides = new HorizSlidePair(hardwareMap);
            slides.setManualMode(true);
            GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
            boolean manualMode = false;

            waitForStart();

            while (opModeIsActive()) {
                gamepads.copyStates();
                slides.update();

                if(gamepads.isPressed("triangle")){
                    manualMode = !manualMode;
                    slides.setManualMode(manualMode);
                }

                if(manualMode) {
                    if (gamepads.isHeld(-1,"left_dpad")) slides.setManualPower(-MAX_MANUAL_POWER);
                    else if (gamepads.isHeld(-1,"right_dpad")) slides.setManualPower(MAX_MANUAL_POWER);
                    else slides.setManualPower(0);
                } else {
                    if (gamepads.isPressed("up_dpad")) slides.changeTargetRotation(DEGREE_INCREMENT);
                    if (gamepads.isPressed("down_dpad")) slides.changeTargetRotation(-DEGREE_INCREMENT);
                }

                telemetry.addLine(slides.log());
                telemetry.addData("Manual Mode", manualMode ? "ON" : "OFF");
                telemetry.update();
            }
        }
    }

}