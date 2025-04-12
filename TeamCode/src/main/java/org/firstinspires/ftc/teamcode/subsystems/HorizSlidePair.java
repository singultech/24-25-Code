package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HorizSlidePair {
    private final CRServo leftSlide;
    private final RTPAxon rightSlide;
    private boolean manualMode;

    public HorizSlidePair(HardwareMap hmap) {
        CRServo leftServo = hmap.crservo.get("leftHorizSlide");
        CRServo rightServo = hmap.crservo.get("rightHorizSlide");
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        AnalogInput rightEncoder = hmap.get(AnalogInput.class, "rightHorizSlideEncoder");
        rightSlide = new RTPAxon(rightServo, rightEncoder);
        leftSlide = leftServo;
        manualMode = false;
    }

    public void update(){
        rightSlide.update();
        leftSlide.setPower(rightSlide.getPower());
    }

    public void setManualMode(boolean manual){
        manualMode = manual;
        rightSlide.setRtp(!manual);
    }

    public void setManualPower(double power){
        if(!manualMode) throw new IllegalStateException("Not in manual mode");

        double limitedPower = Math.max(-rightSlide.getMaxPower(), Math.min(rightSlide.getMaxPower(), power));
        rightSlide.setPower(limitedPower);
    }

    public boolean isManualMode(){
        return manualMode;
    }

    public void setTargetRotation(double target){
        rightSlide.setTargetRotation(target);
    }

    public double getTargetRotation(){
        return rightSlide.getTargetRotation();
    }

    public void changeTargetRotation(double delta){
        rightSlide.changeTargetRotation(delta);
    }

    public double getRotation(){
        return rightSlide.getTotalRotation();
    }

    @NonNull
    public String toString(){
        return String.format("%s\n Manual Mode: %b", rightSlide.log(), manualMode);
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

            waitForStart();

            while (opModeIsActive()) {
                gamepads.copyStates();
                slides.update();


                if(gamepads.isPressed("triangle")){
                    slides.setManualMode(!slides.isManualMode());
                }

                if(slides.isManualMode()) {
                    if (gamepads.isHeld(-1,"left_dpad")) slides.setManualPower(-MAX_MANUAL_POWER);
                    else if (gamepads.isHeld(-1,"right_dpad")) slides.setManualPower(MAX_MANUAL_POWER);
                    else slides.setManualPower(0);
                } else {
                    if (gamepads.isPressed("up_dpad")) slides.changeTargetRotation(DEGREE_INCREMENT);
                    if (gamepads.isPressed("down_dpad")) slides.changeTargetRotation(-DEGREE_INCREMENT);
                }

                telemetry.addLine(slides.toString());
                telemetry.addData("Manual Mode", slides.isManualMode() ? "ON" : "OFF");
                telemetry.update();
            }
        }
    }

}