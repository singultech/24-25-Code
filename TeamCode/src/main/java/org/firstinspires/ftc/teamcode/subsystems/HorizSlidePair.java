package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HorizSlidePair {
    private final DcMotorEx leftSlide;
    private final DcMotorEx rightSlide;
    private boolean manualMode;

    public HorizSlidePair(HardwareMap hmap) {
        leftSlide = hmap.get(DcMotorEx.class, "leftHorizSlide");
        rightSlide = hmap.get(DcMotorEx.class, "rightHorizSlide");
        manualMode = false;
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setManualMode(boolean manual){
        manualMode = manual;
        if(!manual){
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setManualPower(double power){
        if(!manualMode) throw new IllegalStateException("Not in manual mode");

        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    public boolean isManualMode(){
        return manualMode;
    }

    public void setTargetRotation(int target){
        rightSlide.setTargetPosition(target);
        leftSlide.setTargetPosition(target);
    }

    public int getTargetRotation(){
        return rightSlide.getTargetPosition();
    }

    public void changeTargetRotation(int delta){
        int currentPos = rightSlide.getTargetPosition();
        int newPos = currentPos + delta;
        setTargetRotation(newPos);
    }

    public int getRotation(){
        return rightSlide.getCurrentPosition();
    }
    public double getPower(){
        return rightSlide.getPower();
    }
    public void forceStopPower(){
        rightSlide.setPower(0);
    }
    public boolean isAtTarget(){
        return Math.abs(rightSlide.getTargetPosition()- rightSlide.getCurrentPosition()) < 10;
    }

    @NonNull
    public String toString(){
        return String.format("Manual Mode: %b\nTarget Pos: %d\nCurrent Pos: %d", manualMode, getTargetRotation(), getRotation());
    }
    @Config
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
                telemetry.update();
            }
        }
    }

}