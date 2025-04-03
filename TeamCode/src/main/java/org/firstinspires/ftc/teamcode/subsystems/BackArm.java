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

public class BackArm {
    private final CRServo leftServo;
    private final RTPAxon rightServo;
    private boolean manualMode = false;
    private final double GEAR_RATIO = 3; // 3:1

    public BackArm(HardwareMap hmap) {
        leftServo = hmap.crservo.get("leftFlip");
        CRServo rServo = hmap.crservo.get("rightFlip");
        AnalogInput rightEncoder = hmap.get(AnalogInput.class, "rightArmEncoder");
        rightServo = new RTPAxon(rServo, rightEncoder, RTPAxon.Direction.REVERSE);
        leftServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void update() {
        rightServo.update();
        if (!manualMode) {
            leftServo.setPower(rightServo.getPower());
        }
    }

    public void setManualMode(boolean manual) {
        manualMode = manual;
        rightServo.setRtp(!manual);
    }

    public boolean isManualMode() {
        return manualMode;
    }

    public void setManualPower(double power) {
        if (!manualMode) throw new IllegalStateException("Not in manual mode");

        double limitedPower = Math.max(-rightServo.getMaxPower(),
                Math.min(rightServo.getMaxPower(), power));
        rightServo.setPower(limitedPower);
        leftServo.setPower(limitedPower);
    }

    public void setTargetRotation(double target) {
        if (!manualMode) {
            rightServo.setTargetRotation(target*GEAR_RATIO);
        }
    }
    public void changeTargetRotation(double delta) {
        if (!manualMode) {
            rightServo.changeTargetRotation(delta*GEAR_RATIO);
        }
    }

    public double getTargetRotation() {
        return rightServo.getTargetRotation()/GEAR_RATIO;
    }

    public double getRotation() {
        return rightServo.getTotalRotation()/GEAR_RATIO;
    }

    public void setMaxPower(double power) {
        rightServo.setMaxPower(power);
    }

    @NonNull
    @SuppressLint("DefaultLocale")
    public String toString(){
        return String.format("%s\nManual Mode: %b\nTotal Rotation w/ Gear ratio %f",
                rightServo.log(),
                manualMode, getRotation());
    }

    @TeleOp(name = "Back Arm Test", group = "test")
    public static class BackArmTest extends LinearOpMode {
        public static int DEGREE_INCREMENT = 15;
        @Override
        public void runOpMode() throws InterruptedException {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            BackArm backArm = new BackArm(hardwareMap);
            GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
            boolean manualMode = false;
            double maxPower = 0.5;

            waitForStart();
            while (!isStopRequested()) {
                gamepads.copyStates();
                backArm.update();

                if(gamepads.isPressed("triangle")){
                    manualMode = !manualMode;
                    backArm.setManualMode(manualMode);
                }

                if(manualMode) {
                    if (gamepads.isHeld(-1,"left_dpad")) backArm.setManualPower(-maxPower);
                    else if (gamepads.isHeld(-1, "right_dpad")) backArm.setManualPower(maxPower);
                    else backArm.setManualPower(0);
                } else {
                    if (gamepads.isPressed("up_dpad")) backArm.changeTargetRotation(DEGREE_INCREMENT);
                    if (gamepads.isPressed("down_dpad")) backArm.changeTargetRotation(-DEGREE_INCREMENT);
                }

                telemetry.addLine(backArm.toString());
                telemetry.update();
            }
        }
    }
}