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
        CRServo lServo = hmap.crservo.get("leftDiffy");
        CRServo rServo = hmap.crservo.get("rightDiffy");
        lServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftServo = new RTPAxon(lServo, hmap.get(AnalogInput.class, "leftDiffyEncoder"), RTPAxon.Direction.REVERSE);
        rightServo = new RTPAxon(rServo, hmap.get(AnalogInput.class, "rightDiffyEncoder"));
//        leftServo.setK(0.03);
//        rightServo.setK(0.03);
        grabberRoll = 0;
        grabberPitch = 0;
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

    public void setTargetRotation(double target) {
        setTargetRotation(Side.LEFT, target);
        setTargetRotation(Side.RIGHT, target);
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
    public void changeTargetRotation(double amount) {
        changeTargetRotation(Side.LEFT, amount);
        changeTargetRotation(Side.RIGHT, amount);
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
            changeTargetRotation(degrees, degrees);
        }
    }


    public double getGrabberPitch() {
        return grabberPitch;
    }

    public boolean isAtTarget() {
        return leftServo.isAtTarget() && rightServo.isAtTarget();
    }

    @SuppressLint("DefaultLocale")
    public String log() {
        return String.format("Left Servo: %s\nRight Servo: %s\nGrabber Roll: %f\nGrabber Pitch: %f",
                leftServo.log(),
                rightServo.log(),
                grabberRoll,
                grabberPitch);
    }

    @TeleOp(name = "Diffy Test", group = "test")
    public static class DiffyTest extends LinearOpMode {

        public static double MANUAL_POWER_MULTIPLIER = 0.3;
        public static int DEGREE_INCREMENT = 15;
        @Override
        public void runOpMode() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
            Diffy diffy = new Diffy(hardwareMap);
            diffy.setManualMode(true);
            waitForStart();

            while (opModeIsActive()) {
                gamepads.copyStates();


                if(gamepads.isPressed("triangle")) diffy.setManualMode(!diffy.isManualMode());

                if(diffy.isManualMode()) {
                    double verticalPower = gamepads.joystickValue(2, "left", "y");
                    double rotationPower = -gamepads.joystickValue(2, "right", "x");

                    double leftPower = (verticalPower + rotationPower) * MANUAL_POWER_MULTIPLIER;
                    double rightPower = (verticalPower - rotationPower) * MANUAL_POWER_MULTIPLIER;

                    diffy.setLeftPower(leftPower);
                    diffy.setRightPower(rightPower);
                }
                else {
                    if(gamepads.isPressed("dpad_right")) diffy.changeTargetRotation(Diffy.Side.LEFT, DEGREE_INCREMENT);
                    if(gamepads.isPressed("dpad_left")) diffy.changeTargetRotation(Diffy.Side.LEFT, -DEGREE_INCREMENT);
                    if(gamepads.isPressed("circle")) diffy.changeTargetRotation(Diffy.Side.RIGHT, DEGREE_INCREMENT);
                    if(gamepads.isPressed("square")) diffy.changeTargetRotation(Diffy.Side.RIGHT, -DEGREE_INCREMENT);
                }

                telemetry.addLine(diffy.log());
                telemetry.addLine(diffy.isManualMode() ? "Manual Mode" : "Auto Mode(Run to position)");
                telemetry.update();
                diffy.update();
            }
        }
    }
}
