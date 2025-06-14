package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;

@TeleOp(name = "TestMotorRTP")
public class TestMotorRTP extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "rightHorizSlide");
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setPower(0.25); // Only needs to be set once

        int targetPosition = 0;
        final int STEP = 10;
        final int MIN_POS = 0;
        final int MAX_POS = 2000; // adjust based on your hardware

        waitForStart();

        while (!isStopRequested()) {
            gamepads.copyStates();

            if (gamepads.isPressed("dpad_up")) {
                targetPosition = Math.min(targetPosition + STEP, MAX_POS);
                motor.setTargetPosition(targetPosition);
            } else if (gamepads.isPressed("dpad_down")) {
                targetPosition = Math.max(targetPosition - STEP, MIN_POS);
                motor.setTargetPosition(targetPosition);
            }

            telemetry.addData("Motor Power", motor.getPower());
            telemetry.addData("Motor Target", motor.getTargetPosition());
            telemetry.addData("Motor Current", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
