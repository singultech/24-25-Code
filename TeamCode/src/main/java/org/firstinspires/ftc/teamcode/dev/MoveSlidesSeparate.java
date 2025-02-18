package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;

@TeleOp(name = "MoveSlidesSeparate", group = "Dev")
public class MoveSlidesSeparate extends LinearOpMode {

    DcMotorEx rightSlide;
    DcMotorEx leftSlide;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        int maxHeight = 3000;
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(1.0);
        rightSlide.setPower(1.0);


        waitForStart();

        while (opModeIsActive()) {

            if (gamepads.isPressed(-1, "dpad_up") && leftSlide.getCurrentPosition() < maxHeight) {
                leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + 100);
            }
            if (gamepads.isPressed(-1, "dpad_down") && (leftSlide.getCurrentPosition()-100) >= 0) {
                leftSlide.setTargetPosition(leftSlide.getCurrentPosition() - 100);
            }
            if (gamepads.isPressed(-1, "triangle") && rightSlide.getCurrentPosition() < maxHeight){
                rightSlide.setTargetPosition(rightSlide.getCurrentPosition() + 100);
            }
            if (gamepads.isPressed(-1, "cross") && (rightSlide.getCurrentPosition()-100) >= 0){
                rightSlide.setTargetPosition(rightSlide.getCurrentPosition() - 100);
            }
            gamepads.copyStates();
            telemetry.addLine("Vertical D-pad control to increment left Slide.");
            telemetry.addLine("Vert D-pad control to increment right Slide.");
            telemetry.addData("Left Slide Position", leftSlide.getCurrentPosition());
            telemetry.addData("Right Slide Position", rightSlide.getCurrentPosition());
            telemetry.update();
        }
    }
}

