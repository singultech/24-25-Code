package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;
import org.firstinspires.ftc.teamcode.subsystems.VertSlidePair;

@TeleOp(name = "Seperate Slide Test", group = "Dev")
public class MoveSlidesSeparate extends LinearOpMode {

    DcMotorEx rightSlide;
    DcMotorEx leftSlide;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        VertSlidePair slides = new VertSlidePair(4100, hardwareMap);


        waitForStart();

        while (opModeIsActive()) {

            if (gamepads.isHeld(1, "dpad_up")) {
                slides.changeTargetPosition("l", 100);
            }
            if (gamepads.isHeld(1, "dpad_down")) {
                slides.changeTargetPosition("l", -100);
            }
            if (gamepads.isHeld(1, "triangle")){
                slides.changeTargetPosition("r", 100);
            }
            if (gamepads.isHeld(1, "cross")){
                slides.changeTargetPosition("r", -100);
            }
            if (gamepads.isPressed(1, "left_bumper")){
                slides.lowerHook("l");
            }
            if (gamepads.isPressed(1, "right_bumper")){
                slides.lowerHook("r");
            }
            gamepads.copyStates();
            telemetry.addLine("Vertical D-pad control to increment left Slide.");
            telemetry.addLine("Vertical face button control to increment right Slide.");
            telemetry.addData("Left Slide Position", slides.getLeftPosition());
            telemetry.addData("Right Slide Position", slides.getRightPosition());
            telemetry.addData("Left Slide Target", slides.getLeftTargetPosition());
            telemetry.addData("Right Slide Target", slides.getRightTargetPosition());
            telemetry.update();
        }
    }
}

