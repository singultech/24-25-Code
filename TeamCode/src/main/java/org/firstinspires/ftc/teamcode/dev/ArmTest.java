/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.utils.Arm;
import org.firstinspires.ftc.teamcode.utils.FrontGrabber;
import org.firstinspires.ftc.teamcode.utils.GamepadPair;
import org.firstinspires.ftc.teamcode.utils.SlidePair;

@TeleOp(name = "Arm Test", group = "Dev")
public class ArmTest extends LinearOpMode {
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Servo lServo = hardwareMap.servo.get("leftFlip");
        Servo rServo = hardwareMap.servo.get("rightFlip");
        Arm arm = new Arm(1, 0, lServo, rServo);
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            gamepads.copyStates();

            if (gamepads.isPressed(-1, "cross")) {
                if (arm.isUp()) arm.down();
                else arm.up();
            }

            telemetry.addLine("Press X to raise or lower the arm");
            telemetry.addLine(arm.isUp() ? "Arm is up" : "Arm is down");
            telemetry.update();
        }
    }
}


