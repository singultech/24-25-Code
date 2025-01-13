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

import org.firstinspires.ftc.teamcode.utils.FrontArm;
import org.firstinspires.ftc.teamcode.utils.GamepadPair;

@TeleOp(name = "Arm Test", group = "Dev")
public class FrontArmTest extends LinearOpMode {
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FrontArm arm = new FrontArm(1, 0.35, hardwareMap);
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            gamepads.copyStates();

            if (gamepads.isPressed(-1, "cross")) {
                if (arm.isForward()) arm.back();
                else arm.forward();
            }

            telemetry.addLine("Press X to raise or lower the arm");
            telemetry.addLine(arm.isForward() ? "Arm is forward" : "Arm is back");
            telemetry.update();
        }
    }
}

