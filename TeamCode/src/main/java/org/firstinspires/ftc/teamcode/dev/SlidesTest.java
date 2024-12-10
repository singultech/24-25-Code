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

import org.firstinspires.ftc.teamcode.utils.SlidePair;

@TeleOp(name = "Slides Test", group = "Dev")
public class SlidesTest extends LinearOpMode {
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    @Override
    public void runOpMode() {
        double slidePower = 1.0;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        SlidePair slides = new SlidePair(leftSlide, rightSlide, 2700, slidePower);


        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.square) {
                slides.resetPosition();
            }
            if (gamepad1.triangle){
                if (slides.isActive()){
                    slides.setPower(0);
                } else{
                    slides.setPower(slidePower);
                }
            }
            if (gamepad1.dpad_up){
                slides.changeTargetPosition(50);
            }
            if (gamepad1.dpad_down){
                slides.changeTargetPosition(-50);
            }
            telemetry.addLine("Use the D-pad to control the slides.");
            telemetry.addLine("Press ▣ to reset the slides position to 0.");
            telemetry.addLine("Press ▲ to toggle the holding motors");
            telemetry.addLine("Current Left Position " + slides.getLeftPosition());
            telemetry.addLine("Current Right Position " + slides.getRightPosition());
            telemetry.addLine("Target Position " + slides.getTargetPosition());
            telemetry.update();
        }
    }
}


