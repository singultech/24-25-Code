/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package org.firstinspires.ftc.teamcode.dev;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Slides Test", group = "Dev")
public class SlidesTest extends LinearOpMode {
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    int currentPosition;
    boolean isActive;
    @Override
    public void runOpMode() {

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setPower(1.0);
        rightSlide.setPower(1.0);
        isActive = true;
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.square) {
                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (gamepad1.triangle){
                if (isActive){
                    leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    leftSlide.setPower(0.0);
                    rightSlide.setPower(0.0);
                    isActive = false;
                } else{
                    leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    leftSlide.setPower(1.0);
                    rightSlide.setPower(1.0);
                    isActive = true;
                }
                }

            }
            if (gamepad1.dpad_up){
                currentPosition += 1;
            }
            if (gamepad1.dpad_down){
                currentPosition -= 1;
            }
            setSlidePositions(currentPosition);
            telemetry.addLine("Use the D-pad to control the slides.");
            telemetry.addLine("Press ▣ to reset the slides position to 0.");
            telemetry.addLine("Press ▲ to toggle the holding motors");
            telemetry.addLine("Current Left Position " + leftSlide.getCurrentPosition());
            telemetry.addLine("Current Right Position " + rightSlide.getCurrentPosition());
            telemetry.addLine("Target Position " + currentPosition);
            telemetry.update();
    }
    public void setSlidePositions(int pos){

        leftSlide.setTargetPosition(pos);
        rightSlide.setTargetPosition(pos);

    }
}


