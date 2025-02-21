package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BackArm;
import org.firstinspires.ftc.teamcode.subsystems.Diffy;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;
import org.firstinspires.ftc.teamcode.subsystems.RumbleEffects;

@TeleOp(name = "Preset Diffy Pos'", group = "Dev")
public class PresetDiffyPositions extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        Diffy diffy = new Diffy(hardwareMap, true);
        BackArm arm = new BackArm(hardwareMap, true);
        waitForStart();

        while (opModeIsActive()) {
            gamepads.copyStates();
            diffy.update();
            arm.update();

            if (gamepads.isPressed(-1, "cross")){
                new Thread(() -> {
                    arm.setTargetRotation(-290);
                    while(Math.abs(arm.getTargetRotation()-arm.getPosition())>10){
                        sleep(10);
                    }
                    diffy.setLeftTargetRotation(100);
                    diffy.setRightTargetRotation(100);
                    while(Math.abs(diffy.getLeftTargetRotation()-diffy.getLeftTotalRotation())>15){
                        sleep(10);
                    }
                    arm.setTargetRotation(-573);
                    while(Math.abs(arm.getTargetRotation()-arm.getPosition())>10){
                        sleep(10);
                    }
                    diffy.setLeftTargetRotation(280);
                    diffy.setRightTargetRotation(280);
                }).start();
            }
            if (gamepads.isPressed(-1, "circle")){
                new Thread(() -> {
                    diffy.setLeftTargetRotation(100);
                    diffy.setRightTargetRotation(100);
                    while(Math.abs(diffy.getLeftTargetRotation()-diffy.getLeftTotalRotation())>15){
                        sleep(10);
                    }
                    arm.setTargetRotation(-290);
                    while(Math.abs(arm.getTargetRotation()-arm.getPosition())>10){
                        sleep(10);
                    }
                    diffy.setLeftTargetRotation(0);
                    diffy.setRightTargetRotation(0);
                    while(Math.abs(diffy.getLeftTargetRotation()-diffy.getLeftTotalRotation())>15){
                        sleep(10);
                    }
                    arm.setTargetRotation(0);
                }).start();
            }

            telemetry.addData("Left Target", diffy.getLeftTargetRotation());
            telemetry.addData("Left Current", diffy.getLeftTotalRotation());
            telemetry.addData("Right Target", diffy.getRightTargetRotation());
            telemetry.addData("Right Current", diffy.getRightTotalRotation());
            telemetry.addData("Arm Target", arm.getTargetRotation());
            telemetry.addData("Arm Current", arm.getPosition());
            telemetry.addData("Grabber twist", diffy.getGrabberTwist());
            telemetry.addData("Grabber rotation", diffy.getGrabberRotation());
            telemetry.update();
        }
    }
}
