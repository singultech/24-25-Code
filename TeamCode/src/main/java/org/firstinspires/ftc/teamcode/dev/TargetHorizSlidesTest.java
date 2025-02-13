package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;
import org.firstinspires.ftc.teamcode.subsystems.HorizSlidePair;

@TeleOp(name = "Horiz Slides Test - Target", group = "Dev")
public class TargetHorizSlidesTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HorizSlidePair slides = new HorizSlidePair(hardwareMap, true);
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            gamepads.copyStates();

            if(gamepads.isPressed(-1, "dpad_up")){
                slides.setTargetRotation(slides.getTargetRotation()+50);
            }
            if(gamepads.isPressed(-1, "dpad_down")){
                slides.setTargetRotation(slides.getTargetRotation()-50);
            }
            slides.update();

            telemetry.addLine("Use the triggers to control the slides");
            telemetry.addLine("Full Rotation " + slides.getTotalRotation());
            telemetry.addLine("Target: " + slides.getTargetRotation());
            telemetry.update();
        }
    }
}

