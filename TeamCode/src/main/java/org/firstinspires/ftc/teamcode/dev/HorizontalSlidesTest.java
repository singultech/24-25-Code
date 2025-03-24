package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;
import org.firstinspires.ftc.teamcode.subsystems.HorizSlidePair;

@TeleOp(name = "Horiz Slides Test", group = "test")
public class HorizontalSlidesTest extends LinearOpMode {
    public static int DEGREE_INCREMENT = 15;
    public static double MAX_MANUAL_POWER = 0.5;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HorizSlidePair slides = new HorizSlidePair(hardwareMap);
        slides.setManualMode(true);
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        boolean manualMode = false;

        waitForStart();

        while (opModeIsActive()) {
            gamepads.copyStates();
            slides.update();
            
            if(gamepads.isPressed("triangle")){
                manualMode = !manualMode;
                slides.setManualMode(manualMode);
            }

            if(manualMode) {
                if (gamepads.isHeld(-1,"left_dpad")) slides.setManualPower(-MAX_MANUAL_POWER);
                else if (gamepads.isHeld(-1,"right_dpad")) slides.setManualPower(MAX_MANUAL_POWER);
                else slides.setManualPower(0);
            } else {
                if (gamepads.isPressed("up_dpad")) slides.changeTargetRotation(DEGREE_INCREMENT);
                if (gamepads.isPressed("down_dpad")) slides.changeTargetRotation(-DEGREE_INCREMENT);
            }

            telemetry.addLine(slides.log());
            telemetry.addData("Manual Mode", manualMode ? "ON" : "OFF");
            telemetry.update();
        }
    }
}
