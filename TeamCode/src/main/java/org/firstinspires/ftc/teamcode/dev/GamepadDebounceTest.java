package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;

@TeleOp(name = "Debounce Test", group = "Dev")
public class GamepadDebounceTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        int dpadupcount = 0;
        int dpaddowncount = 0;
        int circlecount = 0;
        waitForStart();

        while (opModeIsActive()) {
            gamepads.copyStates();
            if (gamepads.isPressed(-1, "dpad_up")) dpadupcount++;
            if (gamepads.isPressed(-1, "dpad_down")) dpaddowncount++;
            if (gamepads.isPressed(-1, "circle")) circlecount++;


            telemetry.addData("Dpad Up Count", dpadupcount);
            telemetry.addData("Dpad Down Count", dpaddowncount);
            telemetry.addData("Circle Count", circlecount);
            telemetry.update();
        }
    }
}

