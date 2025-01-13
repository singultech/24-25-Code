package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Diffy;
import org.firstinspires.ftc.teamcode.utils.GamepadPair;

@TeleOp(name = "Diffy Test", group = "Dev")
public class DiffyTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        Diffy diffy = new Diffy(hardwareMap, false);
        waitForStart();

        while (opModeIsActive()) {
            gamepads.copyStates();
            diffy.update();
            if (gamepads.isHeld(-1, "y")) {
                diffy.setRightPower(1);
            }
            if (gamepads.isHeld(-1, "dpad_up")){
                diffy.setLeftPower(1);
            }
            if (gamepads.isHeld(-1, "a")){
                diffy.setRightPower(-1);
            }
            if (gamepads.isHeld(-1, "dpad_down")){
                diffy.setLeftPower(-1);
            }
            if (!gamepads.isHeld(-1, "y") && !gamepads.isHeld(-1, "a")) {
                diffy.setRightPower(0);
            }
            if (!gamepads.isHeld(-1, "dpad_up") && !gamepads.isHeld(-1, "dpad_down")) {
                diffy.setLeftPower(0);
            }

            telemetry.addData("right total rotation", diffy.getRightTotalRotation());
            telemetry.addData("left total rotation", diffy.getLeftTotalRotation());
            telemetry.update();
        }
    }
}

