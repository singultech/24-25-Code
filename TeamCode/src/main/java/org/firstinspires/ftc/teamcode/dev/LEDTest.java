package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BackArm;
import org.firstinspires.ftc.teamcode.subsystems.Diffy;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;
import org.firstinspires.ftc.teamcode.subsystems.Underglow;

@TeleOp(name = "LED Test", group = "Dev")
public class LEDTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        Underglow underglow = new Underglow(hardwareMap, RevBlinkinLedDriver.BlinkinPattern.AQUA);
        waitForStart();

        while (opModeIsActive()) {
            gamepads.copyStates();
            if(gamepads.isPressed(-1, "circle")) underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            if(gamepads.isPressed(-1, "cross")) underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

            telemetry.addData("Pattern", underglow.getPattern());
            telemetry.update();
        }
    }
}
