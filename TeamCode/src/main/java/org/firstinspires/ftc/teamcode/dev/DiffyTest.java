package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Diffy;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;

@TeleOp(name = "Diffy Test", group = "test")
public class DiffyTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        Diffy diffy = new Diffy(hardwareMap);
        diffy.setManualMode(true);
        waitForStart();

        while (opModeIsActive()) {
            gamepads.copyStates();
            diffy.update();
            if(diffy.isManualMode()) {
                double verticalPower = gamepads.joystickValue(2, "left", "y");
                double rotationPower = -gamepads.joystickValue(2, "right", "x");

                double leftPower = (verticalPower + rotationPower) * 0.5;
                double rightPower = (verticalPower - rotationPower) * 0.5;

                diffy.setLeftPower(leftPower);
                diffy.setRightPower(rightPower);
            }
            else {
                if(gamepads.isPressed("dpad_right")) diffy.change
            }

            telemetry.addLine(diffy.log());
            telemetry.update();
        }
    }
}
