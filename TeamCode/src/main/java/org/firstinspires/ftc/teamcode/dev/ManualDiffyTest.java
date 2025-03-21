package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Diffy;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;

@TeleOp(name = "Manual Diffy Test", group = "Dev")
public class ManualDiffyTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        Diffy diffy = new Diffy(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            gamepads.copyStates();
            diffy.update();

            double verticalPower = gamepads.joystickValue(2, "left", "y");
            double rotationPower = -gamepads.joystickValue(2, "right", "x");

            double leftPower = (verticalPower + rotationPower) * 0.5;
            double rightPower = (verticalPower - rotationPower) * 0.5;

            diffy.setLeftPower(leftPower);
            diffy.setRightPower(rightPower);

            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Total Rotation", diffy.getRightRotation());
            telemetry.addData("Left Total Rotation", diffy.getLeftRotation());
            telemetry.update();
        }
    }
}
