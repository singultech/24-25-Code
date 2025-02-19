package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BackArm;
import org.firstinspires.ftc.teamcode.subsystems.Diffy;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;

@TeleOp(name = "Manual Diffy Test", group = "Dev")
public class ManualDiffyTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        Diffy diffy = new Diffy(hardwareMap, false);
        BackArm arm = new BackArm(hardwareMap, false);
        waitForStart();

        while (opModeIsActive()) {
            gamepads.copyStates();
            arm.update();
            diffy.update();

            double verticalPower = gamepads.joystickValue(2, "left", "y");
            double rotationPower = -gamepads.joystickValue(2, "right", "x");

            double leftPower = (verticalPower + rotationPower) *0.5;
            double rightPower = (verticalPower - rotationPower) *0.5;

            diffy.setLeftPower(leftPower);
            diffy.setRightPower(rightPower);


            if (gamepads.isHeld(-1, "dpad_right")) {
                arm.setPower(1);
            } else if (gamepads.isHeld(-1, "dpad_left")) arm.setPower(-1);
            else arm.setPower(0);

            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Total Rotation", diffy.getRightTotalRotation());
            telemetry.addData("Left Total Rotation", diffy.getLeftTotalRotation());
            telemetry.update();
        }
    }
}
