package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Diffy;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;

@TeleOp(name = "Preset Diffy Pos'", group = "Dev")
public class PresetDiffyPositions extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        Diffy diffy = new Diffy(hardwareMap, true);
        waitForStart();

        while (opModeIsActive()) {
            gamepads.copyStates();
            diffy.update();

            if (gamepads.isPressed(-1, "cross")){
                diffy.setLeftTargetRotation(diffy.getLeftTargetRotation()+5);
            }

            telemetry.addData("Left Target", diffy.getLeftTargetRotation());
            telemetry.addData("Left Current", diffy.getLeftTotalRotation());
            telemetry.addData("Right Target", diffy.getRightTargetRotation());
            telemetry.addData("Right Current", diffy.getRightTotalRotation());
            telemetry.update();
        }
    }
}
