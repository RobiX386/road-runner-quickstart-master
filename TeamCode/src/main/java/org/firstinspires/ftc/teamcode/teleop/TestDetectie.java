package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Vision;

@TeleOp(name="Testare Detectie", group = "advanced")
public class TestDetectie extends LinearOpMode {
    @Override
    public void runOpMode() {
        Vision.scanTeamProp(hardwareMap, Vision.BCONE);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Camera case", Vision.TeamPropPipeline.cc);
            telemetry.update();
            sleep(50);
        }
    }
}