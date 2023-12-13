package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.RobotConstants.state;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotConstants.LiftLevels;
import org.firstinspires.ftc.teamcode.systems.Brat;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Lift;
import org.firstinspires.ftc.teamcode.systems.Vision;

import java.util.Vector;

@Autonomous(name="Auto Red BB Nando Mocanu numaru 1", group = "advanced")
public class AutoRosu extends LinearOpMode {

    public enum Mod {
        TrajPre,
        Traj1,
        Prep1,
        Extend1,
        Score1,
        Retract0,
        Retract1,
        TrajPre2,
        Traj2,
        Prep2,
        Extend2,
        TrajScore2,
        Score2,
        Retract2,
        Park,
        Idle
    }

    Mod mod = Mod.TrajPre;


    @Override
    public void runOpMode() throws InterruptedException {

        int level=0;

        Intake intake = new Intake(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Brat brat = new Brat(hardwareMap);
        Servo claw = hardwareMap.servo.get("servoClaw");
        Servo joint = hardwareMap.servo.get("angleServo");

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.PI / 2));
        ElapsedTime timer1 = new ElapsedTime();

        telemetry.addData("nu vad camera",0);
        telemetry.update();

        Vision.scanTeamProp(hardwareMap, Vision.RCONE);


        int caz = 2;

        // ------------ DETECTIE ----------------

        while(!isStopRequested() && !isStarted()){
            switch (Vision.TeamPropPipeline.cc) {
                case CENTER:
                    caz=2;
                    break;

                case LEFT:
                    caz=1;
                    break;

                case RIGHT:
                    caz=3;
                    break;
            }
            telemetry.addData("caz: ",caz);
            telemetry.update();
        }

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            switch (level) {
                case 0:
                    state = LiftLevels.Down_position;
                    break;
                case 1:
                    state = LiftLevels.Auto_position;
                    break;
            }

            Lift.liftTarget = state.position;
            Lift.LiftUpdate();

            // ----------------- SWITCH ===========
            switch (mod) {
                case TrajPre:
                    claw.setPosition(0.4);
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(new Vector2d(0, 5), Math.PI / 2)
                                    .build());
                    timer1.reset();
                    mod= Mod.Prep1;
                    break;

                case Prep1:
                    Intake.Drop();
                    Brat.pozitie(0.15);
                    joint.setPosition(0);
                    if(timer1.seconds()>0.5){
                        mod= Mod.Extend1;
                        timer1.reset();
                    }
                    break;

                case Extend1:
                    Brat.pozitie(0.94);
                    joint.setPosition(1);
                    if(timer1.seconds()>0.5){
                        mod= Mod.Traj1;
                        timer1.reset();
                    }
                    break;

                case Traj1:
                    if(caz==3)
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToLinearHeading(new Pose2d(8, 14, Math.toRadians(65)), Math.PI / 2)
                                        .build());
                    if(caz==2)
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(new Vector2d(7, 24.2), Math.PI / 2)
                                        .build());
                    if(caz==1)
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(new Vector2d(1, 16), Math.PI / 2)
                                        .waitSeconds(0.5)
                                        .turnTo(Math.toRadians(140))
                                        .build());
                    timer1.reset();
                    mod= Mod.Score1;
                    break;

                case Score1:
                    if(timer1.seconds()>0.5)
                        claw.setPosition(0.6);
                    if(timer1.seconds()>1){
                        mod= Mod.Retract0;
                        timer1.reset();
                    }
                    break;

                case Retract0:
                    level=1;
                    joint.setPosition(0.95);
                    if(timer1.seconds()>1){
                        timer1.reset();
                        mod= Mod.Retract1;
                    }
                    break;

                case Retract1:
                    level=0;
                    Brat.pozitie(0.5);
                    joint.setPosition(0.55);
                    if(timer1.seconds()>1){
                        timer1.reset();
                        mod= Mod.TrajPre2;
                    }
                    break;

                case TrajPre2:
                    if(caz==3)
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .turnTo(0)
                                        .waitSeconds(1)
                                        .build());
                    else if(caz==2)
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToLinearHeading(new Pose2d(30, 25.5,0), Math.PI / 2)
                                        .waitSeconds(1)
                                        .build());
                    else
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToLinearHeading(new Pose2d(15, 20,0), Math.PI / 2)
                                        .waitSeconds(1)
                                        .build());
                    mod= Mod.Traj2;
                    break;


                case Traj2:
                    if(caz==3)
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToLinearHeading(new Pose2d(39, 20.5, 0), Math.PI / 2)
                                    .build());
                    else if(caz==2)
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(new Vector2d(40, 26), Math.PI / 2)
                                        .waitSeconds(1)
                                        .turnTo(0)
                                        .build());
                    else
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(new Vector2d(39, 33), Math.PI / 2)
                                        .build());
                    timer1.reset();
                    mod= Mod.Prep2;
                    break;

                case Prep2:
                    level = 1;
                    Brat.pozitie(0.64);
                    joint.setPosition(1);
                    if(timer1.seconds()>0.5){
                        timer1.reset();
                        mod= Mod.Extend2;
                    }
                    break;

                case Extend2:
                    level = 1;
                    Brat.pozitie(0.84);
                    joint.setPosition(0.98);
                    if(timer1.seconds()>1){
                        timer1.reset();
                        mod= Mod.Score2;
                    }
                    break;

                case Score2:
                    claw.setPosition(0.5);
                    if(timer1.seconds()>0.5){
                        timer1.reset();
                        mod= Mod.Retract2;
                    }
                    break;

                case Retract2:
                    level = 0;
                    Brat.pozitie(0.54);
                    joint.setPosition(0.55);
                    mod= Mod.Park;
                    break;

                case Park:
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(new Vector2d(32,0),Math.PI)
                                    .build());
                    mod= Mod.Idle;
                    break;

                case Idle:
                    if(caz==1){
                        Brat.pozitie(0.8);
                    }
                    break;
            }
        }
    }
}