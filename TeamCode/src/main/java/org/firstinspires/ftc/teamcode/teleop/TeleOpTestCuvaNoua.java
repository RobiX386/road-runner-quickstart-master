package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.RobotConstants.state;
import static org.firstinspires.ftc.teamcode.RobotConstants.ArmLevels;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotConstants.LiftLevels;
import org.firstinspires.ftc.teamcode.systems.Brat;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Lift;

@TeleOp(name="TeleOp Mocanu Cuva Noua", group = "advanced")
public class TeleOpTestCuvaNoua extends LinearOpMode {

    boolean fieldcentric=false;

    BNO055IMU imu;

    int level=0;
    int lvlCleste = 0;
    int lvlBrat = 2;
    int lvlGlis = 0;

    int aranj=0;

    int pozintake=0;

    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        ElapsedTime timer1 = new ElapsedTime();

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftRear");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightRear");

        Servo claw = hardwareMap.servo.get("servoClaw");
        Servo joint = hardwareMap.servo.get("angleServo");

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake intake = new Intake(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Brat brat = new Brat(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        int oki=0;

        Brat.pozitie(0.54);
        joint.setPosition(0.5);
        Intake.Drop();


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            switch (level) {
                case 0:
                    state = LiftLevels.Down_position;
                    break;
                case 1:
                    state = LiftLevels.First_position;
                    break;
                case 2:
                    state = LiftLevels.Second_position;
                    break;
                case 3:
                    state = LiftLevels.Third_position;
                    break;
                case 4:
                    state = LiftLevels.Fourth_position;
                    break;
                case 5:
                    state = LiftLevels.Fifth_position;
                    break;
                case 6:
                    state = LiftLevels.Sixth_position;
                    break;
                case 7:
                    state = LiftLevels.Seventh_position;
                    break;
                case 8:
                    state = LiftLevels.Eight_position;
                    break;
            }

            Lift.liftTarget = state.position;

            Lift.LiftUpdate();

            double liftPosition=Lift.glisiera1.getCurrentPosition();
            double liftTarget=Lift.liftTarget;

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = Math.toRadians(-imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            if (!fieldcentric) {
                rotX = x;
                rotY = y;
            }

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            //INTAKE
            if (lvlBrat>0) {Intake.Off();}
            if (gamepad1.a) {Intake.On();}
            //if (gamepad1.b) {Intake.Off();}
            if (gamepad1.y) {Intake.Reverse();}


            //BRAT
            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) if(lvlBrat<4) lvlBrat++;
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) if(lvlBrat>0) lvlBrat--;

            if(gamepad1.share){
                aranj=1;
            }

            if(lvlBrat==3) {
                Brat.pozitie(0.84);
                if(aranj==0)
                    joint.setPosition(1);
                if(aranj==1)
                    joint.setPosition(0.83);
            } ///pozitie brat outtake
            if(lvlBrat==2) {Brat.pozitie(0.54);joint.setPosition(0.55);} ///pozitie intermediara
            if(lvlBrat==1) {Brat.pozitie(0.15);joint.setPosition(0.0);lvlCleste=2;} ///pozitie brat putin mai sus de intake (preintake)
            if(lvlBrat==0) {Brat.pozitie(0.04);joint.setPosition(0.01); lvlCleste=0;} ///pozitie intake finala

            //CLESTE
            if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                if(lvlCleste!=0) lvlCleste--;
            }

            if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                if(lvlCleste==0) lvlCleste=2;
                else lvlCleste=0;
            }

            if(lvlCleste==0) claw.setPosition(0.5);
            if(lvlCleste==1) claw.setPosition(0.60);
            if(lvlCleste==2) claw.setPosition(0.40);

            if(gamepad1.square){
                oki=1;
            }
            if(oki==1) {
                level = lvlGlis;
                lvlBrat = 3;
            }

            if(gamepad1.b){
                level=0;
                lvlBrat=1;
                aranj=0;
                oki=0;
            }

            if(gamepad1.right_stick_button && oki==0){
                level=0;
                lvlBrat=0;
            }

            if(currentGamepad1.start && !previousGamepad1.start){
                if(pozintake==1)
                    pozintake=0;
                else
                    pozintake=1;
            }

            if(pozintake==1)
                Intake.Raise();
            else
                Intake.Down();

            //GLISIERE
            if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up) if(lvlGlis<4) lvlGlis++;
           // if(gamepad1.dpad_down) {level=0;lvlBrat=1;}
            if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down) if(lvlGlis>0) lvlGlis--;

            if(currentGamepad1.right_trigger>0.1 && previousGamepad1.right_trigger<0.1) if(lvlGlis<8) lvlGlis++;
            // if(gamepad1.dpad_down) {level=0;lvlBrat=1;}
            if(currentGamepad1.left_trigger>0.1 && previousGamepad1.left_trigger<0.1) if(lvlGlis>0) lvlGlis--;
            telemetry.addData("level",level);
            telemetry.addData("lvlBrat",lvlBrat);
            telemetry.addData("lvlCleste",lvlCleste);
            telemetry.update();

            dashboardTelemetry.addData("Lift Target: ", liftTarget);
            dashboardTelemetry.addData("Lift Position :", liftPosition);
            dashboardTelemetry.update();
        }
    }
}