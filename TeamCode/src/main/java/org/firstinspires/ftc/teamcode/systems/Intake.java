package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Intake {
    public static DcMotor motorIntake;
    ///public static CRServo servoContinuuIntake;
    public static Servo servoPozitieIntake;
    public static final double mIntakePower = -0.8;
    public static final double sIntakePower = -0.3;
    public static final double PozIntakeRaise = 0.30;
    public static final double PozIntakeDrop = 0.50; ///maybe

    public static final double PozIntakeDown = 0.58; ///maybe


    public Intake(HardwareMap hardwareMap) {
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ///servoContinuuIntake = hardwareMap.get(CRServo.class, "servoContinuuIntake");
        servoPozitieIntake = hardwareMap.get(Servo.class, "servoPozitieIntake");
    }

    static public void Raise() {
        servoPozitieIntake.setPosition(PozIntakeRaise);
    }

    static public void Drop() {
        servoPozitieIntake.setPosition(PozIntakeDrop);
    }

    static public void Down() {
        servoPozitieIntake.setPosition(PozIntakeDown);
    }


    static public void On() {
        motorIntake.setPower(mIntakePower);
        ///servoContinuuIntake.setPower(sIntakePower);
    }

    static public void Off() {
        motorIntake.setPower(0);
        ///servoContinuuIntake.setPower(0);
    }

    static public void Reverse() {
        motorIntake.setPower(-mIntakePower);
        ///servoContinuuIntake.setPower(0);
    }

}