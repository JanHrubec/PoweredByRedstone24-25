package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MyHardwareMap {
    public DcMotorEx backLeftMotor,backRightMotor, frontLeftMotor,frontRightMotor, leftOdo, middleOdo, rightOdo, armLeftMotor, armRightMotor;
    public Servo clawServo, clawRotServo;

    public MyHardwareMap(HardwareMap map){
        backLeftMotor = map.get(DcMotorEx.class, "motor0");
        backRightMotor = map.get(DcMotorEx.class, "motor1");
        frontLeftMotor = map.get(DcMotorEx.class, "motor2");
        frontRightMotor = map.get(DcMotorEx.class, "motor3");

        leftOdo = map.get(DcMotorEx.class, "motor0");
        rightOdo = map.get(DcMotorEx.class, "motor1");
        middleOdo = map.get(DcMotorEx.class, "motor2");

        clawServo = map.get(Servo.class, "servo0");
        clawRotServo = map.get(Servo.class, "servo1");

        armLeftMotor = map.get(DcMotorEx.class, "motor4");
        armRightMotor = map.get(DcMotorEx.class, "motor5");
    }
}
