package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Driver")
public class DriverOpMode extends OpMode {
    public DcMotorEx backLeftMotor,backRightMotor, frontLeftMotor, frontRightMotor, leftOdo, middleOdo, rightOdo, armLeftMotor, armRightMotor;
    public Servo clawServo, clawRotServo;

    int passedContactsRightOdo = 0;
    int passedContactsLeftOdo = 0;
    int passedContactsMiddleOdo = 0;

    double posX = 0;
    double posY = 0;
    double robotRotation = 0;

    double deadzone = 0.05;

    double clawVal = 0.5;


    @Override
    public void init() {
        MyHardwareMap hMap = new MyHardwareMap(hardwareMap);

        //get motors from hardware map
        backLeftMotor = hMap.backLeftMotor;
        backRightMotor = hMap.backRightMotor;
        frontLeftMotor = hMap.frontLeftMotor;
        frontRightMotor = hMap.frontRightMotor;

        leftOdo = hMap.leftOdo;
        middleOdo = hMap.middleOdo;
        rightOdo = hMap.rightOdo;

        clawServo = hMap.clawServo;
        clawRotServo = hMap.clawRotServo;

        armRightMotor = hMap.armRightMotor;
        armLeftMotor = hMap.armLeftMotor;

        leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        passedContactsRightOdo = rightOdo.getCurrentPosition();
        passedContactsLeftOdo = leftOdo.getCurrentPosition();
        passedContactsMiddleOdo = middleOdo.getCurrentPosition();

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        //Get odo deltas
        int deltaContactsLeftOdo = leftOdo.getCurrentPosition() - passedContactsLeftOdo;
        int deltaContactsRightOdo = rightOdo.getCurrentPosition() - passedContactsRightOdo;
        int deltaContactsMiddleOdo = middleOdo.getCurrentPosition() - passedContactsMiddleOdo;

        //Update passed odo contacts
        passedContactsLeftOdo += deltaContactsLeftOdo;
        passedContactsRightOdo += deltaContactsRightOdo;
        passedContactsMiddleOdo += deltaContactsMiddleOdo;
//
//        //Get position change
//        double[] positionChange = Odometry.getPositionChange(-deltaContactsRightOdo, deltaContactsLeftOdo, -deltaContactsMiddleOdo, robotRotation);
//
//        double deltaX = positionChange[0];
//        double deltaY = positionChange[1];
//        double deltaRotation = positionChange[2];
//
//        //Update position
//        posX += deltaX;
//        posY += deltaY;
//        robotRotation = ToolBox.scaleAngle(robotRotation + deltaRotation);

        //Move robot
        double joystickY = -gamepad1.left_stick_y;
        double joystickX = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if(Math.abs(joystickX) > deadzone || Math.abs(joystickY) > deadzone || Math.abs(rotate) > deadzone) {
            double joystickAngle = Math.atan2(joystickX, joystickY);
            double magnitude = ToolBox.pythagoras(joystickX, joystickY);
            double[] motorPowers = ToolBox.getMotorPowersByDirection(joystickAngle, magnitude, rotate * 0.8);

            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);
        }
        else {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
        }

        // Claw
        if(gamepad2.dpad_left){
            clawVal = 0.8;
        }
        else if(gamepad2.dpad_right){
            clawVal = 0.2;
        }
        clawServo.setPosition(clawVal);

        double clawRotVal = 0.5;
        if(gamepad2.dpad_down){
            clawRotVal = 0.7;
        }
        else if(gamepad2.dpad_up){
            clawRotVal = 0.3;
        }
        clawRotServo.setPosition(clawRotVal);

        //Rotate arm
        double joystick2Y = -gamepad2.right_stick_y;
        if(Math.abs(joystick2Y) > deadzone){
            armRightMotor.setPower(-joystick2Y/3);
            armLeftMotor.setPower(joystick2Y/3);
        }
        else{
            armRightMotor.setPower(0);
            armLeftMotor.setPower(0);
        }


        //output data
        telemetry.addData("Position X", posX);
        telemetry.addData("Position Y", posY);
        telemetry.addData("Robot direction in PI radians", robotRotation/Math.PI);
        telemetry.addData("PassedContactsRightOdo", passedContactsRightOdo);
        telemetry.addData("PassedContactsLeftOdo", passedContactsLeftOdo);
        telemetry.addData("PassedContactsMiddleOdo", passedContactsMiddleOdo);
    }
}
