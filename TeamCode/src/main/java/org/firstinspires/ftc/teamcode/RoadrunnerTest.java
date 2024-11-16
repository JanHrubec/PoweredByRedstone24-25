package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

@Autonomous(name="ROAD RUNNER TEST")
public class RoadrunnerTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        MyHardwareMap hMap = new MyHardwareMap(hardwareMap);
        Pose2d initialPose = new Pose2d(0,0,Math.PI/2);
        MecanumDrive drive = new MecanumDrive(hardwareMap,initialPose);
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(45,45));
        waitForStart();
        Action action = tab1.build();
        Actions.runBlocking(action);
    }
}
