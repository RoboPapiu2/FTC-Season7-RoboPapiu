package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.TimeUnit;

@TeleOp
public class clesteTest extends LinearOpMode {
    hardwarePapiu robot = new hardwarePapiu();
    double incrementPositive=0, incrementNegative=0;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);
        robot.servoRight.setPosition(0);
        robot.servoLeft.setPosition(0);
        waitForStart();
        while(!isStopRequested()){
            if(gamepad2.x)
                incrementPlus("left");
            if(gamepad2.b)
                incrementPlus("right");
            if(gamepad2.a)
                changeIncrement();

            telemetry.addData("left position: ", robot.servoLeft.getPosition());
            telemetry.addData("right position: ", robot.servoRight.getPosition());
            telemetry.addData("increment: ", incrementPositive);
            telemetry.update();
        }
    }
    public void incrementPlus(String which){
        try {
            if(which == "left"){
                robot.servoLeft.setPosition(robot.servoLeft.getPosition()+incrementPositive);
            } else if (which == "right"){
                robot.servoRight.setPosition(robot.servoRight.getPosition()+incrementPositive);
            }
            TimeUnit.MILLISECONDS.sleep(300);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }
    }
    public void changeIncrement(){
        try {
            if(incrementPositive>0.5){
                incrementPositive=-0.5;
            }
            incrementPositive+=0.1;
            TimeUnit.MILLISECONDS.sleep(300);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }
    }
}
