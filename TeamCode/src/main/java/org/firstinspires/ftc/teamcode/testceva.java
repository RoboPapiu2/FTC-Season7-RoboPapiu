package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@TeleOp
public class testceva extends LinearOpMode
{
    hardwarePapiu robot = new hardwarePapiu();
    public static double MOTOR_POWER = 0.7;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);

        telemetry.addLine("Press play to begin the debugging opmode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        while (!isStopRequested()) {
            telemetry.addLine("Press each button to turn on its respective motor");
            telemetry.addLine();
            telemetry.addLine("<font face=\"monospace\">Xbox/PS4 Button - Motor</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;X / ▢&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Left</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;Y / Δ&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Right</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;B / O&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Right</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;A / X&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Left</font>");
            telemetry.addLine();

            if(gamepad1.x) {
//                drive.setMotorPowers(MOTOR_POWER, 0, 0, 0);
                robot.leftFront.setPower(MOTOR_POWER);
                telemetry.addData("power lf", robot.leftFront.getCurrentPosition());
                telemetry.addLine("Running Motor: Front Left");
            } else if(gamepad1.y) {
//                drive.setMotorPowers(0, 0, 0, MOTOR_POWER);
                robot.rightFront.setPower(MOTOR_POWER);
                telemetry.addData("power fr", robot.rightFront.getCurrentPosition());
                telemetry.addLine("Running Motor: Front Right");
            } else if(gamepad1.b) {
//                drive.setMotorPowers(0, 0, MOTOR_POWER, 0);
                robot.rightRear.setPower(MOTOR_POWER);
                telemetry.addData("power rr", robot.rightRear.getCurrentPosition());
                telemetry.addLine("Running Motor: Rear Right");
            } else if(gamepad1.a) {
//                drive.setMotorPowers(0, MOTOR_POWER, 0, 0);
                robot.leftRear.setPower(MOTOR_POWER);
                telemetry.addData("power lr", robot.leftRear.getCurrentPosition());
                telemetry.addLine("Running Motor: Rear Left");
            } else {
                robot.leftFront.setPower(0);
                robot.leftRear.setPower(0);
                robot.rightRear.setPower(0);
                robot.rightFront.setPower(0);
                telemetry.addLine("Running Motor: None");
            }

            telemetry.update();
        }
    }
}
