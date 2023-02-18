package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.Minimap.mode.Teleop;
import static org.firstinspires.ftc.teamcode.Minimap.mode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import com.google.gson.JsonParser;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configs.VariableConfig;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.json.JSONException;
import org.json.JSONObject;
import java.lang.Math;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

//TODO: NOT TESTED, 99% won't work

@TeleOp
public class Minimap extends LinearOpMode{

//    TODO: get current position
//     when pressing A, enter automode
//     select with Y, X, A, B desired junction
//     head to junction and lift glisiera asyncron
//     relenquish control after the glisiera has reached required level and robot is at 0 momentum
    double goToX, goToY, precision = 5, midX, midY, cx, cy;
    double positionX, positionY, squareLength = 23.5;
    double joystick_r_y, joystick_r_x, joystick_l_y, joystick_l_x, xStart=0, yStart=0, headingStart=0;
    int unghi, precisionAngle = 5, startAngle;
    boolean isOpen = false;
    String selectedJ = "none";
    enum mode{
        Teleop,
        Auto
    }
    mode currentMode = mode.Teleop;
    enum traj {
        GoToMid,
        GoToJunction,
        Done
    }
    traj currentTraj = traj.GoToMid;

    hardwarePapiu robot = new hardwarePapiu();

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);
        robot.EncoderReset();
        drive.setPoseEstimate(new Pose2d(-35.5, -61, Math.toRadians(90)));



        waitForStart();

        while(!isStopRequested()) {
            /** variables **/

            joystick_l_y = gamepad2.left_stick_y;
            joystick_r_x = gamepad2.right_stick_x;
            joystick_r_y = gamepad2.right_stick_y;
            joystick_l_x = gamepad2.left_stick_x;
            float joystic_r_trigger = gamepad2.right_trigger + 1;

            positionX = drive.getPoseEstimate().getX();
            positionY = drive.getPoseEstimate().getY();   //-Aici pt coordonate

            //currentTraj = traj.GoToMid;
            drive.update();                               /****************************************************************************/
            telemetry.addData("positionX = ", positionX);
            telemetry.addData("positionY = ", positionY);
            telemetry.update();

            if (positionX > 0) {
                if (positionX < squareLength) {
                    cx = (int) (squareLength / positionX);
                    midX = squareLength / 2;
                } else {
                    cx = (int) (positionX / squareLength);
                    midX = cx * squareLength + squareLength / 2;
                }
            }
            if (positionX < 0) {
                if (positionX < -squareLength) {
                    cx = (int) (positionX / squareLength);
                    midX = cx * squareLength - squareLength / 2;
                } else {
                    cx = (int) (squareLength / positionX);
                    midX = -squareLength / 2;
                }
            }
            if (positionY > 0) {
                if (positionY < squareLength) {
                    cy = (int) (squareLength / positionY);
                    midY = squareLength / 2;
                } else {
                    cy = (int) (positionY / squareLength);
                    midY = cy * squareLength + squareLength / 2;
                }
            }
            if (positionY < 0) {
                if (positionY < -squareLength) {
                    cy = (int) (positionY / squareLength);
                    midY = cy * squareLength - squareLength / 2;
                } else {
                    cy = (int) (squareLength / positionY);
                    midY = -squareLength / 2;
                }
            }
            telemetry.addData("cx = ", cx);
            telemetry.addData("cy = ", cy);
            telemetry.addData("midX = ", midX);
            telemetry.addData("midY = ", midY);
            telemetry.update();
            switch (currentMode) {

                case Teleop:
                    if (joystic_r_trigger > 1) {
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -joystick_l_y / (1.5 * joystic_r_trigger),
                                        -joystick_l_x / (1.5 * joystic_r_trigger),
                                        -joystick_r_x / (5 * joystic_r_trigger)
                                )
                        );
                    } else {
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -joystick_l_y / 1.2,
                                        -joystick_l_x / 1.2,
                                        -joystick_r_x / 1.2
                                )
                        );
                    }
                   // drive.update();             /*****************************************************************************/

                    if (gamepad2.dpad_up)
                        moveBratSus("up");

                    if (gamepad2.dpad_down)
                        moveBratSus("low");

                    if (gamepad2.y)
                        brateCleste();

                    if (gamepad2.dpad_right)
                        moveBratSus("middle");

                    if (gamepad2.dpad_left)
                        moveBratSus("down");

                    if (gamepad1.y) {                 /**up left**/
                        if (positionY < 0) {
                            if (positionY > -squareLength)
                                goToY = 0 + precision;
                            if (positionY < -squareLength)
                                goToY = squareLength * cy - precision;
                        } else {
                            if (positionY < squareLength)
                                goToY = squareLength + precision;
                            if (positionY > squareLength)
                                goToY = squareLength * (cy + 1) + precision;
                        }
                        if (positionX < 0) {
                            if (positionX > -squareLength)
                                goToX = -squareLength + precision;
                            if (positionX < -squareLength)
                                goToX = squareLength * (cx - 1) + precision;
                        } else {
                            if (positionX < squareLength)
                                goToX = 0 + precision;
                            if (positionX > squareLength)
                                goToX = cx * squareLength + precision;
                        }
                        unghi = 135 + precisionAngle;
                        startAngle = -180;
                        currentMode = mode.Auto;
                    }
                    else if (gamepad1.b) {       /**up right**/
                        if (positionY < 0) {
                            if (positionY > -squareLength)
                                goToY = 0 - precision;
                            if (positionY < -squareLength)
                                goToY = squareLength * cy - precision;
                        } else {
                            if (positionY < squareLength)
                                goToY = squareLength + precision;
                            if (positionY > squareLength)
                                goToY = squareLength * (cy + 1) + precision;
                        }
                        if (positionX < 0) {
                            if (positionX > -squareLength)
                                goToX = 0 - precision;
                            if (positionX < -squareLength)
                                goToX = squareLength * cx - precision;
                        } else {
                            if (positionX < squareLength)
                                goToX = squareLength - precision;
                            if (positionX > squareLength)
                                goToX = squareLength * (cx + 1) - precision;
                        }
                        unghi = 45 + precisionAngle;
                        startAngle = 0;
                        currentMode = mode.Auto;
                    }

                    else if (gamepad1.a) {                 /**down right**/
                        if (positionY < 0) {
                            if (positionY > -squareLength)
                                goToY = -squareLength + precision;
                            if (positionY < -squareLength)
                                goToY = squareLength * (cy - 1) + precision;
                        } else {
                            if (positionY < squareLength)
                                goToY = 0 + precision;
                            if (positionY > squareLength)
                                goToY = squareLength * cy + precision;
                        }
                        if (positionX < 0) {
                            if (positionX > -squareLength)
                                goToX = 0 - precision;
                            if (positionX < -squareLength)
                                goToX = squareLength * cx - precision;
                        } else {
                            if (positionX < squareLength)
                                goToX = squareLength - precision;
                            if (positionX > squareLength)
                                goToX = squareLength * (cx + 1) - precision;
                        }
                        unghi = -45 - precisionAngle;
                        startAngle = 0;
                        currentMode = mode.Auto;
                    }
                    else if (gamepad1.x) {             /**down left**/
                        if (positionY < 0) {
                            if (positionY > -squareLength)
                                goToY = -squareLength + precision;
                            if (positionY < -squareLength)
                                goToY = squareLength * (cy - 1) + precision;
                        } else {
                            if (positionY < squareLength)
                                goToY = 0 + precision;
                            if (positionY > squareLength)
                                goToY = squareLength * cy + precision;
                        }
                        if (positionX < 0) {
                            if (positionX > -squareLength)
                                goToX = -squareLength + precision;
                            if (positionX < -squareLength)
                                goToX = squareLength * (cx - 1) + precision;
                        } else {
                            if (positionX < squareLength)
                                goToX = 0 + precision;
                            if (positionX > squareLength)
                                goToX = squareLength * cx + precision;
                        }
                        unghi = -135 - precisionAngle;
                        startAngle = -180;
                        currentMode = mode.Auto;
                    }
                    telemetry.addLine("SUNT IN TELEOP");
                    telemetry.addData("goToX = ", goToX);
                    telemetry.addData("goToY = ", goToY);
                    telemetry.update();

                    if(currentTraj == traj.Done)
                        currentTraj = traj.GoToMid;

                    moveBratSus("down");

                    if(gamepad2.a) {
                        currentMode = mode.Auto;
                        telemetry.addLine("DA");
                    }

                    break;

                case Auto:

                    telemetry.addLine("SUNT IN AUTO");
                    telemetry.addData("gotoX = ", goToX);
                    telemetry.addData("gotoY = ", goToY);
                    telemetry.update();

                    Trajectory GoToMid = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(),
                                    drive.getPoseEstimate().getHeading()))
                            .lineToLinearHeading(new Pose2d(midX, midY, Math.toRadians(startAngle)))
                            .build();

                    Pose2d StartPos = new Pose2d(midX, midY, Math.toRadians(startAngle));
                    Trajectory GoToJunction = drive.trajectoryBuilder(StartPos)
                            .lineToLinearHeading(new Pose2d(goToX, goToY, Math.toRadians(unghi)))   //TODO: scade viteza
                            .addDisplacementMarker(0, () -> {
                                if (goToX == -2 * squareLength + precision || goToX == precision || goToX == 2 * squareLength + precision)
                                    if (goToY == -2 * squareLength + precision || goToY == precision || goToY == 2 * squareLength + precision) {
                                        moveBratSus("down");
                                    }
                                if (goToX == -2 * squareLength + precision && (goToY == -squareLength + precision || goToY == squareLength + precision) || goToX == -squareLength + precision && (goToY == -2 * squareLength + precision || goToY == 2 * squareLength + precision) ||
                                        goToX == squareLength + precision && (goToY == -2 * squareLength + precision || goToY == 2 * squareLength + precision) || goToX == 2 * squareLength + precision && (goToY == -squareLength + precision || goToY == squareLength + precision))
                                    moveBratSus("low");

                                if (goToX == -squareLength + precision || goToX == squareLength + precision)
                                    if (goToY == -squareLength + precision || goToY == squareLength + precision)
                                        moveBratSus("middle");

                                if (goToY == precision && (goToX == -squareLength + precision || goToX == squareLength + precision) || goToX == precision && (goToY == -squareLength + precision || goToY == squareLength + precision)) {
                                    moveBratSus("up");
                                }
                            })
                            .build();

                    if (gamepad2.a) {         //TODO: se poate tot gamepad2.a? ar fi mai usor sa schimbi currentmode ul de pe acelasi buton
                        currentMode = mode.Teleop;
                        telemetry.addLine("TREC IN TELEOP");
                    }

                    while (currentTraj != traj.Done && !isStopRequested()) {
                        telemetry.addLine("INCEP TRAIECTORIA");
                        switch (currentTraj) {
                            case GoToMid:
                                telemetry.addLine("TRAJ MID");
                                //drive.followTrajectory(GoToMid);
                                if(!drive.isBusy()) {
                                    currentTraj = traj.GoToJunction;
                                    drive.followTrajectory(GoToMid);                   //TODO: Async nu merge, dar simplu merge... ?!
                                    telemetry.addData("currentTraj", currentTraj);
                                    telemetry.addLine("AM FOST IN MID");
                                    telemetry.update();
                                }
                                break;

                            case GoToJunction:
                                telemetry.addLine("TRAJ JUNCTION");
                                telemetry.update();
                                //drive.followTrajectory(GoToJunction);
                                if(!drive.isBusy()) {
                                    drive.followTrajectoryAsync(GoToJunction);
                                    telemetry.addLine("PRIMUL IF llllllllllllllllllllllllllll");
                                    telemetry.update();
                                    if (gamepad2.x) {              //TODO: choose your button carefully :)
                                        drive.breakFollowing();
                                        drive.setDrivePower(new Pose2d());
                                        currentMode = mode.Teleop;
                                        telemetry.addLine("BRAKE FOLLOWING");
                                        telemetry.update();
                                    }
                                    currentTraj = traj.Done;
                                    telemetry.addLine("AM FOST LA JUNCTION");
                                    telemetry.update();
                                }
                                telemetry.addData("currentTraj ", currentTraj);
                                break;
                        }
                        break;
                    }
                    //drive.update();                 /*****************************************************************************/
            }


//            if (gamepad1.y) {                 /**up**/
//                if (positionY < 0) {
//                    if (positionY > -squareLength)
//                        goToY = 0 + precision;
//                    if (positionY < -squareLength)q
//                        goToY = squareLength * (cy - 1) + precision;
//                } else {
//                    if (positionY < squareLength)
//                        goToY = squareLength + 3;
//                    if (positionY > squareLength)
//                        goToY = squareLength * (cy + 1) + precision;
//                }
//                if (gamepad1.x) {             /**left**/
//                    if (positionX < 0) {
//                        if (positionX > -squareLength)
//                            goToX = -squareLength + precision;
//                        if (positionX < -squareLength)
//                            goToX = squareLength * (cx - 1) + precision;
//                    } else {
//                        if (positionX < squareLength)
//                            goToX = 0 + precision;
//                        if (positionX > squareLength)
//                            goToX = cx * squareLength + precision;
//                    }
//
//                    Pose2d StartPos = new Pose2d(midX, midY, Math.toRadians(0));
//                    Trajectory GoToJunction = drive.trajectoryBuilder(StartPos)
//                            .lineToLinearHeading(new Pose2d(goToX, goToY, Math.toRadians(135)))
//                            .addDisplacementMarker(0, () -> {
//                                if (goToX == -2 * squareLength + precision || goToX == precision || goToX == 2 * squareLength + precision)
//                                    if (goToY == -2 * squareLength + precision || goToY == precision || goToY == 2 * squareLength + precision)
//                                        moveBratSus("down");
//
//                                if (goToX == -2 * squareLength + precision && (goToY == -squareLength + precision || goToY == squareLength + precision) || goToX == -squareLength + precision && (goToY == -2 * squareLength + precision || goToY == 2 * squareLength + precision) ||
//                                        goToX == squareLength + precision && (goToY == -2 * squareLength + precision || goToY == 2 * squareLength + precision) || goToX == 2 * squareLength + precision && (goToY == -squareLength + precision || goToY == squareLength + precision))
//                                    moveBratSus("low");
//
//                                if (goToX == -squareLength + precision || goToX == squareLength + precision)
//                                    if (goToY == -squareLength + precision || goToY == squareLength + precision)
//                                        moveBratSus("middle");
//
//                                if (goToY == precision && (goToX == -squareLength + precision || goToX == squareLength + precision) || goToX == precision && (goToY == -squareLength + precision || goToY == squareLength + precision))
//                                    moveBratSus("up");
//                            })
//                            .build();
//                    drive.followTrajectory(GoToJunction);
//                }
//                if (gamepad1.b) {             /**right**/
//                    if (positionX < 0) {
//                        if (positionX > -squareLength)
//                            goToX = 0 + precision;
//                        if (positionX < -squareLength)
//                            goToX = squareLength * cx + precision;
//                    } else {
//                        if (positionX < squareLength)
//                            goToX = squareLength + precision;
//                        if (positionX > squareLength)
//                            goToX = squareLength * (cx + 1) + precision;
//                    }
//                    Pose2d StartPos = new Pose2d(midX, midY, Math.toRadians(0));
//                    Trajectory GoToJunction = drive.trajectoryBuilder(StartPos)
//                            .lineToLinearHeading(new Pose2d(goToX, goToY, Math.toRadians(45)))
//                            .addDisplacementMarker(0, () -> {
//                                if (goToX == -2 * squareLength + precision || goToX == precision || goToX == 5 * squareLength + precision)
//                                    if (goToY == -2 * squareLength + precision || goToY == precision || goToY == 2 * squareLength + precision)
//                                        moveBratSus("down");
//
//                                if (goToX == -2 * squareLength + precision && (goToY == -squareLength + precision || goToY == squareLength + precision) || goToX == -squareLength + precision && (goToY == -2 * squareLength + precision || goToY == 2 * squareLength + precision) ||
//                                        goToX == squareLength + precision && (goToY == -2 * squareLength + precision || goToY == 2 * squareLength + precision) || goToX == 2 * squareLength + precision && (goToY == -squareLength + precision || goToY == squareLength + precision))
//                                    moveBratSus("low");
//
//                                if (goToX == -squareLength + precision || goToX == squareLength + precision)
//                                    if (goToY == -squareLength + precision || goToY == squareLength + precision)
//                                        moveBratSus("middle");
//
//                                if (goToY == precision && (goToX == -squareLength + precision || goToX == squareLength + precision) || goToX == precision && (goToY == -squareLength + precision || goToY == squareLength + precision))
//                                    moveBratSus("up");
//                            })
//                            .build();
//                    drive.followTrajectory(GoToJunction);
//                }
//            }
//            if (gamepad1.a) {                 /**down**/
//                if (positionY < 0) {
//                    if (positionY > -squareLength)
//                        goToY = -squareLength + precision;
//                    if (positionY < -squareLength)
//                        goToY = squareLength * (cy - 1) + precision;
//                } else {
//                    if (positionY < squareLength)
//                        goToY = squareLength + precision;
//                    if (positionY > squareLength)
//                        goToY = squareLength * cy + precision;
//                }
//                if (gamepad1.x) {             /**left**/
//                    if (positionX < 0) {
//                        if (positionX > -squareLength)
//                            goToX = -squareLength + precision;
//                        if (positionX < -squareLength)
//                            goToX = squareLength * (cx - 1) + precision;
//                    } else {
//                        if (positionX < squareLength)
//                            goToX = 0 + precision;
//                        if (positionX > squareLength)
//                            goToX = cx * squareLength + precision;
//                    }
//
//                    Pose2d StartPos = new Pose2d(midX, midY, Math.toRadians(0));       // || 270?
//                    drive.setPoseEstimate(StartPos);
//                    Trajectory GoToJunction = drive.trajectoryBuilder(StartPos)
//                            .lineToLinearHeading(new Pose2d(goToX, goToY, Math.toRadians(-135)))
//                            .addDisplacementMarker(0, () -> {
//                                if (goToX == -2 * squareLength + precision || goToX == precision || goToX == 2 * squareLength + precision)
//                                    if (goToY == -2 * squareLength + precision || goToY == precision || goToY == 2 * squareLength + precision)
//                                        moveBratSus("down");
//
//                                if (goToX == -2 * squareLength + precision && (goToY == -squareLength + precision || goToY == squareLength + precision) || goToX == -squareLength + precision && (goToY == -2 * squareLength + precision || goToY == 2 * squareLength + precision) ||
//                                        goToX == squareLength + precision && (goToY == -2 * squareLength + precision || goToY == 2 * squareLength + precision) || goToX == 2 * squareLength + precision && (goToY == -squareLength + precision || goToY == squareLength + precision))
//                                    moveBratSus("low");
//
//                                if (goToX == -squareLength + precision || goToX == squareLength + precision)
//                                    if (goToY == -squareLength + precision || goToY == squareLength + precision)
//                                        moveBratSus("middle");
//
//                                if (goToY == precision && (goToX == -squareLength + precision || goToX == squareLength + precision) || goToX == precision && (goToY == -squareLength + precision || goToY == squareLength + precision))
//                                    moveBratSus("up");
//                            })
//                            .build();
//                    drive.followTrajectory(GoToJunction);
//                }
//                if (gamepad1.b) {             /**right**/
//                    if (positionX < 0) {
//                        if (positionX > -squareLength)
//                            goToX = 0 + precision;
//                        if (positionX < -squareLength)
//                            goToX = squareLength * cx + precision;
//                    } else {
//                        if (positionX < squareLength)
//                            goToX = squareLength + precision;
//                        if (positionX > squareLength)
//                            goToX = squareLength * (cx + 1) + precision;
//                    }
//                    Pose2d StartPos = new Pose2d(midX, midY, Math.toRadians(0));
//                    drive.setPoseEstimate(StartPos);
//                    Trajectory GoToJunction = drive.trajectoryBuilder(StartPos)
//                            .lineToLinearHeading(new Pose2d(goToX, goToY, Math.toRadians(-45)))
//                            .addDisplacementMarker(0, () -> {
//                                if (goToX == -2 * squareLength + precision || goToX == precision || goToX == 2 * squareLength + precision)
//                                    if (goToY == -2 * squareLength + precision || goToY == precision || goToY == 2 * squareLength + precision)
//                                        moveBratSus("down");
//
//                                if (goToX == -2 * squareLength + precision && (goToY == -squareLength + precision || goToY == squareLength + precision) || goToX == -squareLength + precision && (goToY == -2 * squareLength + precision || goToY == 2 * squareLength + precision) ||
//                                        goToX == squareLength + precision && (goToY == -2 * squareLength + precision || goToY == 2 * squareLength + precision) || goToX == 2 * squareLength + precision && (goToY == -squareLength + precision || goToY == squareLength + precision))
//                                    moveBratSus("low");
//
//                                if (goToX == -squareLength + precision || goToX == squareLength + precision)
//                                    if (goToY == -squareLength + precision || goToY == squareLength + precision)
//                                        moveBratSus("middle");
//
//                                if (goToY == precision && (goToX == -squareLength + precision || goToX == squareLength + precision) || goToX == precision && (goToY == -squareLength + precision || goToY == squareLength + precision))
//                                    moveBratSus("up");
//                            })
//                            .build();
//                    drive.followTrajectory(GoToJunction);
//                }
//            }
        }
    }

    double PI = 3.1415;
    double GEAR_MOTOR_40_TICKS = 1120;
    double GEAR_MOTOR_ORBITAL20_TICKS = 537.6;
    double GEAR_MOTOR_GOBILDA5202_TICKS = 537.7;
    double WHEEL_DIAMETER_CM = 4;
    //double TICKS_PER_CM_Z = GEAR_MOTOR_40_TICKS / (WHEEL_DIAMETER_CM * PI);
    double TICKS_PER_CM_Z = GEAR_MOTOR_GOBILDA5202_TICKS / (WHEEL_DIAMETER_CM * PI);

    public void moveBratSus(String direction){
        if(Objects.equals(direction, "up")){
            int ticks = (int)((16+18+24+4) * TICKS_PER_CM_Z);
            robot.bratz.setTargetPosition(-ticks);
            robot.bratz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bratz.setPower(1);
        } else if(Objects.equals(direction, "down")){
            robot.bratz.setTargetPosition(0);
            robot.bratz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bratz.setPower(1);
        } else if(Objects.equals(direction, "middle")){
            int ticks = (int)((18+24) * TICKS_PER_CM_Z);
            robot.bratz.setTargetPosition(-ticks);
            robot.bratz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bratz.setPower(1);
        } else if(Objects.equals(direction, "low")){
            int ticks = (int)(24 * TICKS_PER_CM_Z);
            robot.bratz.setTargetPosition(-ticks);
            robot.bratz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bratz.setPower(0.6);
        }
    }
    public void brateCleste() {
        try {
            isOpen=!isOpen;
            if(isOpen){ //pt deschis
                robot.servoLeft.setPosition(0);
                robot.servoRight.setPosition(0.4);
            }
            else{ //pt inchis
                robot.servoLeft.setPosition(0.07);
                robot.servoRight.setPosition(0.32);
            }
            TimeUnit.MILLISECONDS.sleep(300);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }
    }
}