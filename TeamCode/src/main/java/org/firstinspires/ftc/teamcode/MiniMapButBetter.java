package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.Objects;
import java.util.concurrent.TimeUnit;
@TeleOp
public class MiniMapButBetter extends LinearOpMode {
    hardwarePapiu robot = new hardwarePapiu();

    //Global values.
    public static double DRAWING_TARGET_RADIUS = 2;
    boolean isMoving=false;
    boolean isOpen=true;
    double joystick_r_y, joystick_r_x, joystick_l_y, joystick_l_x;
    double positionX, positionY, squareLength = 23.5, goToX, goToY, cx, cy, precision=3;
    int liftCountZ=1;

    enum MODE {
        TELEOP,
        AUTO
    }
    MODE currentMode = MODE.TELEOP;

    /** Selected junction position **/
    Vector2d targetPosition = new Vector2d(0, 0);

    // or HEADING_PID
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.getLocalizer().setPoseEstimate(TransferPose.currentPose);
        headingController.setInputBounds(-Math.PI, Math.PI);


        robot.init(hardwareMap);
        robot.EncoderReset();

        Pose2d StartPose = TransferPose.currentPose;
        drive.setPoseEstimate(StartPose);


        /** Default motor values / Reset encoders **/
        robot.bratz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bratz.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.servoLeft.setPosition(0);
        robot.servoRight.setPosition(0.4);

        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            // Read pose
            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();
            positionX = poseEstimate.getX();
            positionY = poseEstimate.getY();


            Pose2d driveDirection = new Pose2d();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            /** variables **/

            joystick_l_y = gamepad2.left_stick_y;
            joystick_r_x = gamepad2.right_stick_x;
            joystick_r_y = gamepad2.right_stick_y;
            joystick_l_x = gamepad2.left_stick_x;
            float joystick_r_trigger = gamepad2.right_trigger+1;
            isMoving= joystick_l_x != 0 && joystick_r_y != 0 && joystick_l_y != 0 && joystick_r_x != 0;
            /** Base Movement**/

            switch (currentMode) {
                case TELEOP:

                    // Switch to AUTO, even if second driver initiates a command
                    if(gamepad2.a)
                        currentMode = MODE.AUTO;
                    if(gamepad1.a){
                        setTargetJunction("BotRight");
                        currentMode = MODE.AUTO;
                    }
                    if(gamepad1.b){
                        setTargetJunction("TopRight");
                        currentMode = MODE.AUTO;
                    }
                    if(gamepad1.x){
                        setTargetJunction("BotLeft");
                        currentMode = MODE.AUTO;
                    }
                    if(gamepad1.y){
                        setTargetJunction("TopLeft");
                        currentMode = MODE.AUTO;

                    }
                    if(gamepad2.dpad_up)
                        moveBrat("up");

                    if(gamepad2.dpad_down)
                        moveBrat("down");

                    if(gamepad2.y)
                        brateCleste();

                    // If trigger is pressed, give slower controls for extra precision
                    if(joystick_r_trigger>1) {
                        driveDirection = new Pose2d(
                                -joystick_l_y/(1.5*joystick_r_trigger),
                                -joystick_l_x/(1.5*joystick_r_trigger),
                                -joystick_r_x/(5*joystick_r_trigger)
                        );
                    }
                    else {
                        driveDirection = new Pose2d(
                                -joystick_l_y/1.2,
                                -joystick_l_x/1.2,
                                -joystick_r_x/1.2
                        );
                    }

                    break;
                case AUTO:

                    // Switch back to TELEOP
                    if(gamepad2.b)
                        currentMode = MODE.TELEOP;

                    if(gamepad2.dpad_up)
                        moveBrat("up");

                    if(gamepad2.dpad_down)
                        moveBrat("down");

                    if(gamepad2.y)
                        brateCleste();

                    //Junction commands
                    if(gamepad1.y) setTargetJunction("TopLeft");
                    if(gamepad1.b) setTargetJunction("TopRight");
                    if(gamepad1.x) setTargetJunction("BotLeft");
                    if(gamepad1.a) setTargetJunction("BotRight");

                    Vector2d fieldFrameInput;

                    if(joystick_r_trigger>1) {
                        fieldFrameInput = new Vector2d(
                                -joystick_l_y/(1.5*joystick_r_trigger),
                                -joystick_l_x/(1.5*joystick_r_trigger)
                        );
                    }
                    else{
                        fieldFrameInput = new Vector2d(
                                -joystick_l_y/1.2,
                                -joystick_l_x/1.2
                        );
                    }
                    Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                    // Difference between the target vector and the bot's position
                    Vector2d difference = targetPosition.minus(poseEstimate.vec());
                    // Obtain the target angle for feedback and derivative for feedforward
                    double theta = difference.angle();

                    // Not technically omega because its power. This is the derivative of atan2
                    double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                    // Set the target heading for the heading controller to our desired angle
                    headingController.setTargetPosition(theta);

                    // Set desired angular velocity to the heading controller output + angular
                    // velocity feedforward
                    double headingInput = (headingController.update(poseEstimate.getHeading())
                            * DriveConstants.kV + thetaFF)
                            * DriveConstants.TRACK_WIDTH;

                    // Combine the field centric x/y velocity with our derived angular velocity
                    driveDirection = new Pose2d(
                            robotFrameInput,
                            headingInput
                    );

                    // Draw the target on the field
                    fieldOverlay.setStroke("#dd2c00");
                    fieldOverlay.strokeCircle(targetPosition.getX(), targetPosition.getY(), DRAWING_TARGET_RADIUS);

                    // Draw lines to target
                    fieldOverlay.setStroke("#b89eff");
                    fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
                    fieldOverlay.setStroke("#ffce7a");
                    fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), targetPosition.getX(), poseEstimate.getY());
                    fieldOverlay.strokeLine(targetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());
                    break;
            }

            // Draw bot on canvas
            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate);

            drive.setWeightedDrivePower(driveDirection);

            // Update the heading controller with our current heading
            headingController.update(poseEstimate.getHeading());

            // Update the localizer
            drive.getLocalizer().update();

            // Send telemetry packet off to dashboard
            FtcDashboard.getInstance().sendTelemetryPacket(packet);


            telemetry.addData("mode", currentMode);
            telemetry.addData("lift level bratz: ", liftCountZ);
            telemetry.update();
        }
    }
    /** Varibile pentru facut automatizare la teleop */

    double PI = 3.1415;
    double GEAR_MOTOR_40_TICKS = 1120;
    double GEAR_MOTOR_ORBITAL20_TICKS = 537.6;
    double GEAR_MOTOR_GOBILDA5202_TICKS = 537.7;
    double WHEEL_DIAMETER_CM = 4;
    //double TICKS_PER_CM_Z = GEAR_MOTOR_40_TICKS / (WHEEL_DIAMETER_CM * PI);
    double TICKS_PER_CM_Z = GEAR_MOTOR_GOBILDA5202_TICKS / (WHEEL_DIAMETER_CM * PI);

    public void setTargetJunction(String junction){
        goToX = 0; goToY = 0;

        if(positionX < squareLength)
            cx = squareLength / positionX;
        else cx = positionX / squareLength;

        if(positionY < squareLength)
            cy = squareLength / positionY;
        else cy = positionY / squareLength;

        if(junction == "TopLeft" || junction == "TopRight"){
            if(positionY < 0) {
                if (positionY > -squareLength)
                    goToY = 0 + precision;
                if (positionY < -squareLength)
                    goToY = squareLength * (cy - 1) + precision;
            }
            else{
                if (positionY < squareLength)
                    goToY = squareLength + 3;
                if (positionY > squareLength)
                    goToY = squareLength * (cy + 1) + precision;
            }
            if(junction == "TopLeft"){
                if(positionX < 0) {
                    if (positionX > -squareLength)
                        goToX = - squareLength + precision;
                    if (positionX < -squareLength)
                        goToX = squareLength * (cx - 1) + precision;
                }
                else{
                    if(positionX < squareLength)
                        goToX = 0 + precision;
                    if(positionX > squareLength)
                        goToX = cx * squareLength + precision;
                }
                targetPosition = new Vector2d(goToX, goToY);
            }
            else if(junction == "TopRight"){
                if(positionX < 0) {
                    if (positionX > -squareLength)
                        goToX = 0 + precision;
                    if (positionX < -squareLength)
                        goToX = squareLength * cx + precision;
                }
                else{
                    if(positionX < squareLength)
                        goToX = squareLength + precision;
                    if(positionX > squareLength)
                        goToX = squareLength * (cx + 1) + precision;
                }
                targetPosition = new Vector2d(goToX, goToY);
            }
        } else if(junction == "BotLeft" || junction == "BotRight"){
            if(positionY < 0) {
                if (positionY > -squareLength)
                    goToY = -squareLength + precision;
                if (positionY < -squareLength)
                    goToY = squareLength * (cy - 1) + precision;
            }
            else{
                if (positionY < squareLength)
                    goToY = squareLength + precision;
                if (positionY > squareLength)
                    goToY = squareLength * (cy + 1) + precision;
            }
            if(junction == "BotLeft"){
                if(positionX < 0) {
                    if (positionX > -squareLength)
                        goToX = - squareLength + precision;
                    if (positionX < -squareLength)
                        goToX = squareLength * (cx - 1) + precision;
                }
                else{
                    if(positionX < squareLength)
                        goToX = 0 + precision;
                    if(positionX > squareLength)
                        goToX = cx * squareLength + precision;
                }
                targetPosition =  new Vector2d(goToX, goToY);
            } else if(junction == "BotRight"){
                if(positionX < 0) {
                    if (positionX > -squareLength)
                        goToX = 0 + precision;
                    if (positionX < -squareLength)
                        goToX = squareLength * cx + precision;
                }
                else{
                    if(positionX < squareLength)
                        goToX = squareLength + precision;
                    if(positionX > squareLength)
                        goToX = squareLength * (cx + 1) + precision;
                }
                targetPosition = new Vector2d(goToX, goToY);
            }
        }
        targetPosition =  new Vector2d(goToX, goToY);
    }

    public void moveBrat(String direction){
        if(Objects.equals(direction, "up")){
            try {
                // Limit movement so that it doesn't go above max height

                if (liftCountZ < 4) {
                    runToPosition(liftCountZ, "up");
                }
                else liftCountZ=4;

                TimeUnit.MILLISECONDS.sleep(500);
            } catch(InterruptedException e){
                Thread.currentThread().interrupt();
            }
        } else if(Objects.equals(direction, "down")){
            try {
                // Limit movement so that it doesn't go below min height
                if (liftCountZ > 1) {
                    runToPosition(liftCountZ, "down");
                }
                else {
                    liftCountZ=1;
                }
                TimeUnit.MILLISECONDS.sleep(500);
            } catch (InterruptedException e){
                Thread.currentThread().interrupt();
            }
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
    public void runToPosition(int level, String rotation) {
        int ticks;
        if(rotation == "up"){
            liftCountZ++;
            level = liftCountZ;
            switch(level){
                case 2:
                    ticks = (int)(24 * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(robot.bratz.getCurrentPosition() - ticks);
                    break;
                case 3:
                    ticks = (int)(18 * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(robot.bratz.getCurrentPosition() - ticks);
                    break;
                case 4:
                    ticks = (int)(16 * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(robot.bratz.getCurrentPosition() - ticks);
                    break;
            }
        } else if(rotation == "down"){
            liftCountZ--;
            level = liftCountZ;
            switch(level){
                case 1:
                    robot.bratz.setTargetPosition(0);
                    break;
                case 2:
                    ticks = (int)(18 * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(robot.bratz.getCurrentPosition() + ticks);
                    break;
                case 3:
                    ticks = (int)(16 * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(robot.bratz.getCurrentPosition() + ticks);
                    break;
            }
        }
        robot.bratz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bratz.setPower(0.6);
    }
}
