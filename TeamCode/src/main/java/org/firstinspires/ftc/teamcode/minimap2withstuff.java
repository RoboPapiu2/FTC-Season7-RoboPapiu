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

import com.acmerobotics.dashboard.config.Config;
import com.vuforia.Device;

import org.firstinspires.ftc.teamcode.configs.VariableConfig;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.Objects;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class minimap2withstuff extends LinearOpMode {
    hardwarePapiu robot = new hardwarePapiu();
    VariableConfig config = new VariableConfig();

    //Global values.
    public static double DRAWING_TARGET_RADIUS = 2;
    boolean isMoving=false;
    boolean isOpen=false;
    double joystick_r_y, joystick_r_x, joystick_l_y, joystick_l_x;
    double positionX, positionY;
    double currentTime2 = 0;
    boolean overrideInput2 = true;

    int tickModifier = 0;

    Vector2d selectedMatPosition = new Vector2d();

    enum MODEPARENT {
        minimap,
        glisiera
    }

    enum MODE {
        minimap,
        minimapSelect
    }
    enum matEnum {
        mat,
        junction
    }
    matEnum currentModeMat = matEnum.mat;
    MODE currentMode = MODE.minimap;
    MODEPARENT currentModeParent = MODEPARENT.glisiera;

    /** Selected junction position **/
    Vector2d targetPosition = new Vector2d(0, 0);


    // or HEADING_PID
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
    // Read pose
    Pose2d poseEstimate = new Pose2d();

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.getLocalizer().setPoseEstimate(TransferPose.currentPose);
        headingController.setInputBounds(-Math.PI, Math.PI);


        robot.init(hardwareMap);
        config.init(hardwareMap);
        robot.EncoderReset();

        Pose2d StartPose = TransferPose.currentPose;
        if(StartPose.getX() == 0 && StartPose.getY() ==0 && StartPose.getHeading() == Math.toRadians(0))
            StartPose = new Pose2d(-35.5, -61, Math.toRadians(90));
        drive.setPoseEstimate(StartPose);


        /** Default motor values / Reset encoders **/
        robot.bratz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bratz.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        config.brateCleste();

        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            poseEstimate = drive.getLocalizer().getPoseEstimate();
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

            switch (currentModeParent){
                case minimap:
                    switch (currentMode) {
                        case minimap:

                            // Switch to AUTO, even if second driver initiates a command
                            if(gamepad2.a)
                                currentMode = MODE.minimapSelect;

                            if(gamepad2.dpad_up)
                                config.moveBratSus("up",tickModifier);

                            if(gamepad2.dpad_down)
                                config.moveBratSus("low",tickModifier);

                            if(gamepad2.y)
                                config.brateCleste();

                            if(gamepad2.dpad_right)
                                config.moveBratSus("middle",tickModifier);

                            if(gamepad2.dpad_left)
                                config.moveBratSus("down",tickModifier);

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
                        case minimapSelect:

                            // Switch back to TELEOP
                            if(gamepad2.b)
                                currentMode = MODE.minimap;

                            if(gamepad2.dpad_up)
                                config.moveBratSus("up",tickModifier);

                            if(gamepad2.dpad_down)
                                config.moveBratSus("low",tickModifier);

                            if(gamepad2.y)
                                config.brateCleste();

                            if(gamepad2.dpad_right)
                                config.moveBratSus("middle",tickModifier);

                            if(gamepad2.dpad_left)
                                config.moveBratSus("down",tickModifier);

                            Vector2d fieldFrameInput;

                            if(joystick_r_trigger>1) {
                                fieldFrameInput = new Vector2d(
                                        -joystick_l_x/(1.5*joystick_r_trigger),
                                        -joystick_l_y/(1.5*joystick_r_trigger)
                                );
                            }
                            else{
                                fieldFrameInput = new Vector2d(
                                        -joystick_l_x/1.2,
                                        -joystick_l_y/1.2
                                );
                            }
                            Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());
                            // Target vector - current position
                            Vector2d difference = targetPosition.minus(poseEstimate.vec());
                            // Angle and Derivative
                            double theta = difference.angle();
                            // Derivative atan2
                            double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());
                            // Set the target heading for the heading controller to our desired angle
                            headingController.setTargetPosition(theta);
                            // velocity feedforward
                            double headingInput = (headingController.update(poseEstimate.getHeading())
                                    * DriveConstants.kV + thetaFF)
                                    * DriveConstants.TRACK_WIDTH
                                    * (Math.abs(joystick_r_x-1));
                            // Refactor pose
                            driveDirection = new Pose2d(
                                    robotFrameInput,
                                    headingInput
                            );

                            // Draw lines to target
                            fieldOverlay.setStroke("#b89eff");
                            fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
                            fieldOverlay.setStroke("#ffce7a");
                            fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), targetPosition.getX(), poseEstimate.getY());
                            fieldOverlay.strokeLine(targetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());
                            break;
                    }
                    switch(currentModeMat){
                        case mat:
                            if(gamepad1.start && overrideInput2) {
                                currentTime2 = getRuntime();
                                overrideInput2 = false;
                                currentModeMat = matEnum.junction;
                            }
                            else {
                                if(getRuntime() - currentTime2 > 0.2)
                                    overrideInput2 = true;
                            }

                            setTargetMat();
                            break;
                        case junction:
                            if(gamepad1.start && overrideInput2) {
                                currentTime2 = getRuntime();
                                overrideInput2 = false;
                                currentModeMat = matEnum.mat;
                            }
                            else {
                                if(getRuntime() - currentTime2 > 0.2)
                                    overrideInput2 = true;
                            }
                            setTargetJunction();
                            break;
                    }
                    if(gamepad1.back && overrideInput2) {
                        currentTime2 = getRuntime();
                        overrideInput2 = false;
                        currentModeParent = MODEPARENT.glisiera;
                    }
                    else {
                        if(getRuntime() - currentTime2 > 0.2)
                            overrideInput2 = true;
                    }
                    break;
                case glisiera:

                    if(gamepad2.dpad_up)
                        config.moveBratSus("up",tickModifier);

                    if(gamepad2.dpad_down)
                        config.moveBratSus("low", tickModifier);

                    if(gamepad2.y)
                        config.brateCleste();

                    if(gamepad2.dpad_right)
                        config.moveBratSus("middle", tickModifier);

                    if(gamepad2.dpad_left)
                        config.moveBratSus("down", tickModifier);

                    // If trigger is pressed, give slower controls for extra precision
                    if(joystick_r_trigger>1) {
                        driveDirection = new Pose2d(
                                -joystick_l_y/(1.5*joystick_r_trigger),
                                -joystick_l_x/(1.5*joystick_r_trigger),
                                -joystick_r_x/(5*joystick_r_trigger)
                        );
                    }
                    else if(gamepad2.right_bumper){
                        driveDirection = new Pose2d(
                                -joystick_l_y,
                                -joystick_l_x,
                                -joystick_r_x
                        );
                    }
                    else {
                        driveDirection = new Pose2d(
                                -joystick_l_y/1.2,
                                -joystick_l_x/1.2,
                                -joystick_r_x/1.2
                        );
                    }

                    if(gamepad1.dpad_up)
                        config.moveBratSus("up", tickModifier);

                    if(gamepad1.dpad_down)
                        config.moveBratSus("low", tickModifier);

                    if(gamepad1.dpad_right)
                        config.moveBratSus("middle", tickModifier);

                    if(gamepad1.dpad_left)
                        config.moveBratSus("down", tickModifier);

                    if(gamepad1.left_stick_y>0){
                        robot.bratz.setTargetPosition(robot.bratz.getCurrentPosition() + 40);
                        robot.bratz.setPower(1);
                    } else if(gamepad1.left_stick_y<0){
                        robot.bratz.setTargetPosition(robot.bratz.getCurrentPosition() - 40);
                        robot.bratz.setPower(1);
                    }
                    if(gamepad1.a){
                        tickModifier = robot.bratz.getCurrentPosition();
                    }

                    telemetry.addLine("Selected mode: GLISIERA\nPress back for minimap");
                    telemetry.addData("brat ticks: ", robot.bratz.getCurrentPosition());
                    telemetry.update();


                    if(gamepad1.back && overrideInput2) {
                        currentTime2 = getRuntime();
                        overrideInput2 = false;
                        currentModeParent = MODEPARENT.minimap;
                    }
                    else {
                        if(getRuntime() - currentTime2 > 0.2)
                            overrideInput2 = true;
                    }
                    break;
            }

            // Draw bot on canvas
            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate);

            // Draw the target on the field
            fieldOverlay.setStroke("#dd2c00");
            fieldOverlay.strokeCircle(targetPosition.getX(), targetPosition.getY(), DRAWING_TARGET_RADIUS);

            drive.setWeightedDrivePower(driveDirection);

            // Update the heading controller with our current heading
            headingController.update(poseEstimate.getHeading());

            // Update the localizer
            drive.getLocalizer().update();

            // Send telemetry packet off to dashboard
            FtcDashboard.getInstance().sendTelemetryPacket(packet);


//            telemetry.addData("mode", currentMode);
//            telemetry.addData("lift level bratz: ", liftCountZ);
        }
    }

    public enum mat{
        mat13(-2*23.5, 2*23.5),
        mat14(-23.5, 2*23.5),
        mat15(0, 2*23.5),
        mat16(23.5, 2*23.5),

        mat9(-2*23.5, 23.5),
        mat10(-23.5, 23.5),
        mat11(0, 23.5),
        mat12(23.5, 23.5),

        mat5(-2*23.5, -23.5),
        mat6(-23.5, -23.5),
        mat7(0, -23.5),
        mat8(23.5, -23.5),

        mat1(-2*23.5, -2*23.5),
        mat2(-23.5, -2*23.5),
        mat3(0, -2*23.5),
        mat4(23.5, -2*23.5);
        private final double y;
        private final double x;

        mat(double x, double y){
            this.x = x;
            this.y = y;
        }
    }

    int indexOfMat=13, iSecond=0;
    String message;
    double currentTime = 0;
    boolean overrideInput = true;
    public void setTargetMat(){
        if(overrideInput) {
            currentTime = getRuntime();
            if (gamepad1.dpad_right) {
                indexOfMat++;
                overrideInput=false;
                if (indexOfMat > 16) indexOfMat = 16;
            }
            if (gamepad1.dpad_left) {
                indexOfMat--;
                overrideInput=false;
                if (indexOfMat < 1) indexOfMat = 1;
            }
            if (gamepad1.dpad_up) {
                indexOfMat -= 4;
                overrideInput=false;
                if (indexOfMat < 1) indexOfMat += 4;
            }
            if (gamepad1.dpad_down) {
                indexOfMat += 4;
                overrideInput=false;
                if (indexOfMat > 16) indexOfMat -= 4;
            }
        } else {
            if(getRuntime() - currentTime > 0.2)
                overrideInput = true;
        }
        int i=0;
        for(mat selectedMat : mat.values()){
            i++;
            if(i==indexOfMat){
                selectedMatPosition = new Vector2d(selectedMat.x, selectedMat.y);
                break;
            }
        }
        iSecond=0;
        message = "";
        for(int m=1;m<=4;++m){
            for(int j=1;j<=4;++j){
                iSecond++;
                if(iSecond==indexOfMat)
                    message = message.concat("  +  ");
                else message = message.concat("  -  ");
            }
            message = message.concat("\n");
        }

        telemetry.addLine("Selected mode: MINIMAP\nPress back for glisiera");
        telemetry.addLine(message);
        telemetry.addData("Current mat: ", indexOfMat);
        telemetry.update();
    }

    public void setTargetJunction(){
        if(gamepad1.y) //top left
            targetPosition = new Vector2d(selectedMatPosition.getX(), selectedMatPosition.getY() + 23.5);
        else if(gamepad1.b) // top right
            targetPosition = new Vector2d(selectedMatPosition.getX()+23.5, selectedMatPosition.getY() + 23.5);
        else if(gamepad1.x) // bot left
            targetPosition = new Vector2d(selectedMatPosition.getX(), selectedMatPosition.getY());
        else if(gamepad1.a) // bot right
            targetPosition = new Vector2d(selectedMatPosition.getX()+23.5, selectedMatPosition.getY());

        telemetry.addLine("Selected mode: MINIMAP\nPress back for glisiera");
        telemetry.addLine("Y - Top left \n B - Top right \n X - Bot left \n A - Bot right \n");
        telemetry.addData("Current mat selected\n", message);
        telemetry.addData("Current position selected: ", targetPosition);
        telemetry.update();
    }
}
