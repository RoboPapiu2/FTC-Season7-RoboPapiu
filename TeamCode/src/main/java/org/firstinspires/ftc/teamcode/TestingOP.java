//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.checkerframework.checker.units.qual.A;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import java.sql.Array;
//import java.util.HashMap;
//import java.util.Objects;
//import java.util.concurrent.TimeUnit;
//
//@Disabled
//@TeleOp (name = "TestingOP")
//public class TestingOP extends LinearOpMode {
//    double ServoIncrement=0.5, MotorIncrement=0.5;
//    double MotorSpeed = 0, ServoPos = 0;
//    int menuItem = 0, prevmenuItem =0;
//    boolean motor0=true, motor1=false,motor2=false,motor3=false,motor4=false,motor5=false,motor6=false,motor7=false;
//    boolean servo0=true, servo1=false,servo2=false,servo3=false,servo4=false,servo5=false,servo6=false,servo7=false;
//    boolean isMotorOn=false;
//    boolean isTSR = false; // False = continue program, true = end
//    HashMap<Integer, String> MotorNames = new HashMap<Integer, String>(); //TODO: FINISH THIS
//    String MenuItemName = "main";
//    hardwareTesting robot = new hardwareTesting();
//    @Override
//    public void runOpMode() throws InterruptedException{
//        robot.init(hardwareMap);
//        waitForStart();
//        if(opModeIsActive()){
//            // You will need to clear the terminal after each update
//            // False disables auto clear on update() call
//            telemetry.setAutoClear(false);
//            setMenu(MenuItemName);
//        }
//    }
//    void setMenu(String name){
//        switch(name){
//            case "main":
//                menuItem = 1;
//                MenuItemName = "out";
//                mainMenu();
//                break;
//            case "motor":
//                menuItem = 1;
//                MenuItemName = "out";
//                motorMenu();
//                break;
//            case "servo":
//                menuItem = 1;
//                MenuItemName = "out";
//                servoMenu();
//            case "control":
//                menuItem = 1;
//                MenuItemName = "out";
//                controlMenu();
//            case "telemetry":
//                MenuItemName = "out";
//                menuItem = 1;
//                telemetryMenu();
//                break;
//        }
//    }
//    void updateMenu(String typeOfMotor){
//        if(menuItem>7) menuItem=7;
//        if(menuItem<0) menuItem=0;
//        if(Objects.equals(typeOfMotor, "motor")) {
//            switch(menuItem){
//                case 0:
//                    motor0=true;
//                    motor1=false;
//                    motor2=false;
//                    motor3=false;
//                    break;
//                case 1:
//                    motor0=false;
//                    motor1=true;
//                    motor2=false;
//                    motor3=false;
//                    break;
//                case 2:
//                    motor0=false;
//                    motor1=false;
//                    motor2=true;
//                    motor3=false;
//                    break;
//                case 3:
//                    motor0=false;
//                    motor1=false;
//                    motor2=false;
//                    motor3=true;
//                    break;
//            }
//
//        } else if(Objects.equals(typeOfMotor, "servo")){
//            switch(menuItem){
//                case 0:
//                    servo0=true;
//                    servo1=false;
//                    servo2=false;
//                    servo3=false;
//                    break;
//                case 1:
//                    servo0=false;
//                    servo1=true;
//                    servo2=false;
//                    servo3=false;
//                    break;
//                case 2:
//                    servo0=false;
//                    servo1=false;
//                    servo2=true;
//                    servo3=false;
//                    break;
//                case 3:
//                    servo0=false;
//                    servo1=false;
//                    servo2=false;
//                    servo3=true;
//                    break;
//            }
//        }
//    }
//    void mainMenu(){
//        int selectedItem=1;
//        boolean terminate = false;
//        telemetry.clear();
//        telemetry.addLine("Motors");
//        telemetry.addLine("Servos");
//        telemetry.addLine("Selected: Motors");
//        telemetry.update();
//        while(!isTSR && !terminate) {
//            if (gamepad2.dpad_up) {
//                selectedItem = 2;
//                telemetry.addLine("Selected: Servos");
//                telemetry.update();
//            } else if (gamepad2.dpad_down) {
//                selectedItem = 1;
//                Telemetry.Line grog = telemetry.addLine("gorg");
//                telemetry.addLine("Selected: Motors");
//                telemetry.removeLine(grog);
//                telemetry.update();
//            }
//            if (gamepad2.a) {
//                try {
//                    terminate=true;
//                    telemetry.clear();
//                    if (selectedItem == 1) {
//                        setMenu("motor");
//                    } else {
//                        setMenu("servo");
//                    }
//                    TimeUnit.MILLISECONDS.sleep(300);
//                } catch (InterruptedException e) {
//                    Thread.currentThread().interrupt();
//                }
//            }
//        }
//    }
//    void motorMenu(){
//        if(gamepad2.y){
//            setMenu("main");
//        }
//        if(gamepad2.dpad_down){
//            menuItem--;
//            updateMenu("motor");
//        } else if(gamepad2.dpad_up) {
//            menuItem++;
//            updateMenu("motor");
//        } else if(gamepad2.a){
//            updateMenu("control");
//        }
//        telemetry.clear();
//        telemetry.addData("Motor 0", motor0);
//        telemetry.addData("Motor 1", motor1);
//        telemetry.addData("Motor 2", motor2);
//        telemetry.addData("Motor 3", motor3);
//        telemetry.update();
//    }
//    void controlMenu(String typeOfMotor){
//        if(Objects.equals(typeOfMotor, "motor")){
//            switch (menuItem){
//                case 1:
//
//            }
//        }
//        if(gamepad2.a){
//            try {
//                isMotorOn=!isMotorOn;
//                if(isMotorOn) //todo: finish this
//                TimeUnit.MILLISECONDS.sleep(300);
//            } catch (InterruptedException e){
//                Thread.currentThread().interrupt();
//            }
//        }
//        if(gamepad2.dpad_left) robot.setIncrements(0);
//        else if(gamepad2.dpad_right) robot.setIncrements(1);
//        telemetry.addLine("Gamepad B: Start/Stop motor");
//        telemetry.addLine("Gamepad Y: go to previous menu");
//        telemetry.addLine("Dpad right/left: add or decrease increment");
//        telemetry.addLine("Dpad up/down: add or decrease speed");
//        telemetry.addLine(" ");
//        // Returns DcMotor, might break the app
//        // telemetry.addData("Current motor:", robot.MotorMap.get(menuItem));
//        telemetry.addData("Motor moving:", isMotorOn);
//        telemetry.addData("Speed: ", MotorSpeed);
//        telemetry.addData("Increment speed by: ", MotorIncrement);
//        telemetry.update();
//    }
//}
