package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="iha ok, mut ootteko kattonu simpsonit sarjasta jakson himo läski homer :D siinä esiintyy koko simpsonit perhe eli myös bart simpsons homer poika fanit saavat nauraa ja naurattaahan se tietty myös vaikka homerin läski kuteet ja muut :D kannattaa kattoo nopee", group="Linear Opmode")
public class OPMode extends LinearOpMode {
    @Override
    public void runOpMode(){
        // create objects
        MotorIO motorIO = new MotorIO(telemetry, hardwareMap);
        Pid pid = new Pid(telemetry, hardwareMap, motorIO);
        GamepadIO gamepadIO = new GamepadIO(telemetry, hardwareMap, gamepad1, motorIO, pid);
        
        // create thread objects
        Thread motorIOThread = new Thread(motorIO);
        Thread pidThread = new Thread(pid);
        Thread gamepadIOThread = new Thread(gamepadIO);

        waitForStart();

        // start threads
        motorIOThread.start();
        pidThread.start();
        gamepadIOThread.start();
        
        while(opModeIsActive()){
            try{
                Thread.sleep(30);
            }
            catch(InterruptedException e){
            }
        }
        // kill threads
        motorIO.kys();
        gamepadIO.kys();
        pid.kys();
    }
}