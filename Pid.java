package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.stormbots.MiniPID;

// dumb "pid" that tries to keep the orientation stable
// uses the integrated imu

public class Pid implements Runnable{
    
    Telemetry t;
    HardwareMap hwmap;
    MotorIO motorIO;
    bno055driver d;
    
    MiniPID pidctrl = new MiniPID(2, 0.0, 7);
    
    double rawAngle = 0;
    
    boolean kys_signal = false;
    double target = 0;
    double angleComp = Math.toRadians(180);
    double[] vec = {0, 0};
    
    boolean enabled = false;
    
    public void setTarget(){
        this.target = Math.toRadians(d.getAngles()[0]);
    }
    
    public void kys(){
        kys_signal = true;
    }
    
    public Pid(Telemetry t, HardwareMap hwmap, MotorIO motorIO){
        this.t = t;
        this.hwmap = hwmap;
        this.motorIO = motorIO;
        d = new bno055driver("imu", this.hwmap);
    }
    public void enable(){
        this.enabled=true;
    }
    public void disable(){
        this.enabled=false;
    }
    public double getAngle(){
        return this.rawAngle;
    }
    @Override
    public void run(){
        while(!kys_signal){
            if(this.enabled){
                double rawAngle = Math.toRadians(d.getAngles()[0]);
                double diff;
                if(Math.abs(rawAngle - target) > Math.PI){
                    if(rawAngle<target){
                        diff = (Math.PI-rawAngle - target);
                    }
                    else{
                        diff = -(Math.PI-rawAngle - target);
                    }
                }
                else{
                    diff = rawAngle - target;
                }
                if(Math.abs(diff)>0.08){
                        motorIO.setRotation(diff);
                }
                else{
                    motorIO.setRotation(0.0);
                }
            }
            else{
                motorIO.setRotation(0.0);
            }
         }
         
    }
    
}