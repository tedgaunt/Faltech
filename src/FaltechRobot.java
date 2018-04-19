package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Admin on 11/24/2017.
 */

public class FaltechRobot {
    DriveTrain driveTrain = new DriveTrain();
    JewelArm jewelArm = new JewelArm();
    GlyphColllection glyphColllection = new GlyphColllection();
    Flipper flipper = new Flipper();
    RelicArm relicArm = new RelicArm();
    double glyphDistance[]={10, 17.5, 22};
    public void init(HardwareMap ahwMap, Telemetry myTelemetry){
        myTelemetry.addData("Robot","Init");
        myTelemetry.update();

           
         
        driveTrain.init(ahwMap,myTelemetry);
        
        jewelArm.init(ahwMap,myTelemetry);
        glyphColllection.init(ahwMap,myTelemetry);
        flipper.init(ahwMap,myTelemetry);
        relicArm.init(ahwMap,myTelemetry);

        myTelemetry.addData("Robot","Finished_Init");
        myTelemetry.update();
    }
    public void robotStop(){
        driveTrain.stop();
        jewelArm.stop();
        glyphColllection.stop();
        flipper.stop();
        relicArm.stop();
    }

}

