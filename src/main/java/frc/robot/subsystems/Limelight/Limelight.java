package frc.robot.subsystems.Limelight;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight.LimelightIO.LimelightIOInputsAutoLogged;
import frc.robot.subsystems.Limelight.LimelightIO.LimelightIO;
import frc.robot.utils.ShooterUtils;

public class Limelight extends SubsystemBase{
   private LimelightIO limelightIO;
   private LimelightIOInputsAutoLogged limelightIOInputs = new LimelightIOInputsAutoLogged();

   public Limelight(LimelightIO limelightIO) {
        this.limelightIO = limelightIO;
   }

   public double getAngleOffset() {
        return limelightIO.getAngleOffset();
   }

   public double getDistance() {
        return limelightIO.getDistance();
   }

   @Override
   public void periodic() {
       limelightIO.updateInputs(limelightIOInputs);
       Logger.processInputs("Limelight", limelightIOInputs);
   }
   
}
