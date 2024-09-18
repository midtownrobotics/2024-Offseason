package frc.robot.subsystems.Limelight;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight.LimelightIO.LimelightIO;
import frc.robot.subsystems.Limelight.LimelightIO.LimelightIOInputsAutoLogged;

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

   public double getTx() {
     return limelightIO.getTx();
   }

   public boolean isValidTarget(String[] targetIds) {
     return limelightIO.getValidTargetExists() && Arrays.stream(targetIds).anyMatch(Integer.toString(limelightIO.getId())::equals);
   }

   @Override
   public void periodic() {
       limelightIO.updateInputs(limelightIOInputs);
       Logger.processInputs("Limelight", limelightIOInputs);
   }
   
}
