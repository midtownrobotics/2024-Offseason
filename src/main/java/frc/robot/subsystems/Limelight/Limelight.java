package frc.robot.subsystems.Limelight;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight.LimelightIO.LimelighIOInputsAutoLogged;
import frc.robot.subsystems.Limelight.LimelightIO.LimelightIO;

public class Limelight extends SubsystemBase{
   private LimelightIO limelightIO;
   private LimelighIOInputsAutoLogged limelighIOInputs = new LimelighIOInputsAutoLogged();

   public double getAngleOffset() {
        return limelightIO.getAngleOffset();
   }

   public double getDistance() {
        return limelightIO.getDistance();
   }

   @Override
   public void periodic() {
       limelightIO.updateInputs(limelighIOInputs);
       Logger.processInputs("Limelight", limelighIOInputs);
   }
   
}
