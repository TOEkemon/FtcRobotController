

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.library.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.command.SubsystemBase;


public IntakeSubystem extend SubsystemBase {

  private DcMotorEx frontIntakeMotor, rearIntakeMotor;



}


public init(HardwareMap HwMap) {

  frontintakeMotor = HwMap.get(DcMotorEx.class, "FI");
  rearIntakeMotor = HwMap.get(DcMotorEx.class, "RI");

  frontIntakeMotor.setDirection(Direction.REVERSE);
  rearIntakeMotor.setDirection(Direction.REVERSE);

  frontIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  rearIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

  frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  rearIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  

  }




public void setIntakeMotorPower(double power) {
  if() {



  }







// public void
