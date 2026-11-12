import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.library.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;


public IntakeSubystem {

  private DcMotorEx frontIntakeMotor, rearIntakeMotor;



}


public init(HardwareMap HwMap) {

  frontintakeMotor = HwMap.get(DcMotorEx.class, "FI");
  rearIntakeMotor = HwMap.get(DcMotorEx.class, "RI");

  frontIntakeMotor.setDirection(DcMotorSimpleDirection.REVERSE);
  rearIntakeMotor.setDirection(DcMotorSimpleDirection.Reverse);
  

  }












public void 
