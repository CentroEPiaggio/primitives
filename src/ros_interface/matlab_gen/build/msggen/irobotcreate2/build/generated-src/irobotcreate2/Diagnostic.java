package irobotcreate2;

public interface Diagnostic extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irobotcreate2/Diagnostic";
  static final java.lang.String _DEFINITION = "Header header\nint16 left_motor_current\nint16 right_motor_current\nint16 main_brush_current\nint16 side_brush_current\nbool left_motor_overcurrent\nbool right_motor_overcurrent\nbool main_brush_overcurrent\nbool side_brush_overcurrent\nint16 battery_current\nint16 battery_voltage\nint8 battery_temperature\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  short getLeftMotorCurrent();
  void setLeftMotorCurrent(short value);
  short getRightMotorCurrent();
  void setRightMotorCurrent(short value);
  short getMainBrushCurrent();
  void setMainBrushCurrent(short value);
  short getSideBrushCurrent();
  void setSideBrushCurrent(short value);
  boolean getLeftMotorOvercurrent();
  void setLeftMotorOvercurrent(boolean value);
  boolean getRightMotorOvercurrent();
  void setRightMotorOvercurrent(boolean value);
  boolean getMainBrushOvercurrent();
  void setMainBrushOvercurrent(boolean value);
  boolean getSideBrushOvercurrent();
  void setSideBrushOvercurrent(boolean value);
  short getBatteryCurrent();
  void setBatteryCurrent(short value);
  short getBatteryVoltage();
  void setBatteryVoltage(short value);
  byte getBatteryTemperature();
  void setBatteryTemperature(byte value);
}
