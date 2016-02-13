package irobotcreate2;

public interface Brushes extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irobotcreate2/Brushes";
  static final java.lang.String _DEFINITION = "Header header\nbool main_brush\nint8 main_brush_pwm\nbool main_brush_direction\nbool side_brush\nint8 side_brush_pwm\nbool side_brush_clockwise\nbool vacuum\nint8 vacuum_pwm\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  boolean getMainBrush();
  void setMainBrush(boolean value);
  byte getMainBrushPwm();
  void setMainBrushPwm(byte value);
  boolean getMainBrushDirection();
  void setMainBrushDirection(boolean value);
  boolean getSideBrush();
  void setSideBrush(boolean value);
  byte getSideBrushPwm();
  void setSideBrushPwm(byte value);
  boolean getSideBrushClockwise();
  void setSideBrushClockwise(boolean value);
  boolean getVacuum();
  void setVacuum(boolean value);
  byte getVacuumPwm();
  void setVacuumPwm(byte value);
}
