package irobotcreate2;

public interface Leds extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irobotcreate2/Leds";
  static final java.lang.String _DEFINITION = "Header header\nbool warning\nbool dock\nbool spot\nbool dirt_detect\nuint8 clean_color\nuint8 clean_intensity\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  boolean getWarning();
  void setWarning(boolean value);
  boolean getDock();
  void setDock(boolean value);
  boolean getSpot();
  void setSpot(boolean value);
  boolean getDirtDetect();
  void setDirtDetect(boolean value);
  byte getCleanColor();
  void setCleanColor(byte value);
  byte getCleanIntensity();
  void setCleanIntensity(byte value);
}
