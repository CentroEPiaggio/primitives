package irobotcreate2;

public interface Battery extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irobotcreate2/Battery";
  static final java.lang.String _DEFINITION = "Header header\nbool power_cord\nbool dock\nfloat32 level\t\t\t# in %\nuint32 time_remaining\t\t# in minutes\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  boolean getPowerCord();
  void setPowerCord(boolean value);
  boolean getDock();
  void setDock(boolean value);
  float getLevel();
  void setLevel(float value);
  int getTimeRemaining();
  void setTimeRemaining(int value);
}
