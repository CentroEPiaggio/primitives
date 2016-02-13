package irobotcreate2;

public interface Buttons extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irobotcreate2/Buttons";
  static final java.lang.String _DEFINITION = "Header header\nbool clean\nbool spot\nbool dock\nbool day\nbool hour\nbool minute\nbool schedule\nbool clock\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  boolean getClean();
  void setClean(boolean value);
  boolean getSpot();
  void setSpot(boolean value);
  boolean getDock();
  void setDock(boolean value);
  boolean getDay();
  void setDay(boolean value);
  boolean getHour();
  void setHour(boolean value);
  boolean getMinute();
  void setMinute(boolean value);
  boolean getSchedule();
  void setSchedule(boolean value);
  boolean getClock();
  void setClock(boolean value);
}
