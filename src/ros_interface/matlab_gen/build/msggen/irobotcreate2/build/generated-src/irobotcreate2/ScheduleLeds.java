package irobotcreate2;

public interface ScheduleLeds extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irobotcreate2/ScheduleLeds";
  static final java.lang.String _DEFINITION = "Header header\nbool sunday\nbool monday\nbool tuesday\nbool wednesday\nbool thursday\nbool friday\nbool saturday\nbool colon\nbool pm\nbool am\nbool clock\nbool schedule\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  boolean getSunday();
  void setSunday(boolean value);
  boolean getMonday();
  void setMonday(boolean value);
  boolean getTuesday();
  void setTuesday(boolean value);
  boolean getWednesday();
  void setWednesday(boolean value);
  boolean getThursday();
  void setThursday(boolean value);
  boolean getFriday();
  void setFriday(boolean value);
  boolean getSaturday();
  void setSaturday(boolean value);
  boolean getColon();
  void setColon(boolean value);
  boolean getPm();
  void setPm(boolean value);
  boolean getAm();
  void setAm(boolean value);
  boolean getClock();
  void setClock(boolean value);
  boolean getSchedule();
  void setSchedule(boolean value);
}
