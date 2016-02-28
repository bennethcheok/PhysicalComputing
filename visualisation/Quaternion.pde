//Model for quaternions
class Quaternion
{
  float w, x, y, z;
  
  public Quaternion()
  {
    w = 1;
    x = 0;
    y = 0;
    z = 0;
  }
  
  public Quaternion(float w, float x, float y, float z)
  {
    this.w = w;
    this.x = x;
    this.y = y;
    this.z = z;
  }
  
  public String toString()
  {
    return w + ", " + x + ", " + y + ", " + z;
  }
}