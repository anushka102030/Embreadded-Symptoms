//A class for knobs on the synth. 

class Knob
{
  private:
  bool sign;
  uint8_t rotation;
  int maxr;
  int minr;

  public:
  Knob(uint8_t rotation, bool sign, int maxr, int minr)
  {
    this->rotation = rotation;
    this->sign = sign;
    this->maxr = maxr;
    this->minr = minr;
  }

  void add (uint8_t i)
  {
    if(rotation != maxr) __atomic_add_fetch(&rotation, i, __ATOMIC_RELAXED);
    sign = true;
  }

  void sub (uint8_t i)
  {
    if(rotation != minr) __atomic_sub_fetch(&rotation, i, __ATOMIC_RELAXED);
    sign = false;
  }

  void cond_addsub (uint8_t i)
  {
    if(sign) add(i);
    else sub(i);
  }
  bool get_sign ()
  {
    return sign;
  }

  uint8_t get_rotation ()
  {
    return rotation;
  }

};
