#ifndef UPBIT_COUNTER_h
#define UPBIT_COUNTER_h

// from_t is unsigned type only
template <typename from_t, typename to_t>
class UpbitCounter{
private:
  const from_t overflowValue = (from_t)(-1);
  to_t loopped = 0;
  from_t lastCounter;
public:
  void update(from_t counter){
    if(abs(counter - lastCounter) > (overflowValue >> 1))
      loopped += (counter < lastCounter) ? 1 : -1;
    lastCounter = counter;
  }
  to_t getCount(){
    to_t bigCount = loopped * overflowValue + lastCounter;
    return bigCount;
  }
  void reset(){
    loopped = 0;
    lastCounter = 0;
  }
};

#endif