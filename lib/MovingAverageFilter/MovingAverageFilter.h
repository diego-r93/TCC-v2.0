#ifndef MOVINGAVERAGEFILTER_H
#define MOVINGAVERAGEFILTER_H

class MovingAverageFilter {
  public:
   MovingAverageFilter(int size);
   ~MovingAverageFilter();
   void addValue(int value);
   float getAverage() const;

  private:
   int *_buffer;
   int _size;
   int _index;
   long _sum;
};

#endif
