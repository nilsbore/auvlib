//    $Id: Timestamp.hh,v 1.11 2007/03/19 23:34:26 johnf Exp $
//
// = AUTHOR(S)
//    John Folkesson
//    
//    Copyright (c) 2004 John Folkesson
//    

#ifndef CURE_TIMESTAMP_HH
#define CURE_TIMESTAMP_HH

#include <iostream>
#include <stdio.h>
#include <sys/time.h>  // struct timeval

namespace Cure{

/**
 * The Timestamp class has two longs: one for the Seconds and one for the 
 * Microsec.
 * Negative timestamps are normally a negative seconds and a 
 * positive microsec. So;
 *  
 * t=Seconds+Microsec*1E-6.
 * 
 * Printing of the number however till present the number as you would
 * expect, that is -0.1 is printed as -0.100000 and not as -1.900000
 * as it would be if the second and microsecond parts where simply
 * printed.
 * 
 * Not too exciting but it is nice to have +, -,  =, ==, <, >, ...
 * 
 * @author John Folkesson
 */
class Timestamp
{
public:
  /**
   * The seconds
   */
  long Seconds;
  /**
   * The MicroSeconds
   */
  long Microsec;
  /**
   * The constructor.
   */
  Timestamp(){
    Seconds=0;
    Microsec=0;
  }
  /**
   *  The constructor with initial values.
   * @param s the second value to intialize to.
   * @param us the microsecon value.
   */
  Timestamp (long s, long us) {
    Seconds = s;
    Microsec = us;
  }
  /**
   *  The constructor with initial values.
   * @param tv the initial time value
   */
  Timestamp(struct timeval &tv) {
    Seconds = tv.tv_sec;
    Microsec = tv.tv_usec;
  }
  /**
   *  The constructor with initial values.
   * @param t the initial timestamp is copied 
   */
  Timestamp(const Timestamp& t){
    Seconds=t.Seconds;
    Microsec=t.Microsec;
  }
  /**
   *  The constructor with initial values.
   * @param t the initial time value as a double number of seconds.
   */
  Timestamp(const double t){
    Seconds=(long)t;
    Microsec=(long)((double)(1000000*(t-(double)Seconds)));
  }
  ~Timestamp(){}
  /**
   *  The copy operator
   * @param t the time is copied from here.
   */
  void operator = (const Timestamp& t){
    Seconds=t.Seconds;
    Microsec=t.Microsec;
  }
  /**
   *  The assignment operator.
   * @param t the time in whole seconds only.
   */
  void operator = (const long t){
    Seconds=t;
    Microsec=0;
  }

  /**
   *  The assignment operator.
   * @param t the initial time value as a double number of seconds.
   */
  void operator = (const double t){
    Seconds=(long)t;
    Microsec=(long)((double)(1000000*(t-(double)Seconds)));
  }

  /**
   * Subtracts a timestamp from this.
   * @param t The time to subtract from this
   */
  void operator -= (const Timestamp& t)
  {
    Seconds-=t.Seconds;
    Microsec-=t.Microsec;
    normalize();
  }

  /**
   * Adds a timestamp to this.
   * @param t The time to add to this
   */
  void operator += (const Timestamp& t)
  {
    Seconds+=t.Seconds;
    Microsec+=t.Microsec;
    normalize();
  }

  /**
   * Subtracts a number of microseconds from this.
   * @param microseconds The time to subtract from this
   */
  void operator -= (const long microseconds)
  {
    Microsec-=microseconds;
    normalize();
  }

  /**
   * Adds a number of microseconds to this.
   * @param microseconds The time to add to this
   */
  void operator += (const long microseconds)
  {
    Microsec+=microseconds;
    normalize();
  }
  
  /**
   * Multiply by a factor.
   * @param d the factor to multiply
   */
  void operator *= (const double d)
  {
    normalize();
    double f=d;
    if (d<0)f=-d;
    double r=((double)Microsec*f);
    double s=r/1000000;
    unsigned long sec=(unsigned long)s;
    r-=((double)sec*1000000.0);
    Microsec=(long)r;
    r=((double)Seconds*f);
    Seconds=(long)r;
    r-=(double)Seconds;
    r*=1000000.0;
    Microsec+=(long)r;
    Seconds+=sec;
    if (d<0){
      Seconds=-Seconds;
      Microsec=-Microsec;
    }
    normalize();
  }
  /**
   * Divide by a factor.
   * @param d the factor to divide
   */
  void operator /= (const double d){
    operator *=((double)(1.0/d));
  }

  /**
   * Turns the timestamp into a double.
   * @return the time as a double number of seconds.
  */
  double getDouble()const{
    return((double)Seconds+(double)Microsec/(double)1000000);
  }

  /** Set this timestamp to be equal to the current time */
  void setToCurrentTime();
  
  /** Return a Timestamp set to current time */
  static Timestamp getCurrentTime();

  /**
   * This function will always use a positive microsecond part even in
   * case of negative numbers. That is the time -0.1 will be represented as 
   * Seconds=-1
   * Microsec=900000
   */
  void normalize()
  {
    while (Microsec<0)
      {
        Seconds--;
        Microsec+=1000000;
      }
    while (Microsec>999999)
      {
        Seconds++;
        Microsec-=1000000;
      }
  }

  /**
   * Test equality.
   * @param t the timestamp to compare. 
   * @return 1 if the times are the same else 0.
   */
  short operator == (const Timestamp& t)const {
    if(t.Seconds==Seconds)
      if (t.Microsec==Microsec)
	return 1;
    return 0;
  }
  /**
   * Test not equality.
   * @param t the timestamp to compare. 
   * @return 1 if the times are not the same else 0.
   */
  short operator != (const Timestamp& t)const {
    if(t.Seconds!=Seconds || t.Microsec!=Microsec)
      return 1;
    return 0;
  }
  /**
   * Test inequality.
   * @param t the timestamp to compare. 
   * @return 1 if the time is greater than t.
   */
  short operator > (const Timestamp& t)const {
    if (Seconds>t.Seconds)return 1;
    else if (Seconds<t.Seconds)return 0;
    if (Microsec>t.Microsec)return 1;
    return 0;
  }
  /**
   * Test inequality.
   * @param t the timestamp to compare. 
   * @return 1 if the time is greater or equal to t.
   */
  short operator >= (const Timestamp& t)const {
    if (Seconds>t.Seconds)return 1;
    else if (Seconds<t.Seconds)return 0;
    if (Microsec>=t.Microsec)return 1;
    return 0;
  }
  /**
   * Test inequality.
   * @param t the timestamp to compare. 
   * @return 1 if the time is less than t.
   */
  short operator < (const Timestamp& t)const {
    if (Seconds<t.Seconds)return 1;
    else if (Seconds>t.Seconds)return 0;
    if (Microsec<t.Microsec)return 1;
    return 0;
  }
  /**
   * Test inequality.
   * @param t the timestamp to compare. 
   * @return 1 if the time is less or equal to t.
   */
  short operator <= (const Timestamp& t)const {
    if (Seconds<t.Seconds)return 1;
    else if (Seconds>t.Seconds)return 0;
    if (Microsec<=t.Microsec)return 1;
    return 0;
  }

  void print(std::ostream &os) const
  {
    char ubuf[10];
    if (Seconds >= 0) {
      sprintf(ubuf, "%06ld", Microsec);
      os << Seconds << "." << ubuf;
    } else if (Seconds<-1){
      sprintf(ubuf, "%06ld", 1000000-Microsec);
      os << Seconds+1 << "." << ubuf;
    }else{
      sprintf(ubuf, "%06ld", 1000000-Microsec);
      os  << "-0." << ubuf;
    }
  }

  /**
   * Print timestamp to display.
   */
  void print()const
  {
    std::cerr << std::endl;
    std::cerr << "timestamp: ";
    print(std::cerr);
  }
};

/**
 * Print a timestamp on an output stream
 */
inline std::ostream& operator<< (std::ostream& os, const Timestamp &t)
{
  t.print(os);
  return os;
}

} // namespace Cure

/**
 * This function adds two timestamps and returns the sum
 *
 * @param t1 timestamp 1
 * @param t2 timestamp 2
 *
 * @return sum of two timestamps, i.e. t1 + t2
 */
inline Cure::Timestamp 
operator+(const Cure::Timestamp &t1, const Cure::Timestamp &t2)
{
  Cure::Timestamp res(t1);
  res+=t2;
  return res;
}

/**
 * This function substracts two timestamps and returns the sum
 *
 * @param t1 timestamp 1
 * @param t2 timestamp 2
 *
 * @return difference between two timestamps, i.e. t1 - t2
 */
inline Cure::Timestamp 
operator-(const Cure::Timestamp &t1, const Cure::Timestamp &t2)
{
  Cure::Timestamp res(t1);
  res-=t2;
  return res;
}

#endif // CURE_TIMESTAMP_HH
