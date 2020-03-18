//    $Id: Timestamp.cc,v 1.5 2007/09/14 09:14:06 johnf Exp $
//
// = AUTHOR(S)
//    John Folkesson
//    
//    Copyright (c) 2004 John Folkesson
//    
#include <preprocess/Timestamp.hh>
#include <sys/time.h>

namespace Cure {

void 
Timestamp::setToCurrentTime()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  Seconds = tv.tv_sec;
  Microsec = tv.tv_usec;
}

Timestamp
Timestamp::getCurrentTime()
{
  Timestamp t;
  t.setToCurrentTime();
  return t;
}

} // namespace Cure
