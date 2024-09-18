#ifndef UTILITIES_H_INCLUDED
#define UTILITIES_H_INCLUDED

/* absolute value macro (so we don't have to import all of stdlib.h
 * for one macro)
 */
#define Abs(k) (((k) < 0) ? ((-1) * (k)) : (k))

/* macro to convert a milliseconds time to microseconds (for rtc functions) */
#define MsToUs(x) ((x)*(1000ul))


#endif // UTILITIES_H_INCLUDED
