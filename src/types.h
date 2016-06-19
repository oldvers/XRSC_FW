#ifndef TYPES_H
#define TYPES_H

#ifndef NULL
#define NULL  ((void *)0)
#endif

#ifndef FALSE
#define FALSE   0
#endif

#ifndef TRUE
#define TRUE    1
#endif

#ifndef U8
  typedef unsigned char        U8;
#endif
//#ifndef S8
//  typedef char                 S8;
//#endif
#ifndef U8C
  typedef unsigned char const  U8C;
#endif
#ifndef CU8
  typedef const unsigned char  CU8;
#endif
#ifndef U16
  typedef unsigned short       U16;
#endif
#ifndef S16
  typedef short                S16;
#endif
#ifndef U16C
  typedef unsigned short const U16C;
#endif
#ifndef CU16
  typedef const unsigned short CU16;
#endif
#ifndef S32
  typedef int                  S32;
#endif
#ifndef U32
  typedef unsigned int         U32;
#endif
#ifndef U32C
  typedef unsigned int const   U32C;
#endif
#ifndef CU32
  typedef const unsigned int   CU32;
#endif

#endif //TYPES_H
