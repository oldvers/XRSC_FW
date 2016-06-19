#define MAX_PICTURE_SIZE   50*35

//typedef struct sPorts3
//{
//  unsigned char P1;
//  unsigned char P2;
//  unsigned char P3;
//} tPorts3, * pPorts3;

//typedef struct sPorts4
//{
//  unsigned char P1;
//  unsigned char P2;
//  unsigned char P3;
//	unsigned char P4;
//} tPorts4, * pPorts4;

//typedef struct sPorts5
//{
//  unsigned char P1;
//  unsigned char P2;
//  unsigned char P3;
//  unsigned char P4;
//  unsigned char P5;
//} tPorts5, * pPorts5;

//extern const char DT[4][4];

unsigned char * unpack_full(unsigned char const * apackedbuf, unsigned short const apackedsize);
unsigned char * unpack_mono(unsigned char const * apackedbuf, unsigned short const apackedsize);

