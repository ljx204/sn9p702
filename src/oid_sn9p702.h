#ifndef  __OID_SN9P702_H__
#define  __OID_SN9P702_H__


typedef unsigned char  BOOLEAN;                
typedef unsigned char  INT8U;                  
typedef signed   char  INT8S;                  
typedef unsigned short INT16U;                  
typedef signed   short INT16S;                
typedef unsigned int   INT32U;                 
typedef signed   int   INT32S;                  
typedef float          FP32;                    
typedef double         FP64;                    
typedef	signed   long long      INT64S;	       
typedef	unsigned long long      INT64U;	       


#define OID3_TYPE_COMMAND 			(1<<7)
#define OID3_TYPE_CODEMODE3			(1<<6)
#define OID3_TYPE_OID3_POS			(1<<5)
#define OID3_TYPE_READ_ERR			(1<<4)

#define TRUE	1
#define FALSE	0

typedef union {
	struct {
		INT16U command;
		INT16U reserved[3];
	} write_ack;
	struct {
		INT16U command;
		INT16U reserved[2];
		INT16U err;
	} cmd;
	struct {
		INT16U index;
		INT16U reserved[2];
		INT16U err;
	} oid2;
	struct {
		INT32U index:28;
		INT32U resv_b:4;
		INT32U reserved[1];
	} oid3_code;
	struct {
		INT32U x:14;
		INT32U y:14;
		INT32U resv_b:4;
		INT32U reserved[1];
	} oid3_pos;
	struct {
		INT8U reserved[7];
		INT8U type;
	} type;
	INT64U ddw64;
} OID3_READ_ST;


typedef struct
{
	INT32S module_status;
}OID_VAR;

#endif
