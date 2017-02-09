#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 150 "/usr/bin/../lib/gcc/msp430/4.6.3/include/stddef.h" 3
typedef long int ptrdiff_t;
#line 212
typedef unsigned int size_t;
#line 324
typedef int wchar_t;
# 8 "/usr/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
#line 8
  int dummy;
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
#line 13
  int dummy;
}  ;
#line 14
struct __nesc_attr_one_nok {
#line 14
  int dummy;
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
#line 17
  int dummy;
}  ;
# 38 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/stdint.h" 3
typedef signed char int8_t;
typedef int int16_t;
typedef long int int32_t;
__extension__ 
#line 41
typedef long long int int64_t;

typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long int uint32_t;
__extension__ 
#line 46
typedef unsigned long long int uint64_t;





typedef signed char int_least8_t;
typedef int int_least16_t;
typedef long int int_least32_t;
__extension__ 
#line 55
typedef long long int int_least64_t;


typedef unsigned char uint_least8_t;
typedef unsigned int uint_least16_t;
typedef unsigned long int uint_least32_t;
__extension__ 
#line 61
typedef unsigned long long int uint_least64_t;





typedef signed char int_fast8_t;
typedef int int_fast16_t;
typedef long int int_fast32_t;
__extension__ 
#line 70
typedef long long int int_fast64_t;


typedef unsigned char uint_fast8_t;
typedef unsigned int uint_fast16_t;
typedef unsigned long int uint_fast32_t;
__extension__ 
#line 76
typedef unsigned long long int uint_fast64_t;









typedef int16_t intptr_t;
typedef uint16_t uintptr_t;





__extension__ 
#line 93
typedef long long int intmax_t;
__extension__ 
#line 94
typedef unsigned long long int uintmax_t;
# 281 "/usr/lib/ncc/nesc_nx.h"
static __inline uint8_t __nesc_ntoh_uint8(const void * source)  ;




static __inline uint8_t __nesc_hton_uint8(void * target, uint8_t value)  ;
#line 315
static __inline uint16_t __nesc_hton_uint16(void * target, uint16_t value)  ;
#line 431
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 48 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/types.h" 3
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 44 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/string.h" 3
extern void *memset(void *arg_0x7f34485a0020, int arg_0x7f34485a02a0, size_t arg_0x7f34485a0560);
#line 65
extern void *memset(void *arg_0x7f3448588060, int arg_0x7f34485882e0, size_t arg_0x7f34485885a0);
# 62 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/stdlib.h" 3
#line 59
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;






#line 66
typedef struct __nesc_unnamed4243 {
  long int quot;
  long int rem;
} ldiv_t;
# 122 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
# 19 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/reent.h" 3
typedef unsigned long __ULong;
#line 31
struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *arg_0x7f3448549170);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x7f3448544480);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/math.h" 3
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;










extern double sin(double arg_0x7f344853b0d0);
#line 212
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 265
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 25 "/home/ezio/tinyos-main-read-only/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 26
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;
uint16_t TOS_NODE_ID = 1;






struct __nesc_attr_atmostonce {
};
#line 37
struct __nesc_attr_atleastonce {
};
#line 38
struct __nesc_attr_exactlyonce {
};
# 47 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/intrinsics.h" 3
void __nop(void );



void __dint(void );



void __eint(void );


unsigned int __read_status_register(void );


typedef unsigned int __istate_t;
# 164 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/msp430f1611.h" 3
extern volatile unsigned char ME1 __asm ("__""ME1");
#line 183
extern volatile unsigned char ME2 __asm ("__""ME2");
#line 195
extern volatile unsigned int WDTCTL __asm ("__""WDTCTL");
#line 267
extern volatile unsigned char P1OUT __asm ("__""P1OUT");

extern volatile unsigned char P1DIR __asm ("__""P1DIR");





extern volatile unsigned char P1IE __asm ("__""P1IE");

extern volatile unsigned char P1SEL __asm ("__""P1SEL");




extern volatile unsigned char P2OUT __asm ("__""P2OUT");

extern volatile unsigned char P2DIR __asm ("__""P2DIR");





extern volatile unsigned char P2IE __asm ("__""P2IE");

extern volatile unsigned char P2SEL __asm ("__""P2SEL");










extern volatile unsigned char P3OUT __asm ("__""P3OUT");

extern volatile unsigned char P3DIR __asm ("__""P3DIR");

extern volatile unsigned char P3SEL __asm ("__""P3SEL");




extern volatile unsigned char P4OUT __asm ("__""P4OUT");

extern volatile unsigned char P4DIR __asm ("__""P4DIR");

extern volatile unsigned char P4SEL __asm ("__""P4SEL");










extern volatile unsigned char P5OUT __asm ("__""P5OUT");

extern volatile unsigned char P5DIR __asm ("__""P5DIR");

extern volatile unsigned char P5SEL __asm ("__""P5SEL");




extern volatile unsigned char P6OUT __asm ("__""P6OUT");

extern volatile unsigned char P6DIR __asm ("__""P6DIR");

extern volatile unsigned char P6SEL __asm ("__""P6SEL");
#line 384
extern volatile unsigned char U0TCTL __asm ("__""U0TCTL");
#line 439
extern volatile unsigned char U1CTL __asm ("__""U1CTL");

extern volatile unsigned char U1TCTL __asm ("__""U1TCTL");



extern volatile unsigned char U1MCTL __asm ("__""U1MCTL");

extern volatile unsigned char U1BR0 __asm ("__""U1BR0");

extern volatile unsigned char U1BR1 __asm ("__""U1BR1");

extern const volatile unsigned char U1RXBUF __asm ("__""U1RXBUF");
#line 595
extern volatile unsigned int TACTL __asm ("__""TACTL");

extern volatile unsigned int TACCTL0 __asm ("__""TACCTL0");

extern volatile unsigned int TACCTL1 __asm ("__""TACCTL1");

extern volatile unsigned int TACCTL2 __asm ("__""TACCTL2");

extern volatile unsigned int TAR __asm ("__""TAR");
#line 720
extern volatile unsigned int TBCCTL0 __asm ("__""TBCCTL0");
#line 734
extern volatile unsigned int TBR __asm ("__""TBR");

extern volatile unsigned int TBCCR0 __asm ("__""TBCCR0");
#line 849
extern volatile unsigned char DCOCTL __asm ("__""DCOCTL");

extern volatile unsigned char BCSCTL1 __asm ("__""BCSCTL1");

extern volatile unsigned char BCSCTL2 __asm ("__""BCSCTL2");
#line 1021
extern volatile unsigned int ADC12CTL0 __asm ("__""ADC12CTL0");

extern volatile unsigned int ADC12CTL1 __asm ("__""ADC12CTL1");
# 343 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/msp430hardware.h"
static volatile uint8_t U0CTLnr __asm ("0x0070");
static volatile uint8_t I2CTCTLnr __asm ("0x0071");
static volatile uint8_t I2CDCTLnr __asm ("0x0072");
#line 378
typedef uint8_t mcu_power_t  ;
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)  ;


enum __nesc_unnamed4248 {
  MSP430_POWER_ACTIVE = 0, 
  MSP430_POWER_LPM0 = 1, 
  MSP430_POWER_LPM1 = 2, 
  MSP430_POWER_LPM2 = 3, 
  MSP430_POWER_LPM3 = 4, 
  MSP430_POWER_LPM4 = 5
};

static inline void __nesc_disable_interrupt(void )  ;





static inline void __nesc_enable_interrupt(void )  ;




typedef bool __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);






__nesc_atomic_t __nesc_atomic_start(void )   ;







void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)   ;
#line 433
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_float;typedef float __nesc_nxbase_nx_float  ;
#line 448
enum __nesc_unnamed4249 {
  MSP430_PORT_RESISTOR_INVALID, 
  MSP430_PORT_RESISTOR_OFF, 
  MSP430_PORT_RESISTOR_PULLDOWN, 
  MSP430_PORT_RESISTOR_PULLUP
};
# 8 "/home/ezio/tinyos-main-read-only/tos/platforms/telosb/hardware.h"
enum __nesc_unnamed4250 {
  TOS_SLEEP_NONE = MSP430_POWER_ACTIVE
};
#line 36
static inline void TOSH_SET_SIMO0_PIN()  ;
#line 36
static inline void TOSH_CLR_SIMO0_PIN()  ;
#line 36
static inline void TOSH_MAKE_SIMO0_OUTPUT()  ;
static inline void TOSH_SET_UCLK0_PIN()  ;
#line 37
static inline void TOSH_CLR_UCLK0_PIN()  ;
#line 37
static inline void TOSH_MAKE_UCLK0_OUTPUT()  ;
#line 79
enum __nesc_unnamed4251 {

  TOSH_HUMIDITY_ADDR = 5, 
  TOSH_HUMIDTEMP_ADDR = 3, 
  TOSH_HUMIDITY_RESET = 0x1E
};



static inline void TOSH_SET_FLASH_CS_PIN()  ;
#line 88
static inline void TOSH_CLR_FLASH_CS_PIN()  ;
#line 88
static inline void TOSH_MAKE_FLASH_CS_OUTPUT()  ;
static inline void TOSH_SET_FLASH_HOLD_PIN()  ;
#line 89
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT()  ;
# 58 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/chips/msp430/chip_thread.h"
#line 44
typedef struct thread_regs {
  uint16_t status;
  uint16_t r4;
  uint16_t r5;
  uint16_t r6;
  uint16_t r7;
  uint16_t r8;
  uint16_t r9;
  uint16_t r10;
  uint16_t r11;
  uint16_t r12;
  uint16_t r13;
  uint16_t r14;
  uint16_t r15;
} thread_regs_t;

typedef uint16_t *stack_ptr_t;
# 42 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/types/linked_list.h"
#line 39
typedef struct list_element {
  struct list_element *next;
  uint8_t *element_data;
} list_element_t;




#line 44
typedef struct linked_list {
  list_element_t *head;
  volatile uint8_t size;
} linked_list_t;
# 43 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/types/thread_queue.h"
#line 41
typedef struct thread_queue {
  linked_list_t l;
} thread_queue_t;
# 45 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/types/refcounter.h"
#line 42
typedef struct refcounter {
  uint8_t count;
  thread_queue_t thread_queue;
} refcounter_t;
# 42 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/types/thread.h"
typedef uint8_t thread_id_t;
typedef uint8_t syscall_id_t;
typedef thread_id_t tosthread_t;










enum __nesc_unnamed4252 {



  TOSTHREAD_MAX_NUM_THREADS = 33, 

  TOSTHREAD_NUM_STATIC_THREADS = 1U, 
  TOSTHREAD_MAX_DYNAMIC_THREADS = TOSTHREAD_MAX_NUM_THREADS - TOSTHREAD_NUM_STATIC_THREADS, 
  TOSTHREAD_TOS_THREAD_ID = TOSTHREAD_MAX_NUM_THREADS, 
  TOSTHREAD_INVALID_THREAD_ID = TOSTHREAD_MAX_NUM_THREADS, 
  TOSTHREAD_PREEMPTION_PERIOD = 5
};

enum __nesc_unnamed4253 {
  INVALID_ID = 0xFF, 
  SYSCALL_WAIT_ON_EVENT = 0
};

typedef struct syscall syscall_t;
typedef struct thread thread_t;
typedef struct init_block init_block_t;



struct init_block {
  void *globals;
  void (*init_ptr)(void *arg_0x7f344805c4c0);
  void *init_arg;
  refcounter_t thread_counter;
};


struct syscall {

  struct syscall *next_call;
  syscall_id_t id;
  thread_t *thread;
  void (*syscall_ptr)(struct syscall *arg_0x7f344805ace0);
  void *params;
};


struct thread {

  volatile struct thread *next_thread;
  thread_id_t id;
  init_block_t *init_block;
  stack_ptr_t stack_ptr;
  volatile uint8_t state;
  volatile uint8_t mutex_count;
  uint8_t joinedOnMe[(TOSTHREAD_MAX_NUM_THREADS - 1) / 8 + 1];
  void (*start_ptr)(void *arg_0x7f34480572f0);
  void *start_arg_ptr;
  syscall_t *syscall;
  thread_regs_t regs;
};

enum __nesc_unnamed4254 {
  TOSTHREAD_STATE_INACTIVE = 0, 
  TOSTHREAD_STATE_ACTIVE = 1, 
  TOSTHREAD_STATE_READY = 2, 
  TOSTHREAD_STATE_SUSPENDED = 3
};
# 51 "/home/ezio/tinyos-main-read-only/tos/types/TinyError.h"
enum __nesc_unnamed4255 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2)  ;
# 43 "/home/ezio/tinyos-main-read-only/tos/types/Leds.h"
enum __nesc_unnamed4256 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4257 {
  MSP430TIMER_CM_NONE = 0, 
  MSP430TIMER_CM_RISING = 1, 
  MSP430TIMER_CM_FALLING = 2, 
  MSP430TIMER_CM_BOTH = 3, 

  MSP430TIMER_STOP_MODE = 0, 
  MSP430TIMER_UP_MODE = 1, 
  MSP430TIMER_CONTINUOUS_MODE = 2, 
  MSP430TIMER_UPDOWN_MODE = 3, 

  MSP430TIMER_TACLK = 0, 
  MSP430TIMER_TBCLK = 0, 
  MSP430TIMER_ACLK = 1, 
  MSP430TIMER_SMCLK = 2, 
  MSP430TIMER_INCLK = 3, 

  MSP430TIMER_CLOCKDIV_1 = 0, 
  MSP430TIMER_CLOCKDIV_2 = 1, 
  MSP430TIMER_CLOCKDIV_4 = 2, 
  MSP430TIMER_CLOCKDIV_8 = 3
};
#line 75
#line 62
typedef struct __nesc_unnamed4258 {

  int ccifg : 1;
  int cov : 1;
  int out : 1;
  int cci : 1;
  int ccie : 1;
  int outmod : 3;
  int cap : 1;
  int clld : 2;
  int scs : 1;
  int ccis : 2;
  int cm : 2;
} msp430_compare_control_t;
#line 87
#line 77
typedef struct __nesc_unnamed4259 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} msp430_timer_a_control_t;
#line 102
#line 89
typedef struct __nesc_unnamed4260 {

  int tbifg : 1;
  int tbie : 1;
  int tbclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tbssel : 2;
  int _unused1 : 1;
  int cntl : 2;
  int tbclgrp : 2;
  int _unused2 : 1;
} msp430_timer_b_control_t;
# 41 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4261 {
#line 41
  int notUsed;
} 
#line 41
TSecond;
typedef struct __nesc_unnamed4262 {
#line 42
  int notUsed;
} 
#line 42
TMilli;
typedef struct __nesc_unnamed4263 {
#line 43
  int notUsed;
} 
#line 43
T32khz;
typedef struct __nesc_unnamed4264 {
#line 44
  int notUsed;
} 
#line 44
TMicro;
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/cc2420/CC2420.h"
typedef uint8_t cc2420_status_t;
#line 93
#line 87
typedef nx_struct security_header_t {
  unsigned char __nesc_filler0[1];


  nx_uint32_t frameCounter;
  nx_uint8_t keyID[1];
} __attribute__((packed)) security_header_t;
#line 113
#line 95
typedef nx_struct cc2420_header_t {
  nxle_uint8_t length;
  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;







  nxle_uint8_t network;


  nxle_uint8_t type;
} __attribute__((packed)) cc2420_header_t;





#line 118
typedef nx_struct cc2420_footer_t {
} __attribute__((packed)) cc2420_footer_t;
#line 143
#line 128
typedef nx_struct cc2420_metadata_t {
  nx_uint8_t rssi;
  nx_uint8_t lqi;
  nx_uint8_t tx_power;
  nx_bool crc;
  nx_bool ack;
  nx_bool timesync;
  nx_uint32_t timestamp;
  nx_uint16_t rxInterval;
} __attribute__((packed)) 





cc2420_metadata_t;





#line 146
typedef nx_struct cc2420_packet_t {
  cc2420_header_t packet;
  nx_uint8_t data[];
} __attribute__((packed)) cc2420_packet_t;
#line 179
enum __nesc_unnamed4265 {

  MAC_HEADER_SIZE = sizeof(cc2420_header_t ) - 1, 

  MAC_FOOTER_SIZE = sizeof(uint16_t ), 

  MAC_PACKET_SIZE = MAC_HEADER_SIZE + 28 + MAC_FOOTER_SIZE, 

  CC2420_SIZE = MAC_HEADER_SIZE + MAC_FOOTER_SIZE
};

enum cc2420_enums {
  CC2420_TIME_ACK_TURNAROUND = 7, 
  CC2420_TIME_VREN = 20, 
  CC2420_TIME_SYMBOL = 2, 
  CC2420_BACKOFF_PERIOD = 20 / CC2420_TIME_SYMBOL, 
  CC2420_MIN_BACKOFF = 20 / CC2420_TIME_SYMBOL, 
  CC2420_ACK_WAIT_DELAY = 256
};

enum cc2420_status_enums {
  CC2420_STATUS_RSSI_VALID = 1 << 1, 
  CC2420_STATUS_LOCK = 1 << 2, 
  CC2420_STATUS_TX_ACTIVE = 1 << 3, 
  CC2420_STATUS_ENC_BUSY = 1 << 4, 
  CC2420_STATUS_TX_UNDERFLOW = 1 << 5, 
  CC2420_STATUS_XOSC16M_STABLE = 1 << 6
};

enum cc2420_config_reg_enums {
  CC2420_SNOP = 0x00, 
  CC2420_SXOSCON = 0x01, 
  CC2420_STXCAL = 0x02, 
  CC2420_SRXON = 0x03, 
  CC2420_STXON = 0x04, 
  CC2420_STXONCCA = 0x05, 
  CC2420_SRFOFF = 0x06, 
  CC2420_SXOSCOFF = 0x07, 
  CC2420_SFLUSHRX = 0x08, 
  CC2420_SFLUSHTX = 0x09, 
  CC2420_SACK = 0x0a, 
  CC2420_SACKPEND = 0x0b, 
  CC2420_SRXDEC = 0x0c, 
  CC2420_STXENC = 0x0d, 
  CC2420_SAES = 0x0e, 
  CC2420_MAIN = 0x10, 
  CC2420_MDMCTRL0 = 0x11, 
  CC2420_MDMCTRL1 = 0x12, 
  CC2420_RSSI = 0x13, 
  CC2420_SYNCWORD = 0x14, 
  CC2420_TXCTRL = 0x15, 
  CC2420_RXCTRL0 = 0x16, 
  CC2420_RXCTRL1 = 0x17, 
  CC2420_FSCTRL = 0x18, 
  CC2420_SECCTRL0 = 0x19, 
  CC2420_SECCTRL1 = 0x1a, 
  CC2420_BATTMON = 0x1b, 
  CC2420_IOCFG0 = 0x1c, 
  CC2420_IOCFG1 = 0x1d, 
  CC2420_MANFIDL = 0x1e, 
  CC2420_MANFIDH = 0x1f, 
  CC2420_FSMTC = 0x20, 
  CC2420_MANAND = 0x21, 
  CC2420_MANOR = 0x22, 
  CC2420_AGCCTRL = 0x23, 
  CC2420_AGCTST0 = 0x24, 
  CC2420_AGCTST1 = 0x25, 
  CC2420_AGCTST2 = 0x26, 
  CC2420_FSTST0 = 0x27, 
  CC2420_FSTST1 = 0x28, 
  CC2420_FSTST2 = 0x29, 
  CC2420_FSTST3 = 0x2a, 
  CC2420_RXBPFTST = 0x2b, 
  CC2420_FMSTATE = 0x2c, 
  CC2420_ADCTST = 0x2d, 
  CC2420_DACTST = 0x2e, 
  CC2420_TOPTST = 0x2f, 
  CC2420_TXFIFO = 0x3e, 
  CC2420_RXFIFO = 0x3f
};

enum cc2420_ram_addr_enums {
  CC2420_RAM_TXFIFO = 0x000, 
  CC2420_RAM_RXFIFO = 0x080, 
  CC2420_RAM_KEY0 = 0x100, 
  CC2420_RAM_RXNONCE = 0x110, 
  CC2420_RAM_SABUF = 0x120, 
  CC2420_RAM_KEY1 = 0x130, 
  CC2420_RAM_TXNONCE = 0x140, 
  CC2420_RAM_CBCSTATE = 0x150, 
  CC2420_RAM_IEEEADR = 0x160, 
  CC2420_RAM_PANID = 0x168, 
  CC2420_RAM_SHORTADR = 0x16a
};

enum cc2420_nonce_enums {
  CC2420_NONCE_BLOCK_COUNTER = 0, 
  CC2420_NONCE_KEY_SEQ_COUNTER = 2, 
  CC2420_NONCE_FRAME_COUNTER = 3, 
  CC2420_NONCE_SOURCE_ADDRESS = 7, 
  CC2420_NONCE_FLAGS = 15
};

enum cc2420_main_enums {
  CC2420_MAIN_RESETn = 15, 
  CC2420_MAIN_ENC_RESETn = 14, 
  CC2420_MAIN_DEMOD_RESETn = 13, 
  CC2420_MAIN_MOD_RESETn = 12, 
  CC2420_MAIN_FS_RESETn = 11, 
  CC2420_MAIN_XOSC16M_BYPASS = 0
};

enum cc2420_mdmctrl0_enums {
  CC2420_MDMCTRL0_RESERVED_FRAME_MODE = 13, 
  CC2420_MDMCTRL0_PAN_COORDINATOR = 12, 
  CC2420_MDMCTRL0_ADR_DECODE = 11, 
  CC2420_MDMCTRL0_CCA_HYST = 8, 
  CC2420_MDMCTRL0_CCA_MOD = 6, 
  CC2420_MDMCTRL0_AUTOCRC = 5, 
  CC2420_MDMCTRL0_AUTOACK = 4, 
  CC2420_MDMCTRL0_PREAMBLE_LENGTH = 0
};

enum cc2420_mdmctrl1_enums {
  CC2420_MDMCTRL1_CORR_THR = 6, 
  CC2420_MDMCTRL1_DEMOD_AVG_MODE = 5, 
  CC2420_MDMCTRL1_MODULATION_MODE = 4, 
  CC2420_MDMCTRL1_TX_MODE = 2, 
  CC2420_MDMCTRL1_RX_MODE = 0
};

enum cc2420_rssi_enums {
  CC2420_RSSI_CCA_THR = 8, 
  CC2420_RSSI_RSSI_VAL = 0
};

enum cc2420_syncword_enums {
  CC2420_SYNCWORD_SYNCWORD = 0
};

enum cc2420_txctrl_enums {
  CC2420_TXCTRL_TXMIXBUF_CUR = 14, 
  CC2420_TXCTRL_TX_TURNAROUND = 13, 
  CC2420_TXCTRL_TXMIX_CAP_ARRAY = 11, 
  CC2420_TXCTRL_TXMIX_CURRENT = 9, 
  CC2420_TXCTRL_PA_CURRENT = 6, 
  CC2420_TXCTRL_RESERVED = 5, 
  CC2420_TXCTRL_PA_LEVEL = 0
};

enum cc2420_rxctrl0_enums {
  CC2420_RXCTRL0_RXMIXBUF_CUR = 12, 
  CC2420_RXCTRL0_HIGH_LNA_GAIN = 10, 
  CC2420_RXCTRL0_MED_LNA_GAIN = 8, 
  CC2420_RXCTRL0_LOW_LNA_GAIN = 6, 
  CC2420_RXCTRL0_HIGH_LNA_CURRENT = 4, 
  CC2420_RXCTRL0_MED_LNA_CURRENT = 2, 
  CC2420_RXCTRL0_LOW_LNA_CURRENT = 0
};

enum cc2420_rxctrl1_enums {
  CC2420_RXCTRL1_RXBPF_LOCUR = 13, 
  CC2420_RXCTRL1_RXBPF_MIDCUR = 12, 
  CC2420_RXCTRL1_LOW_LOWGAIN = 11, 
  CC2420_RXCTRL1_MED_LOWGAIN = 10, 
  CC2420_RXCTRL1_HIGH_HGM = 9, 
  CC2420_RXCTRL1_MED_HGM = 8, 
  CC2420_RXCTRL1_LNA_CAP_ARRAY = 6, 
  CC2420_RXCTRL1_RXMIX_TAIL = 4, 
  CC2420_RXCTRL1_RXMIX_VCM = 2, 
  CC2420_RXCTRL1_RXMIX_CURRENT = 0
};

enum cc2420_rsctrl_enums {
  CC2420_FSCTRL_LOCK_THR = 14, 
  CC2420_FSCTRL_CAL_DONE = 13, 
  CC2420_FSCTRL_CAL_RUNNING = 12, 
  CC2420_FSCTRL_LOCK_LENGTH = 11, 
  CC2420_FSCTRL_LOCK_STATUS = 10, 
  CC2420_FSCTRL_FREQ = 0
};

enum cc2420_secctrl0_enums {
  CC2420_SECCTRL0_RXFIFO_PROTECTION = 9, 
  CC2420_SECCTRL0_SEC_CBC_HEAD = 8, 
  CC2420_SECCTRL0_SEC_SAKEYSEL = 7, 
  CC2420_SECCTRL0_SEC_TXKEYSEL = 6, 
  CC2420_SECCTRL0_SEC_RXKEYSEL = 5, 
  CC2420_SECCTRL0_SEC_M = 2, 
  CC2420_SECCTRL0_SEC_MODE = 0
};

enum cc2420_secctrl1_enums {
  CC2420_SECCTRL1_SEC_TXL = 8, 
  CC2420_SECCTRL1_SEC_RXL = 0
};

enum cc2420_battmon_enums {
  CC2420_BATTMON_BATT_OK = 6, 
  CC2420_BATTMON_BATTMON_EN = 5, 
  CC2420_BATTMON_BATTMON_VOLTAGE = 0
};

enum cc2420_iocfg0_enums {
  CC2420_IOCFG0_BCN_ACCEPT = 11, 
  CC2420_IOCFG0_FIFO_POLARITY = 10, 
  CC2420_IOCFG0_FIFOP_POLARITY = 9, 
  CC2420_IOCFG0_SFD_POLARITY = 8, 
  CC2420_IOCFG0_CCA_POLARITY = 7, 
  CC2420_IOCFG0_FIFOP_THR = 0
};

enum cc2420_iocfg1_enums {
  CC2420_IOCFG1_HSSD_SRC = 10, 
  CC2420_IOCFG1_SFDMUX = 5, 
  CC2420_IOCFG1_CCAMUX = 0
};

enum cc2420_manfidl_enums {
  CC2420_MANFIDL_PARTNUM = 12, 
  CC2420_MANFIDL_MANFID = 0
};

enum cc2420_manfidh_enums {
  CC2420_MANFIDH_VERSION = 12, 
  CC2420_MANFIDH_PARTNUM = 0
};

enum cc2420_fsmtc_enums {
  CC2420_FSMTC_TC_RXCHAIN2RX = 13, 
  CC2420_FSMTC_TC_SWITCH2TX = 10, 
  CC2420_FSMTC_TC_PAON2TX = 6, 
  CC2420_FSMTC_TC_TXEND2SWITCH = 3, 
  CC2420_FSMTC_TC_TXEND2PAOFF = 0
};

enum cc2420_sfdmux_enums {
  CC2420_SFDMUX_SFD = 0, 
  CC2420_SFDMUX_XOSC16M_STABLE = 24
};

enum cc2420_security_enums {
  CC2420_NO_SEC = 0, 
  CC2420_CBC_MAC = 1, 
  CC2420_CTR = 2, 
  CC2420_CCM = 3, 
  NO_SEC = 0, 
  CBC_MAC_4 = 1, 
  CBC_MAC_8 = 2, 
  CBC_MAC_16 = 3, 
  CTR = 4, 
  CCM_4 = 5, 
  CCM_8 = 6, 
  CCM_16 = 7
};


enum __nesc_unnamed4266 {

  CC2420_INVALID_TIMESTAMP = 0x80000000L
};
# 6 "/home/ezio/tinyos-main-read-only/tos/types/AM.h"
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum __nesc_unnamed4267 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4268 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
# 83 "/home/ezio/tinyos-main-read-only/tos/lib/serial/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4269 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4270 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4271 {
  SERIAL_PROTO_ACK = 67, 
  SERIAL_PROTO_PACKET_ACK = 68, 
  SERIAL_PROTO_PACKET_NOACK = 69, 
  SERIAL_PROTO_PACKET_UNKNOWN = 255
};
#line 121
#line 109
typedef struct radio_stats {
  uint8_t version;
  uint8_t flags;
  uint8_t reserved;
  uint8_t platform;
  uint16_t MTU;
  uint16_t radio_crc_fail;
  uint16_t radio_queue_drops;
  uint16_t serial_crc_fail;
  uint16_t serial_tx_fail;
  uint16_t serial_short_packets;
  uint16_t serial_proto_drops;
} radio_stats_t;







#line 123
typedef nx_struct serial_header {
  nx_am_addr_t dest;
  nx_am_addr_t src;
  nx_uint8_t length;
  nx_am_group_t group;
  nx_am_id_t type;
} __attribute__((packed)) serial_header_t;




#line 131
typedef nx_struct serial_packet {
  serial_header_t header;
  nx_uint8_t data[];
} __attribute__((packed)) serial_packet_t;



#line 136
typedef nx_struct serial_metadata {
  nx_uint8_t ack;
} __attribute__((packed)) serial_metadata_t;
# 59 "/home/ezio/tinyos-main-read-only/tos/platforms/telosa/platform_message.h"
#line 56
typedef union message_header {
  cc2420_header_t cc2420;
  serial_header_t serial;
} message_header_t;



#line 61
typedef union TOSRadioFooter {
  cc2420_footer_t cc2420;
} message_footer_t;




#line 65
typedef union TOSRadioMetadata {
  cc2420_metadata_t cc2420;
  serial_metadata_t serial;
} message_metadata_t;
# 19 "/home/ezio/tinyos-main-read-only/tos/types/message.h"
#line 14
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[28];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute__((packed)) message_t;
# 43 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/types/syscall_queue.h"
#line 41
typedef struct syscall_queue {
  linked_list_t l;
} syscall_queue_t;
# 91 "/home/ezio/tinyos-main-read-only/tos/system/crc.h"
static uint16_t crcByte(uint16_t crc, uint8_t b);
# 56 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/msp430usart.h"
#line 48
typedef enum __nesc_unnamed4272 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;










#line 58
typedef struct __nesc_unnamed4273 {
  unsigned int swrst : 1;
  unsigned int mm : 1;
  unsigned int sync : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
} __attribute((packed))  msp430_uctl_t;









#line 69
typedef struct __nesc_unnamed4274 {
  unsigned int txept : 1;
  unsigned int stc : 1;
  unsigned int txwake : 1;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
} __attribute((packed))  msp430_utctl_t;










#line 79
typedef struct __nesc_unnamed4275 {
  unsigned int rxerr : 1;
  unsigned int rxwake : 1;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int brk : 1;
  unsigned int oe : 1;
  unsigned int pe : 1;
  unsigned int fe : 1;
} __attribute((packed))  msp430_urctl_t;
#line 116
#line 99
typedef struct __nesc_unnamed4276 {
  unsigned int ubr : 16;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int  : 3;

  unsigned int  : 1;
  unsigned int stc : 1;
  unsigned int  : 2;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
  unsigned int  : 0;
} msp430_spi_config_t;





#line 118
typedef struct __nesc_unnamed4277 {
  uint16_t ubr;
  uint8_t uctl;
  uint8_t utctl;
} msp430_spi_registers_t;




#line 124
typedef union __nesc_unnamed4278 {
  msp430_spi_config_t spiConfig;
  msp430_spi_registers_t spiRegisters;
} msp430_spi_union_config_t;
#line 169
#line 150
typedef enum __nesc_unnamed4279 {

  UBR_32KHZ_1200 = 0x001B, UMCTL_32KHZ_1200 = 0x94, 
  UBR_32KHZ_1800 = 0x0012, UMCTL_32KHZ_1800 = 0x84, 
  UBR_32KHZ_2400 = 0x000D, UMCTL_32KHZ_2400 = 0x6D, 
  UBR_32KHZ_4800 = 0x0006, UMCTL_32KHZ_4800 = 0x77, 
  UBR_32KHZ_9600 = 0x0003, UMCTL_32KHZ_9600 = 0x29, 

  UBR_1MHZ_1200 = 0x0369, UMCTL_1MHZ_1200 = 0x7B, 
  UBR_1MHZ_1800 = 0x0246, UMCTL_1MHZ_1800 = 0x55, 
  UBR_1MHZ_2400 = 0x01B4, UMCTL_1MHZ_2400 = 0xDF, 
  UBR_1MHZ_4800 = 0x00DA, UMCTL_1MHZ_4800 = 0xAA, 
  UBR_1MHZ_9600 = 0x006D, UMCTL_1MHZ_9600 = 0x44, 
  UBR_1MHZ_19200 = 0x0036, UMCTL_1MHZ_19200 = 0xB5, 
  UBR_1MHZ_38400 = 0x001B, UMCTL_1MHZ_38400 = 0x94, 
  UBR_1MHZ_57600 = 0x0012, UMCTL_1MHZ_57600 = 0x84, 
  UBR_1MHZ_76800 = 0x000D, UMCTL_1MHZ_76800 = 0x6D, 
  UBR_1MHZ_115200 = 0x0009, UMCTL_1MHZ_115200 = 0x10, 
  UBR_1MHZ_230400 = 0x0004, UMCTL_1MHZ_230400 = 0x55
} msp430_uart_rate_t;
#line 200
#line 171
typedef struct __nesc_unnamed4280 {
  unsigned int ubr : 16;

  unsigned int umctl : 8;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
  unsigned int  : 0;

  unsigned int  : 3;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int  : 1;

  unsigned int  : 2;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int  : 4;
  unsigned int  : 0;

  unsigned int utxe : 1;
  unsigned int urxe : 1;
} msp430_uart_config_t;








#line 202
typedef struct __nesc_unnamed4281 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl;
  uint8_t utctl;
  uint8_t urctl;
  uint8_t ume;
} msp430_uart_registers_t;




#line 211
typedef union __nesc_unnamed4282 {
  msp430_uart_config_t uartConfig;
  msp430_uart_registers_t uartRegisters;
} msp430_uart_union_config_t;

msp430_uart_union_config_t msp430_uart_default_config = { 
{ 
.utxe = 1, 
.urxe = 1, 
.ubr = UBR_1MHZ_57600, 
.umctl = UMCTL_1MHZ_57600, 
.ssel = 0x02, 
.pena = 0, 
.pev = 0, 
.spb = 0, 
.clen = 1, 
.listen = 0, 
.mm = 0, 
.ckpl = 0, 
.urxse = 0, 
.urxeie = 1, 
.urxwie = 0, 
.utxe = 1, 
.urxe = 1 } };
#line 248
#line 240
typedef struct __nesc_unnamed4283 {
  unsigned int i2cstt : 1;
  unsigned int i2cstp : 1;
  unsigned int i2cstb : 1;
  unsigned int i2cctrx : 1;
  unsigned int i2cssel : 2;
  unsigned int i2ccrm : 1;
  unsigned int i2cword : 1;
} __attribute((packed))  msp430_i2ctctl_t;
#line 276
#line 253
typedef struct __nesc_unnamed4284 {
  unsigned int  : 1;
  unsigned int mst : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int xa : 1;
  unsigned int  : 1;
  unsigned int txdmaen : 1;
  unsigned int rxdmaen : 1;

  unsigned int  : 4;
  unsigned int i2cssel : 2;
  unsigned int i2crm : 1;
  unsigned int i2cword : 1;

  unsigned int i2cpsc : 8;

  unsigned int i2csclh : 8;

  unsigned int i2cscll : 8;

  unsigned int i2coa : 10;
  unsigned int  : 6;
} msp430_i2c_config_t;








#line 278
typedef struct __nesc_unnamed4285 {
  uint8_t uctl;
  uint8_t i2ctctl;
  uint8_t i2cpsc;
  uint8_t i2csclh;
  uint8_t i2cscll;
  uint16_t i2coa;
} msp430_i2c_registers_t;




#line 287
typedef union __nesc_unnamed4286 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
#line 309
typedef uint8_t uart_speed_t;
typedef uint8_t uart_parity_t;
typedef uint8_t uart_duplex_t;

enum __nesc_unnamed4287 {
  TOS_UART_1200 = 0, 
  TOS_UART_1800 = 1, 
  TOS_UART_2400 = 2, 
  TOS_UART_4800 = 3, 
  TOS_UART_9600 = 4, 
  TOS_UART_19200 = 5, 
  TOS_UART_38400 = 6, 
  TOS_UART_57600 = 7, 
  TOS_UART_76800 = 8, 
  TOS_UART_115200 = 9, 
  TOS_UART_230400 = 10
};

enum __nesc_unnamed4288 {
  TOS_UART_OFF, 
  TOS_UART_RONLY, 
  TOS_UART_TONLY, 
  TOS_UART_DUPLEX
};

enum __nesc_unnamed4289 {
  TOS_UART_PARITY_NONE, 
  TOS_UART_PARITY_EVEN, 
  TOS_UART_PARITY_ODD
};
# 33 "/home/ezio/tinyos-main-read-only/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
# 44 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/types/mutex.h"
#line 41
typedef struct mutex {
  bool lock;
  thread_queue_t thread_queue;
} mutex_t;
typedef TMilli TinyThreadSchedulerP__PreemptionAlarm__precision_tag;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__0____nesc_unnamed4290 {
  Msp430Timer32khzC__0__ALARM_ID = 0U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC__0__to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC__0__from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__size_type;
typedef TMilli /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__precision_tag;
typedef /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__precision_tag /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__TimerFrom__precision_tag;
typedef /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__precision_tag /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__Timer__precision_tag;
typedef TMilli ThreadSleepP__TimerMilli__precision_tag;
typedef uint16_t TestSineSensorC__BlockingRead__val_t;
enum /*TestSineSensorAppC.MainThread*/ThreadC__0____nesc_unnamed4291 {
  ThreadC__0__THREAD_ID = 0U
};
typedef uint16_t /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__Read__val_t;
typedef uint16_t /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__BlockingRead__val_t;
typedef uint16_t /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__Read__val_t;
enum /*TestSineSensorAppC.BlockingSineSensorC*/BlockingSineSensorC__0____nesc_unnamed4292 {
  BlockingSineSensorC__0__ID = 0U
};
enum /*PlatformSerialC.UartC*/Msp430Uart1C__0____nesc_unnamed4293 {
  Msp430Uart1C__0__CLIENT_ID = 0U
};
typedef T32khz /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__precision_tag;
typedef uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__size_type;
enum /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0____nesc_unnamed4294 {
  Msp430Usart1C__0__CLIENT_ID = 0U
};
enum /*BlockingSerialActiveMessageC.BlockingStdControlC*/BlockingStdControlC__0____nesc_unnamed4295 {
  BlockingStdControlC__0__CLIENT_ID = 0U
};
typedef TMilli /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__Timer__precision_tag;
# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
#line 62
static error_t MotePlatformC__Init__init(void );
# 46 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 43
static void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );



static void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 42
static void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );





static void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 45
static void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 40
static void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__default__initClocks(void );
# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );
# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t Msp430ClockP__Init__init(void );
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(
# 51 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x7f3447f5e110);
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 51 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x7f3447f5e110);
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
static bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
# 44 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t time);
# 42 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 44 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t time);
# 42 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 44 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t time);
# 42 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 44 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t time);
# 42 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );
#line 47
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );
# 41 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t delta);
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 44 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t time);
# 42 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 44 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t time);
# 42 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void );
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 44 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t time);
# 42 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 44 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t time);
# 42 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 44 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t time);
# 42 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 44 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t time);
# 42 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 47 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
static error_t TinyThreadSchedulerP__ThreadScheduler__suspendCurrentThread(void );
#line 39
static uint8_t TinyThreadSchedulerP__ThreadScheduler__currentThreadId(void );
static thread_t *TinyThreadSchedulerP__ThreadScheduler__currentThreadInfo(void );



static error_t TinyThreadSchedulerP__ThreadScheduler__startThread(thread_id_t id);
#line 41
static thread_t *TinyThreadSchedulerP__ThreadScheduler__threadInfo(thread_id_t id);

static error_t TinyThreadSchedulerP__ThreadScheduler__initThread(thread_id_t id);






static error_t TinyThreadSchedulerP__ThreadScheduler__wakeupThread(thread_id_t id);
#line 48
static error_t TinyThreadSchedulerP__ThreadScheduler__interruptCurrentThread(void );
# 83 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static void TinyThreadSchedulerP__PreemptionAlarm__fired(void );
# 75 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static void TinyThreadSchedulerP__alarmTask__runTask(void );
# 60 "/home/ezio/tinyos-main-read-only/tos/interfaces/Boot.nc"
static void TinyThreadSchedulerP__ThreadSchedulerBoot__booted(void );
# 76 "/home/ezio/tinyos-main-read-only/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/BitArrayUtils.nc"
static void BitArrayUtilsC__BitArrayUtils__clrArray(uint8_t *array, uint8_t numBytes);
# 44 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/LinkedList.nc"
static error_t LinkedListC__LinkedList__addLast(linked_list_t *l, list_element_t *e);









static list_element_t *LinkedListC__LinkedList__removeAt(linked_list_t *l, uint8_t i);
#line 39
static void LinkedListC__LinkedList__init(linked_list_t *l);


static error_t LinkedListC__LinkedList__addAt(linked_list_t *l, list_element_t *e, uint8_t i);
static error_t LinkedListC__LinkedList__addFirst(linked_list_t *l, list_element_t *e);









static list_element_t *LinkedListC__LinkedList__remove(linked_list_t *l, list_element_t *e);
#line 50
static list_element_t *LinkedListC__LinkedList__getAfter(linked_list_t *l, list_element_t *e);





static list_element_t *LinkedListC__LinkedList__removeLast(linked_list_t *l);
#line 48
static list_element_t *LinkedListC__LinkedList__getFirst(linked_list_t *l);






static list_element_t *LinkedListC__LinkedList__removeFirst(linked_list_t *l);
#line 41
static uint8_t LinkedListC__LinkedList__size(linked_list_t *l);
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadQueue.nc"
static void ThreadQueueP__ThreadQueue__enqueue(thread_queue_t *q, thread_t *t);


static bool ThreadQueueP__ThreadQueue__isEmpty(thread_queue_t *q);
#line 39
static void ThreadQueueP__ThreadQueue__init(thread_queue_t *q);

static thread_t *ThreadQueueP__ThreadQueue__dequeue(thread_queue_t *q);
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 103 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type dt);
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );
# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 64 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
#line 64
static /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
# 109 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 103
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);
#line 116
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
# 82 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 75 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );
# 78 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
# 136 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
#line 129
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );
# 75 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
# 83 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );
#line 136
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__getNow(
# 48 "/home/ezio/tinyos-main-read-only/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x7f3447ba8020);
# 83 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(
# 48 "/home/ezio/tinyos-main-read-only/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x7f3447ba8020);
# 129 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShotAt(
# 48 "/home/ezio/tinyos-main-read-only/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x7f3447ba8020, 
# 129 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
uint32_t t0, uint32_t dt);
#line 73
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(
# 48 "/home/ezio/tinyos-main-read-only/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x7f3447ba8020, 
# 73 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
uint32_t dt);




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(
# 48 "/home/ezio/tinyos-main-read-only/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x7f3447ba8020);
# 82 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/Leds.nc"
static void LedsP__Leds__led0Toggle(void );
# 99 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc(void );
#line 99
static void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc(void );
#line 99
static void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc(void );
#line 58
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle(void );
#line 85
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );
#line 85
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );
#line 85
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );
# 42 "/home/ezio/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle(void );



static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );





static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );





static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
# 58 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/SchedulerBasicP.nc"
uint8_t arg_0x7f3448052020);
# 75 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 58 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/SchedulerBasicP.nc"
uint8_t arg_0x7f3448052020);
# 73 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/TaskScheduler.nc"
static bool SchedulerBasicP__TaskScheduler__hasTasks(void );
#line 58
static void SchedulerBasicP__TaskScheduler__init(void );
#line 80
static void SchedulerBasicP__TaskScheduler__taskLoop(void );
#line 66
static bool SchedulerBasicP__TaskScheduler__runNextTask(void );
# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/PlatformInterrupt.nc"
static void TOSThreadsInterruptP__PlatformInterrupt__postAmble(void );
# 60 "/home/ezio/tinyos-main-read-only/tos/interfaces/Boot.nc"
static void TinyOSMainP__TinyOSBoot__booted(void );
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t *TinyOSMainP__ThreadInfo__get(void );
#line 39
static error_t TinyOSMainP__ThreadInfo__reset(void );
# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadNotification.nc"
static void StaticThreadP__ThreadNotification__default__justCreated(
# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x7f3447868240);
# 38 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadNotification.nc"
static void StaticThreadP__ThreadNotification__default__aboutToDestroy(
# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x7f3447868240);
# 42 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/Thread.nc"
static void StaticThreadP__Thread__default__run(
# 38 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x7f344786ec60, 
# 42 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/Thread.nc"
void *arg);
#line 37
static error_t StaticThreadP__Thread__start(
# 38 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x7f344786ec60, 
# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/Thread.nc"
void *arg);
# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadCleanup.nc"
static void StaticThreadP__ThreadCleanup__cleanup(
# 46 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x7f3447862410);
# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadFunction.nc"
static void StaticThreadP__ThreadFunction__signalThreadRun(
# 45 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x7f3447864820, 
# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadFunction.nc"
void *arg);
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t *StaticThreadP__ThreadInfo__default__get(
# 44 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x7f34478659f0);
# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static error_t StaticThreadP__ThreadInfo__default__reset(
# 44 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x7f34478659f0);
# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadCleanup.nc"
static void ThreadMapP__DynamicThreadCleanup__default__cleanup(
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadMapP.nc"
uint8_t arg_0x7f3447832dc0);
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t *ThreadMapP__StaticThreadInfo__default__get(
# 43 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadMapP.nc"
uint8_t arg_0x7f3447831b60);
# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadCleanup.nc"
static void ThreadMapP__ThreadCleanup__cleanup(
# 45 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadMapP.nc"
uint8_t arg_0x7f344782f820);
# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadCleanup.nc"
static void ThreadMapP__StaticThreadCleanup__default__cleanup(
# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadMapP.nc"
uint8_t arg_0x7f3447832110);
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t *ThreadMapP__DynamicThreadInfo__default__get(
# 44 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadMapP.nc"
uint8_t arg_0x7f34478309b0);
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t *ThreadMapP__ThreadInfo__get(
# 38 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadMapP.nc"
uint8_t arg_0x7f3447833280);
# 75 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__updateFromTimer__runTask(void );
# 83 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__TimerFrom__fired(void );
#line 73
static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__Timer__startOneShot(
# 48 "/home/ezio/tinyos-main-read-only/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x7f3447ba8020, 
# 73 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
uint32_t dt);




static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__Timer__stop(
# 48 "/home/ezio/tinyos-main-read-only/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x7f3447ba8020);
# 83 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static void ThreadSleepP__TimerMilli__fired(
# 43 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadSleepP.nc"
uint8_t arg_0x7f34477edb70);
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCall.nc"
static error_t SystemCallP__SystemCall__finish(syscall_t *s);
#line 39
static error_t SystemCallP__SystemCall__start(void *syscall_ptr, syscall_t *s, syscall_id_t id, void *params);
# 75 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static void SystemCallP__threadTask__runTask(void );
# 60 "/home/ezio/tinyos-main-read-only/tos/interfaces/Boot.nc"
static void TestSineSensorC__Boot__booted(void );
# 42 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/Thread.nc"
static void TestSineSensorC__MainThread__run(void *arg);
# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__Init__init(void );
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t */*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__ThreadInfo__get(void );
#line 39
static error_t /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__ThreadInfo__reset(void );
# 55 "/home/ezio/tinyos-main-read-only/tos/interfaces/Read.nc"
static error_t /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__Read__default__read(
# 42 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingReadP.nc"
uint8_t arg_0x7f3447713e80);
# 63 "/home/ezio/tinyos-main-read-only/tos/interfaces/Read.nc"
static void /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__Read__readDone(
# 42 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingReadP.nc"
uint8_t arg_0x7f3447713e80, 
# 63 "/home/ezio/tinyos-main-read-only/tos/interfaces/Read.nc"
error_t result, /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__Read__val_t val);
# 43 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/BlockingRead.nc"
static error_t /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__BlockingRead__read(
# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingReadP.nc"
uint8_t arg_0x7f34477145d0, 
# 43 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/BlockingRead.nc"
/*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__BlockingRead__val_t *val);
# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCallQueue.nc"
static void SystemCallQueueP__SystemCallQueue__enqueue(syscall_queue_t *q, syscall_t *t);
#line 40
static void SystemCallQueueP__SystemCallQueue__init(syscall_queue_t *q);


static syscall_t *SystemCallQueueP__SystemCallQueue__remove(syscall_queue_t *q, syscall_t *t);

static syscall_t *SystemCallQueueP__SystemCallQueue__find(syscall_queue_t *q, syscall_id_t id);
# 75 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static void /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__readTask__runTask(void );
# 55 "/home/ezio/tinyos-main-read-only/tos/interfaces/Read.nc"
static error_t /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__Read__read(void );
# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__Init__init(void );
# 100 "/home/ezio/tinyos-main-read-only/tos/interfaces/Send.nc"
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(
#line 96
message_t * msg, 



error_t error);
# 78 "/home/ezio/tinyos-main-read-only/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 80 "/home/ezio/tinyos-main-read-only/tos/interfaces/AMSend.nc"
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(
# 47 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x7f344767ab30, 
# 80 "/home/ezio/tinyos-main-read-only/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 126 "/home/ezio/tinyos-main-read-only/tos/interfaces/Packet.nc"
static 
#line 123
void * 


/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__getPayload(
#line 121
message_t * msg, 




uint8_t len);
#line 106
static uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength(void );
# 78 "/home/ezio/tinyos-main-read-only/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(
# 48 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x7f3447677d10, 
# 71 "/home/ezio/tinyos-main-read-only/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 147 "/home/ezio/tinyos-main-read-only/tos/interfaces/AMPacket.nc"
static am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(
#line 143
message_t * amsg);
# 104 "/home/ezio/tinyos-main-read-only/tos/interfaces/SplitControl.nc"
static error_t SerialP__SplitControl__start(void );
# 75 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static void SerialP__stopDoneTask__runTask(void );
#line 75
static void SerialP__RunTx__runTask(void );
# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t SerialP__Init__init(void );
# 54 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialFlush.nc"
static void SerialP__SerialFlush__flushDone(void );
#line 49
static void SerialP__SerialFlush__default__flush(void );
# 75 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static void SerialP__startDoneTask__runTask(void );
# 94 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialFrameComm.nc"
static void SerialP__SerialFrameComm__dataReceived(uint8_t data);





static void SerialP__SerialFrameComm__putDone(void );
#line 85
static void SerialP__SerialFrameComm__delimiterReceived(void );
# 75 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static void SerialP__defaultSerialFlushTask__runTask(void );
# 71 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SendBytePacket.nc"
static error_t SerialP__SendBytePacket__completeSend(void );
#line 62
static error_t SerialP__SendBytePacket__startSend(uint8_t first_byte);
# 75 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask(void );
# 75 "/home/ezio/tinyos-main-read-only/tos/interfaces/Send.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(
# 51 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f344752e110, 
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 100
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(
# 51 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f344752e110, 
# 96 "/home/ezio/tinyos-main-read-only/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 75 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask(void );
# 78 "/home/ezio/tinyos-main-read-only/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(
# 50 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f344752f570, 
# 71 "/home/ezio/tinyos-main-read-only/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 31 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(
# 54 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f344752c4d0, 
# 31 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(
# 54 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f344752c4d0);
# 23 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(
# 54 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f344752c4d0, 
# 23 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t upperLen);
# 81 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SendBytePacket.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte(void );









static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error_t error);
# 62 "/home/ezio/tinyos-main-read-only/tos/lib/serial/ReceiveBytePacket.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket(void );






static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(uint8_t data);










static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(error_t result);
# 79 "/home/ezio/tinyos-main-read-only/tos/interfaces/UartStream.nc"
static void HdlcTranslateC__UartStream__receivedByte(uint8_t byte);
#line 99
static void HdlcTranslateC__UartStream__receiveDone(
#line 95
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void HdlcTranslateC__UartStream__sendDone(
#line 53
uint8_t * buf, 



uint16_t len, error_t error);
# 56 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialFrameComm.nc"
static error_t HdlcTranslateC__SerialFrameComm__putDelimiter(void );
#line 79
static void HdlcTranslateC__SerialFrameComm__resetReceive(void );
#line 65
static error_t HdlcTranslateC__SerialFrameComm__putData(uint8_t data);
# 65 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(
# 44 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f3447424a10);
# 59 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(
# 44 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f3447424a10);
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(
# 49 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f344741e020);
# 48 "/home/ezio/tinyos-main-read-only/tos/interfaces/UartStream.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__send(
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f3447423840, 
# 44 "/home/ezio/tinyos-main-read-only/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len);
#line 79
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f3447423840, 
# 79 "/home/ezio/tinyos-main-read-only/tos/interfaces/UartStream.nc"
uint8_t byte);
#line 99
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f3447423840, 
# 95 "/home/ezio/tinyos-main-read-only/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f3447423840, 
# 53 "/home/ezio/tinyos-main-read-only/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
# 82 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Counter.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow(void );
# 120 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__release(
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f3447421cb0);
# 97 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f3447421cb0);
# 102 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f3447421cb0);
# 128 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f3447421cb0);
# 120 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__release(
# 43 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f3447425790);
# 97 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(
# 43 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f3447425790);
# 102 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(
# 43 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f3447425790);
# 54 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(
# 51 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f34473fd1c0, 
# 54 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(
# 51 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f34473fd1c0);
# 143 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430Usart1P__Usart__enableUartRx(void );
#line 123
static void HplMsp430Usart1P__Usart__enableUart(void );
#line 97
static void HplMsp430Usart1P__Usart__resetUsart(bool reset);
#line 179
static void HplMsp430Usart1P__Usart__disableIntr(void );
#line 90
static void HplMsp430Usart1P__Usart__setUmctl(uint8_t umctl);
#line 133
static void HplMsp430Usart1P__Usart__enableUartTx(void );
#line 148
static void HplMsp430Usart1P__Usart__disableUartRx(void );
#line 182
static void HplMsp430Usart1P__Usart__enableIntr(void );
#line 207
static void HplMsp430Usart1P__Usart__clrIntr(void );
#line 80
static void HplMsp430Usart1P__Usart__setUbr(uint16_t ubr);
#line 224
static void HplMsp430Usart1P__Usart__tx(uint8_t data);
#line 128
static void HplMsp430Usart1P__Usart__disableUart(void );
#line 174
static void HplMsp430Usart1P__Usart__setModeUart(msp430_uart_union_config_t *config);
#line 158
static void HplMsp430Usart1P__Usart__disableSpi(void );
#line 138
static void HplMsp430Usart1P__Usart__disableUartTx(void );
# 95 "/home/ezio/tinyos-main-read-only/tos/interfaces/AsyncStdControl.nc"
static error_t HplMsp430Usart1P__AsyncStdControl__start(void );









static error_t HplMsp430Usart1P__AsyncStdControl__stop(void );
# 54 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x7f34472d5060, 
# 54 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x7f34472d5060);
# 54 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );
# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void );
# 53 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceQueue.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );
#line 70
static resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 61 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(
# 55 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
uint8_t arg_0x7f3447293840);
# 65 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(
# 60 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
uint8_t arg_0x7f3447290c40);
# 59 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(
# 60 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
uint8_t arg_0x7f3447290c40);
# 56 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
# 120 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(
# 54 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
uint8_t arg_0x7f34472944b0);
# 97 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(
# 54 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
uint8_t arg_0x7f34472944b0);
# 102 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(
# 54 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
uint8_t arg_0x7f34472944b0);
# 128 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(
# 54 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
uint8_t arg_0x7f34472944b0);
# 90 "/home/ezio/tinyos-main-read-only/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );
# 75 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
# 62 "/home/ezio/tinyos-main-read-only/tos/lib/power/PowerDownCleanup.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup(void );
# 46 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void );
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t *TelosSerialP__Msp430UartConfigure__getConfig(void );
# 102 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
static void TelosSerialP__Resource__granted(void );
# 95 "/home/ezio/tinyos-main-read-only/tos/interfaces/StdControl.nc"
static error_t TelosSerialP__StdControl__start(void );









static error_t TelosSerialP__StdControl__stop(void );
# 31 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t SerialPacketInfoActiveMessageP__Info__offset(void );







static uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen);
# 113 "/home/ezio/tinyos-main-read-only/tos/interfaces/SplitControl.nc"
static void BlockingStdControlImplP__SplitControl__startDone(
# 42 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingStdControlImplP.nc"
uint8_t arg_0x7f34471dba90, 
# 113 "/home/ezio/tinyos-main-read-only/tos/interfaces/SplitControl.nc"
error_t error);
#line 138
static void BlockingStdControlImplP__SplitControl__stopDone(
# 42 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingStdControlImplP.nc"
uint8_t arg_0x7f34471dba90, 
# 138 "/home/ezio/tinyos-main-read-only/tos/interfaces/SplitControl.nc"
error_t error);
#line 104
static error_t BlockingStdControlImplP__SplitControl__default__start(
# 42 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingStdControlImplP.nc"
uint8_t arg_0x7f34471dba90);
# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t BlockingStdControlImplP__Init__init(void );
# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/BlockingStdControl.nc"
static error_t BlockingStdControlImplP__BlockingStdControl__start(
# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingStdControlImplP.nc"
uint8_t arg_0x7f34471dccd0);
# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__Init__init(void );
# 78 "/home/ezio/tinyos-main-read-only/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__Receive__receive(
# 45 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingAMReceiverImplP.nc"
uint8_t arg_0x7f344719d020, 
# 71 "/home/ezio/tinyos-main-read-only/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 83 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static void /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__Timer__fired(
# 44 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingAMReceiverImplP.nc"
uint8_t arg_0x7f34471a47a0);
# 110 "/home/ezio/tinyos-main-read-only/tos/interfaces/AMSend.nc"
static void /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__AMSend__sendDone(
# 45 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingAMSenderImplP.nc"
am_id_t arg_0x7f3447145ce0, 
# 103 "/home/ezio/tinyos-main-read-only/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__Init__init(void );
# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/BlockingAMSend.nc"
static error_t /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__BlockingAMSend__send(
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingAMSenderImplP.nc"
am_id_t arg_0x7f3447149a10, 
# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/BlockingAMSend.nc"
am_addr_t addr, message_t *msg, uint8_t len);
# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/Mutex.nc"
static error_t MutexP__Mutex__unlock(mutex_t *m);
#line 40
static error_t MutexP__Mutex__lock(mutex_t *m);
#line 39
static void MutexP__Mutex__init(mutex_t *m);
# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t PlatformP__MoteInit__init(void );
#line 62
static error_t PlatformP__MoteClockInit__init(void );
#line 62
static error_t PlatformP__LedsInit__init(void );
# 10 "/home/ezio/tinyos-main-read-only/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void );
# 6 "/home/ezio/tinyos-main-read-only/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__uwait(uint16_t u);




static __inline void MotePlatformC__TOSH_wait(void );




static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set);










static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void );
#line 56
static inline error_t MotePlatformC__Init__init(void );
# 43 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__initTimerB(void );
#line 42
static void Msp430ClockP__Msp430ClockInit__initTimerA(void );
#line 40
static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__initClocks(void );
# 51 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP__IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP__TACTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP__TAIV __asm ("0x012E");
static volatile uint16_t Msp430ClockP__TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP__TBIV __asm ("0x011E");

enum Msp430ClockP____nesc_unnamed4296 {

  Msp430ClockP__ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP__TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP__ACLK_CALIB_PERIOD
};

static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );



static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 79
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 100
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 115
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 130
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );





static inline void Msp430ClockP__startTimerA(void );
#line 163
static inline void Msp430ClockP__startTimerB(void );
#line 175
static void Msp430ClockP__set_dco_calib(int calib);





static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib);
#line 204
static inline void Msp430ClockP__busyCalibrateDco(void );
#line 229
static inline error_t Msp430ClockP__Init__init(void );
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(
# 51 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x7f3447f5e110);
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void );
# 126 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n);
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(
# 51 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x7f3447f5e110);
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void );
# 62 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
#line 81
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
#line 126
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n);
# 86 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time);
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void );
# 55 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t;


static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 86 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time);
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void );
# 55 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t;


static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 86 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time);
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void );
# 55 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 86 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time);
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void );
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void );
# 55 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void );
#line 85
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );
#line 130
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x);
#line 180
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 86 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time);
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void );
# 55 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t;


static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 86 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time);
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void );
# 55 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t;


static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 86 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time);
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void );
# 55 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 86 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time);
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void );
# 55 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 86 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time);
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void );
# 55 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 86 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time);
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void );
# 55 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP__VectorTimerB1__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerA0__fired(void );
# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/PlatformInterrupt.nc"
static void Msp430TimerCommonP__PlatformInterrupt__postAmble(void );
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP__VectorTimerA1__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerB0__fired(void );
# 12 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/chips/msp430/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x000C)))  ;



void sig_TIMERA1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x000A)))  ;



void sig_TIMERB0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x001A)))  ;



void sig_TIMERB1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0018)))  ;
# 73 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static void TinyThreadSchedulerP__PreemptionAlarm__startOneShot(uint32_t dt);




static void TinyThreadSchedulerP__PreemptionAlarm__stop(void );
# 60 "/home/ezio/tinyos-main-read-only/tos/interfaces/Boot.nc"
static void TinyThreadSchedulerP__TinyOSBoot__booted(void );
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadQueue.nc"
static void TinyThreadSchedulerP__ThreadQueue__enqueue(thread_queue_t *q, thread_t *t);


static bool TinyThreadSchedulerP__ThreadQueue__isEmpty(thread_queue_t *q);
#line 39
static void TinyThreadSchedulerP__ThreadQueue__init(thread_queue_t *q);

static thread_t *TinyThreadSchedulerP__ThreadQueue__dequeue(thread_queue_t *q);
# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/BitArrayUtils.nc"
static void TinyThreadSchedulerP__BitArrayUtils__clrArray(uint8_t *array, uint8_t numBytes);
# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadCleanup.nc"
static void TinyThreadSchedulerP__ThreadCleanup__cleanup(
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
uint8_t arg_0x7f3447e63710);
# 76 "/home/ezio/tinyos-main-read-only/tos/interfaces/McuSleep.nc"
static void TinyThreadSchedulerP__McuSleep__sleep(void );
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t *TinyThreadSchedulerP__ThreadInfo__get(
# 44 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
uint8_t arg_0x7f3447e62af0);
#line 64
enum TinyThreadSchedulerP____nesc_unnamed4297 {
#line 64
  TinyThreadSchedulerP__alarmTask = 0U
};
#line 64
typedef int TinyThreadSchedulerP____nesc_sillytask_alarmTask[TinyThreadSchedulerP__alarmTask];
#line 54
thread_t *TinyThreadSchedulerP__current_thread;

thread_t *TinyThreadSchedulerP__tos_thread;

thread_t *TinyThreadSchedulerP__yielding_thread;

uint8_t TinyThreadSchedulerP__num_runnable_threads;

thread_queue_t TinyThreadSchedulerP__ready_queue;

static inline void TinyThreadSchedulerP__alarmTask__runTask(void );
#line 84
static void TinyThreadSchedulerP__switchThreads(void ) __attribute((noinline)) ;


static void TinyThreadSchedulerP__restoreThread(void ) __attribute((noinline)) ;









static void TinyThreadSchedulerP__sleepWhileIdle(void );
#line 111
static void TinyThreadSchedulerP__scheduleNextThread(void );
#line 124
static void TinyThreadSchedulerP__interrupt(thread_t *thread);
#line 138
static inline void TinyThreadSchedulerP__suspend(thread_t *thread);










static inline void TinyThreadSchedulerP__wakeupJoined(thread_t *t);
#line 171
static inline void TinyThreadSchedulerP__stop(thread_t *t);
#line 186
static void TinyThreadSchedulerP__threadWrapper(void ) __attribute((noinline)) __attribute((naked)) ;
#line 201
static inline void TinyThreadSchedulerP__ThreadSchedulerBoot__booted(void );
#line 213
static inline error_t TinyThreadSchedulerP__ThreadScheduler__initThread(uint8_t id);








static inline error_t TinyThreadSchedulerP__ThreadScheduler__startThread(uint8_t id);
#line 253
static error_t TinyThreadSchedulerP__ThreadScheduler__suspendCurrentThread(void );










static error_t TinyThreadSchedulerP__ThreadScheduler__interruptCurrentThread(void );
#line 291
static error_t TinyThreadSchedulerP__ThreadScheduler__wakeupThread(uint8_t id);
#line 307
static inline uint8_t TinyThreadSchedulerP__ThreadScheduler__currentThreadId(void );



static thread_t *TinyThreadSchedulerP__ThreadScheduler__threadInfo(uint8_t id);



static inline thread_t *TinyThreadSchedulerP__ThreadScheduler__currentThreadInfo(void );



static inline void TinyThreadSchedulerP__PreemptionAlarm__fired(void );
# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void );
# 59 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/McuSleepC.nc"
bool McuSleepC__dirty = TRUE;
mcu_power_t McuSleepC__powerState = MSP430_POWER_ACTIVE;




const uint16_t McuSleepC__msp430PowerBits[MSP430_POWER_LPM4 + 1] = { 
0, 
0x0010, 
0x0040 + 0x0010, 
0x0080 + 0x0010, 
0x0080 + 0x0040 + 0x0010, 
0x0080 + 0x0040 + 0x0020 + 0x0010 };


static inline mcu_power_t McuSleepC__getPowerState(void );
#line 112
static inline void McuSleepC__computePowerState(void );




static inline void McuSleepC__McuSleep__sleep(void );
# 49 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BitArrayUtilsC.nc"
static inline void BitArrayUtilsC__BitArrayUtils__clrArray(uint8_t *array, uint8_t size);
# 42 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/LinkedListC.nc"
static list_element_t *LinkedListC__get_elementAt(linked_list_t *l, uint8_t i);
#line 54
static inline list_element_t *LinkedListC__get_element(linked_list_t *l, list_element_t *e);








static inline list_element_t *LinkedListC__get_element_before(linked_list_t *l, list_element_t *e);
#line 84
static error_t LinkedListC__insert_element(linked_list_t *l, list_element_t **previous_next, list_element_t *e);







static list_element_t *LinkedListC__remove_element(linked_list_t *l, list_element_t **previous_next);







static inline void LinkedListC__LinkedList__init(linked_list_t *l);










static inline uint8_t LinkedListC__LinkedList__size(linked_list_t *l);


static inline error_t LinkedListC__LinkedList__addFirst(linked_list_t *l, list_element_t *e);


static list_element_t *LinkedListC__LinkedList__getFirst(linked_list_t *l);



static inline list_element_t *LinkedListC__LinkedList__removeFirst(linked_list_t *l);



static inline error_t LinkedListC__LinkedList__addLast(linked_list_t *l, list_element_t *e);





static inline list_element_t *LinkedListC__LinkedList__removeLast(linked_list_t *l);


static inline error_t LinkedListC__LinkedList__addAt(linked_list_t *l, list_element_t *e, uint8_t i);
#line 148
static list_element_t *LinkedListC__LinkedList__removeAt(linked_list_t *l, uint8_t i);
#line 171
static list_element_t *LinkedListC__LinkedList__getAfter(linked_list_t *l, list_element_t *e);










static list_element_t *LinkedListC__LinkedList__remove(linked_list_t *l, list_element_t *e);
# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/LinkedList.nc"
static void ThreadQueueP__LinkedList__init(linked_list_t *l);



static error_t ThreadQueueP__LinkedList__addFirst(linked_list_t *l, list_element_t *e);
#line 56
static list_element_t *ThreadQueueP__LinkedList__removeLast(linked_list_t *l);
#line 41
static uint8_t ThreadQueueP__LinkedList__size(linked_list_t *l);
# 45 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadQueueP.nc"
static inline void ThreadQueueP__ThreadQueue__init(thread_queue_t *q);


static inline void ThreadQueueP__ThreadQueue__enqueue(thread_queue_t *q, thread_t *t);


static inline thread_t *ThreadQueueP__ThreadQueue__dequeue(thread_queue_t *q);





static inline bool ThreadQueueP__ThreadQueue__isEmpty(thread_queue_t *q);
# 41 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta);
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void );
# 78 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void );
# 57 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void );
#line 47
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void );
#line 44
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 53 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
#line 65
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 114
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void );
# 82 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void );
# 49 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 64 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void );
# 67 "/home/ezio/tinyos-main-read-only/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC__0____nesc_unnamed4298 {

  TransformCounterC__0__LOW_SHIFT_RIGHT = 5, 
  TransformCounterC__0__HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT, 
  TransformCounterC__0__NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) + 5, 



  TransformCounterC__0__OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
#line 133
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
# 78 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void );
#line 103
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt);
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void );
# 64 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void );
# 77 "/home/ezio/tinyos-main-read-only/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0____nesc_unnamed4299 {

  TransformAlarmC__0__MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type ) - 1 - 5, 
  TransformAlarmC__0__MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void );
#line 147
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt);
#line 162
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
#line 177
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void );
# 109 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void );
#line 103
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt);
#line 116
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void );
#line 73
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void );
# 83 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void );
# 74 "/home/ezio/tinyos-main-read-only/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_unnamed4300 {
#line 74
  AlarmToTimerC__0__fired = 1U
};
#line 74
typedef int /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired];
#line 55
uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt;
bool /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot);
#line 71
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );


static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
#line 93
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void );
# 136 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void );
#line 129
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(
# 48 "/home/ezio/tinyos-main-read-only/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x7f3447ba8020);
#line 71
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4301 {
#line 71
  VirtualizeTimerC__0__updateFromTimer = 2U
};
#line 71
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer];
#line 53
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4302 {

  VirtualizeTimerC__0__NUM_TIMERS = 2U, 
  VirtualizeTimerC__0__END_OF_LIST = 255
};








#line 59
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4303 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now);
#line 100
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
#line 139
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);
#line 159
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num);
#line 184
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShotAt(uint8_t num, uint32_t t0, uint32_t dt);




static inline uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__getNow(uint8_t num);
#line 204
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num);
# 58 "/home/ezio/tinyos-main-read-only/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 42 "/home/ezio/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
static void LedsP__Led0__toggle(void );



static void LedsP__Led0__makeOutput(void );
#line 40
static void LedsP__Led0__set(void );





static void LedsP__Led1__makeOutput(void );
#line 40
static void LedsP__Led1__set(void );





static void LedsP__Led2__makeOutput(void );
#line 40
static void LedsP__Led2__set(void );
# 56 "/home/ezio/tinyos-main-read-only/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void );
#line 84
static inline void LedsP__Leds__led0Toggle(void );
# 65 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc(void );
#line 65
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc(void );
#line 67
static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc(void );
#line 67
static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc(void );
#line 67
static inline void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );

static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle(void );




static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
# 58 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__toggle(void );
#line 85
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void );
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );

static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle(void );



static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
# 85 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void );
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );





static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
# 85 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void );
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );





static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
# 47 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
static error_t SchedulerBasicP__ThreadScheduler__suspendCurrentThread(void );
# 75 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(
# 58 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/SchedulerBasicP.nc"
uint8_t arg_0x7f3448052020);




enum SchedulerBasicP____nesc_unnamed4304 {
  SchedulerBasicP__NUM_TASKS = 13U, 
  SchedulerBasicP__NO_TASK = 255
};

volatile uint8_t SchedulerBasicP__m_head;
volatile uint8_t SchedulerBasicP__m_tail;
volatile uint8_t SchedulerBasicP__m_next[SchedulerBasicP__NUM_TASKS];








static __inline uint8_t SchedulerBasicP__popTask(void );
#line 94
static inline bool SchedulerBasicP__isWaiting(uint8_t id);



static inline bool SchedulerBasicP__TaskScheduler__hasTasks(void );



static inline bool SchedulerBasicP__pushTask(uint8_t id);
#line 119
static inline void SchedulerBasicP__TaskScheduler__init(void );







static bool SchedulerBasicP__TaskScheduler__runNextTask(void );
#line 139
static inline void SchedulerBasicP__TaskScheduler__taskLoop(void );
#line 156
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id);



static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id);
# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
static uint8_t TOSThreadsInterruptP__ThreadScheduler__currentThreadId(void );










static error_t TOSThreadsInterruptP__ThreadScheduler__wakeupThread(thread_id_t id);
#line 48
static error_t TOSThreadsInterruptP__ThreadScheduler__interruptCurrentThread(void );
# 73 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/TaskScheduler.nc"
static bool TOSThreadsInterruptP__TaskScheduler__hasTasks(void );
# 46 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TOSThreadsInterruptP.nc"
static void TOSThreadsInterruptP__interruptThread(void ) __attribute((noinline)) ;





static __inline void TOSThreadsInterruptP__PlatformInterrupt__postAmble(void );
# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t TinyOSMainP__SoftwareInit__init(void );
# 60 "/home/ezio/tinyos-main-read-only/tos/interfaces/Boot.nc"
static void TinyOSMainP__Boot__booted(void );
# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t TinyOSMainP__PlatformInit__init(void );
# 58 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/TaskScheduler.nc"
static void TinyOSMainP__TaskScheduler__init(void );
#line 80
static void TinyOSMainP__TaskScheduler__taskLoop(void );
#line 66
static bool TinyOSMainP__TaskScheduler__runNextTask(void );
# 76 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyOSMainP.nc"
thread_t TinyOSMainP__thread_info;

static inline void TinyOSMainP__TinyOSBoot__booted(void );
#line 114
static inline error_t TinyOSMainP__ThreadInfo__reset(void );



static inline thread_t *TinyOSMainP__ThreadInfo__get(void );
# 60 "/home/ezio/tinyos-main-read-only/tos/interfaces/Boot.nc"
static void RealMainImplP__ThreadSchedulerBoot__booted(void );
# 60 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/RealMainImplP.nc"
int main(void )   ;
# 44 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
static error_t StaticThreadP__ThreadScheduler__startThread(thread_id_t id);
#line 43
static error_t StaticThreadP__ThreadScheduler__initThread(thread_id_t id);
# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadNotification.nc"
static void StaticThreadP__ThreadNotification__justCreated(
# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x7f3447868240);
# 38 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadNotification.nc"
static void StaticThreadP__ThreadNotification__aboutToDestroy(
# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x7f3447868240);
# 42 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/Thread.nc"
static void StaticThreadP__Thread__run(
# 38 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x7f344786ec60, 
# 42 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/Thread.nc"
void *arg);
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t *StaticThreadP__ThreadInfo__get(
# 44 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x7f34478659f0);
# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static error_t StaticThreadP__ThreadInfo__reset(
# 44 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x7f34478659f0);







static inline error_t StaticThreadP__init(uint8_t id, void *arg);










static inline error_t StaticThreadP__Thread__start(uint8_t id, void *arg);
#line 97
static inline void StaticThreadP__ThreadFunction__signalThreadRun(uint8_t id, void *arg);



static inline void StaticThreadP__ThreadCleanup__cleanup(uint8_t id);



static inline void StaticThreadP__Thread__default__run(uint8_t id, void *arg);
static inline thread_t *StaticThreadP__ThreadInfo__default__get(uint8_t id);
static inline error_t StaticThreadP__ThreadInfo__default__reset(uint8_t id);
static inline void StaticThreadP__ThreadNotification__default__justCreated(uint8_t id);
static inline void StaticThreadP__ThreadNotification__default__aboutToDestroy(uint8_t id);
# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadCleanup.nc"
static void ThreadMapP__DynamicThreadCleanup__cleanup(
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadMapP.nc"
uint8_t arg_0x7f3447832dc0);
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t *ThreadMapP__StaticThreadInfo__get(
# 43 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadMapP.nc"
uint8_t arg_0x7f3447831b60);
# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadCleanup.nc"
static void ThreadMapP__StaticThreadCleanup__cleanup(
# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadMapP.nc"
uint8_t arg_0x7f3447832110);
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t *ThreadMapP__DynamicThreadInfo__get(
# 44 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadMapP.nc"
uint8_t arg_0x7f34478309b0);





static inline thread_t *ThreadMapP__ThreadInfo__get(uint8_t id);





static inline thread_t *ThreadMapP__StaticThreadInfo__default__get(uint8_t id);





static thread_t *ThreadMapP__DynamicThreadInfo__default__get(uint8_t id);





static inline void ThreadMapP__ThreadCleanup__cleanup(uint8_t id);


static inline void ThreadMapP__StaticThreadCleanup__default__cleanup(uint8_t id);


static void ThreadMapP__DynamicThreadCleanup__default__cleanup(uint8_t id);
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static error_t /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__updateFromTimer__postTask(void );
# 136 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static uint32_t /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__TimerFrom__getNow(void );
#line 129
static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__TimerFrom__stop(void );




static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__Timer__fired(
# 48 "/home/ezio/tinyos-main-read-only/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x7f3447ba8020);
#line 71
enum /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1____nesc_unnamed4305 {
#line 71
  VirtualizeTimerC__1__updateFromTimer = 3U
};
#line 71
typedef int /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1____nesc_sillytask_updateFromTimer[/*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__updateFromTimer];
#line 53
enum /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1____nesc_unnamed4306 {

  VirtualizeTimerC__1__NUM_TIMERS = 33, 
  VirtualizeTimerC__1__END_OF_LIST = 255
};








#line 59
typedef struct /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1____nesc_unnamed4307 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__Timer_t;

/*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__Timer_t /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__m_timers[/*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__NUM_TIMERS];




static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__fireTimers(uint32_t now);
#line 100
static inline void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__updateFromTimer__runTask(void );
#line 139
static inline void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__TimerFrom__fired(void );




static inline void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);
#line 159
static inline void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__Timer__startOneShot(uint8_t num, uint32_t dt);




static inline void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__Timer__stop(uint8_t num);
# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
static thread_t *ThreadSleepP__ThreadScheduler__threadInfo(thread_id_t id);
# 73 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static void ThreadSleepP__TimerMilli__startOneShot(
# 43 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadSleepP.nc"
uint8_t arg_0x7f34477edb70, 
# 73 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
uint32_t dt);
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCall.nc"
static error_t ThreadSleepP__SystemCall__finish(syscall_t *s);
# 50 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadSleepP.nc"
#line 48
typedef struct ThreadSleepP__sleep_params {
  uint32_t *milli;
} ThreadSleepP__sleep_params_t;

static inline void ThreadSleepP__sleepTask(syscall_t *s);
#line 76
static inline void ThreadSleepP__TimerMilli__fired(uint8_t id);
# 47 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
static error_t SystemCallP__ThreadScheduler__suspendCurrentThread(void );
#line 40
static thread_t *SystemCallP__ThreadScheduler__currentThreadInfo(void );









static error_t SystemCallP__ThreadScheduler__wakeupThread(thread_id_t id);
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static error_t SystemCallP__threadTask__postTask(void );
# 48 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/SystemCallP.nc"
enum SystemCallP____nesc_unnamed4308 {
#line 48
  SystemCallP__threadTask = 4U
};
#line 48
typedef int SystemCallP____nesc_sillytask_threadTask[SystemCallP__threadTask];
#line 46
syscall_t *SystemCallP__current_call = (void *)0;

static inline void SystemCallP__threadTask__runTask(void );
#line 63
static error_t SystemCallP__SystemCall__start(void *syscall_ptr, syscall_t *s, syscall_id_t id, void *p);
#line 82
static inline error_t SystemCallP__SystemCall__finish(syscall_t *s);
# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/BlockingStdControl.nc"
static error_t TestSineSensorC__AMControl__start(void );
# 126 "/home/ezio/tinyos-main-read-only/tos/interfaces/Packet.nc"
static 
#line 123
void * 


TestSineSensorC__Packet__getPayload(
#line 121
message_t * msg, 




uint8_t len);
# 43 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/BlockingRead.nc"
static error_t TestSineSensorC__BlockingRead__read(TestSineSensorC__BlockingRead__val_t *val);
# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/Thread.nc"
static error_t TestSineSensorC__MainThread__start(void *arg);
# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/BlockingAMSend.nc"
static error_t TestSineSensorC__BlockingAMSend__send(am_addr_t addr, message_t *msg, uint8_t len);
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/Leds.nc"
static void TestSineSensorC__Leds__led0Toggle(void );
# 56 "TestSineSensorC.nc"
static inline void TestSineSensorC__Boot__booted(void );



static inline void TestSineSensorC__MainThread__run(void *arg);
# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadFunction.nc"
static void /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__ThreadFunction__signalThreadRun(void *arg);
# 47 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadInfoP.nc"
uint8_t /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__stack[150];
thread_t /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__thread_info;

static void /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__run_thread(void *arg) __attribute((noinline)) ;



static error_t /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__init(void );
#line 67
static inline error_t /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__Init__init(void );



static inline error_t /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__ThreadInfo__reset(void );



static inline thread_t */*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__ThreadInfo__get(void );
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCall.nc"
static error_t /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__SystemCall__finish(syscall_t *s);
#line 39
static error_t /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__SystemCall__start(void *syscall_ptr, syscall_t *s, syscall_id_t id, void *params);
# 55 "/home/ezio/tinyos-main-read-only/tos/interfaces/Read.nc"
static error_t /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__Read__read(
# 42 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingReadP.nc"
uint8_t arg_0x7f3447713e80);
# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCallQueue.nc"
static void /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__SystemCallQueue__enqueue(syscall_queue_t *q, syscall_t *t);

static syscall_t */*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__SystemCallQueue__remove(syscall_queue_t *q, syscall_t *t);

static syscall_t */*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__SystemCallQueue__find(syscall_queue_t *q, syscall_id_t id);
# 53 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingReadP.nc"
#line 50
typedef struct /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__read_params {
  uint16_t *val;
  error_t error;
} /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__read_params_t;

syscall_queue_t /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__read_queue;







static inline void /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__readTask(syscall_t *s);







static inline error_t /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__BlockingRead__read(uint8_t id, uint16_t *val);
#line 89
static inline void /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__Read__readDone(uint8_t id, error_t result, uint16_t val);






static inline error_t /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__Read__default__read(uint8_t id);
# 44 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/LinkedList.nc"
static error_t SystemCallQueueP__LinkedList__addLast(linked_list_t *l, list_element_t *e);
#line 39
static void SystemCallQueueP__LinkedList__init(linked_list_t *l);
#line 53
static list_element_t *SystemCallQueueP__LinkedList__remove(linked_list_t *l, list_element_t *e);
#line 50
static list_element_t *SystemCallQueueP__LinkedList__getAfter(linked_list_t *l, list_element_t *e);
#line 48
static list_element_t *SystemCallQueueP__LinkedList__getFirst(linked_list_t *l);
# 46 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/SystemCallQueueP.nc"
static inline void SystemCallQueueP__SystemCallQueue__init(syscall_queue_t *q);


static void SystemCallQueueP__SystemCallQueue__enqueue(syscall_queue_t *q, syscall_t *s);






static inline syscall_t *SystemCallQueueP__SystemCallQueue__remove(syscall_queue_t *q, syscall_t *s);


static syscall_t *SystemCallQueueP__SystemCallQueue__find(syscall_queue_t *q, uint8_t id);
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static error_t /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__readTask__postTask(void );
# 63 "/home/ezio/tinyos-main-read-only/tos/interfaces/Read.nc"
static void /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__Read__readDone(error_t result, /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__Read__val_t val);
# 33 "/home/ezio/tinyos-main-read-only/tos/system/SineSensorC.nc"
enum /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0____nesc_unnamed4309 {
#line 33
  SineSensorC__0__readTask = 5U
};
#line 33
typedef int /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0____nesc_sillytask_readTask[/*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__readTask];
#line 26
uint32_t /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__counter;

static inline error_t /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__Init__init(void );




static inline void /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__readTask__runTask(void );







static inline error_t /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__Read__read(void );
# 75 "/home/ezio/tinyos-main-read-only/tos/interfaces/Send.nc"
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__send(
#line 67
message_t * msg, 







uint8_t len);
# 110 "/home/ezio/tinyos-main-read-only/tos/interfaces/AMSend.nc"
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(
# 47 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x7f344767ab30, 
# 103 "/home/ezio/tinyos-main-read-only/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 78 "/home/ezio/tinyos-main-read-only/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(
# 48 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x7f3447677d10, 
# 71 "/home/ezio/tinyos-main-read-only/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
#line 78
static 
#line 74
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__ReceiveDefault__receive(
# 49 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x7f3447675aa0, 
# 71 "/home/ezio/tinyos-main-read-only/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 61 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/lib/serial/SerialActiveMessageP.nc"
static inline serial_header_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(message_t * msg);







static inline error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(am_id_t id, am_addr_t dest, 
message_t *msg, 
uint8_t len);
#line 97
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(message_t *msg, error_t result);







static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(uint8_t id, message_t *msg, void *payload, uint8_t len);







static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 131
static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength(void );



static inline void */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__getPayload(message_t *msg, uint8_t len);
#line 172
static inline am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(message_t *amsg);
# 113 "/home/ezio/tinyos-main-read-only/tos/interfaces/SplitControl.nc"
static void SerialP__SplitControl__startDone(error_t error);
#line 138
static void SerialP__SplitControl__stopDone(error_t error);
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static error_t SerialP__stopDoneTask__postTask(void );
# 95 "/home/ezio/tinyos-main-read-only/tos/interfaces/StdControl.nc"
static error_t SerialP__SerialControl__start(void );









static error_t SerialP__SerialControl__stop(void );
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static error_t SerialP__RunTx__postTask(void );
# 49 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialFlush.nc"
static void SerialP__SerialFlush__flush(void );
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static error_t SerialP__startDoneTask__postTask(void );
# 56 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialFrameComm.nc"
static error_t SerialP__SerialFrameComm__putDelimiter(void );
#line 79
static void SerialP__SerialFrameComm__resetReceive(void );
#line 65
static error_t SerialP__SerialFrameComm__putData(uint8_t data);
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static error_t SerialP__defaultSerialFlushTask__postTask(void );
# 81 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SendBytePacket.nc"
static uint8_t SerialP__SendBytePacket__nextByte(void );









static void SerialP__SendBytePacket__sendCompleted(error_t error);
# 62 "/home/ezio/tinyos-main-read-only/tos/lib/serial/ReceiveBytePacket.nc"
static error_t SerialP__ReceiveBytePacket__startPacket(void );






static void SerialP__ReceiveBytePacket__byteReceived(uint8_t data);










static void SerialP__ReceiveBytePacket__endPacket(error_t result);
# 191 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialP.nc"
enum SerialP____nesc_unnamed4310 {
#line 191
  SerialP__RunTx = 6U
};
#line 191
typedef int SerialP____nesc_sillytask_RunTx[SerialP__RunTx];
#line 322
enum SerialP____nesc_unnamed4311 {
#line 322
  SerialP__startDoneTask = 7U
};
#line 322
typedef int SerialP____nesc_sillytask_startDoneTask[SerialP__startDoneTask];









enum SerialP____nesc_unnamed4312 {
#line 332
  SerialP__stopDoneTask = 8U
};
#line 332
typedef int SerialP____nesc_sillytask_stopDoneTask[SerialP__stopDoneTask];








enum SerialP____nesc_unnamed4313 {
#line 341
  SerialP__defaultSerialFlushTask = 9U
};
#line 341
typedef int SerialP____nesc_sillytask_defaultSerialFlushTask[SerialP__defaultSerialFlushTask];
#line 81
enum SerialP____nesc_unnamed4314 {
  SerialP__RX_DATA_BUFFER_SIZE = 2, 
  SerialP__TX_DATA_BUFFER_SIZE = 4, 
  SerialP__SERIAL_MTU = 255, 
  SerialP__SERIAL_VERSION = 1, 
  SerialP__ACK_QUEUE_SIZE = 5
};

enum SerialP____nesc_unnamed4315 {
  SerialP__RXSTATE_NOSYNC, 
  SerialP__RXSTATE_PROTO, 
  SerialP__RXSTATE_TOKEN, 
  SerialP__RXSTATE_INFO, 
  SerialP__RXSTATE_INACTIVE
};

enum SerialP____nesc_unnamed4316 {
  SerialP__TXSTATE_IDLE, 
  SerialP__TXSTATE_PROTO, 
  SerialP__TXSTATE_SEQNO, 
  SerialP__TXSTATE_INFO, 
  SerialP__TXSTATE_FCS1, 
  SerialP__TXSTATE_FCS2, 
  SerialP__TXSTATE_ENDFLAG, 
  SerialP__TXSTATE_ENDWAIT, 
  SerialP__TXSTATE_FINISH, 
  SerialP__TXSTATE_ERROR, 
  SerialP__TXSTATE_INACTIVE
};





#line 111
typedef enum SerialP____nesc_unnamed4317 {
  SerialP__BUFFER_AVAILABLE, 
  SerialP__BUFFER_FILLING, 
  SerialP__BUFFER_COMPLETE
} SerialP__tx_data_buffer_states_t;

enum SerialP____nesc_unnamed4318 {
  SerialP__TX_ACK_INDEX = 0, 
  SerialP__TX_DATA_INDEX = 1, 
  SerialP__TX_BUFFER_COUNT = 2
};






#line 124
typedef struct SerialP____nesc_unnamed4319 {
  uint8_t writePtr;
  uint8_t readPtr;
  uint8_t buf[SerialP__RX_DATA_BUFFER_SIZE + 1];
} SerialP__rx_buf_t;




#line 130
typedef struct SerialP____nesc_unnamed4320 {
  uint8_t state;
  uint8_t buf;
} SerialP__tx_buf_t;





#line 135
typedef struct SerialP____nesc_unnamed4321 {
  uint8_t writePtr;
  uint8_t readPtr;
  uint8_t buf[SerialP__ACK_QUEUE_SIZE + 1];
} SerialP__ack_queue_t;



SerialP__rx_buf_t SerialP__rxBuf;
SerialP__tx_buf_t SerialP__txBuf[SerialP__TX_BUFFER_COUNT];



uint8_t SerialP__rxState;
uint8_t SerialP__rxByteCnt;
uint8_t SerialP__rxProto;
uint8_t SerialP__rxSeqno;
uint16_t SerialP__rxCRC;



uint8_t SerialP__txState;
uint8_t SerialP__txByteCnt;
uint8_t SerialP__txProto;
uint8_t SerialP__txSeqno;
uint16_t SerialP__txCRC;
uint8_t SerialP__txPending;
uint8_t SerialP__txIndex;


SerialP__ack_queue_t SerialP__ackQ;

bool SerialP__offPending = FALSE;



static __inline void SerialP__txInit(void );
static __inline void SerialP__rxInit(void );
static __inline void SerialP__ackInit(void );

static __inline bool SerialP__ack_queue_is_full(void );
static __inline bool SerialP__ack_queue_is_empty(void );
static __inline void SerialP__ack_queue_push(uint8_t token);
static __inline uint8_t SerialP__ack_queue_top(void );
static inline uint8_t SerialP__ack_queue_pop(void );




static __inline void SerialP__rx_buffer_push(uint8_t data);
static __inline uint8_t SerialP__rx_buffer_top(void );
static __inline uint8_t SerialP__rx_buffer_pop(void );
static __inline uint16_t SerialP__rx_current_crc(void );

static void SerialP__rx_state_machine(bool isDelimeter, uint8_t data);
static void SerialP__MaybeScheduleTx(void );




static __inline void SerialP__txInit(void );
#line 207
static __inline void SerialP__rxInit(void );








static __inline void SerialP__ackInit(void );



static inline error_t SerialP__Init__init(void );
#line 234
static __inline bool SerialP__ack_queue_is_full(void );









static __inline bool SerialP__ack_queue_is_empty(void );





static __inline void SerialP__ack_queue_push(uint8_t token);









static __inline uint8_t SerialP__ack_queue_top(void );









static inline uint8_t SerialP__ack_queue_pop(void );
#line 297
static __inline void SerialP__rx_buffer_push(uint8_t data);



static __inline uint8_t SerialP__rx_buffer_top(void );



static __inline uint8_t SerialP__rx_buffer_pop(void );





static __inline uint16_t SerialP__rx_current_crc(void );










static inline void SerialP__startDoneTask__runTask(void );









static inline void SerialP__stopDoneTask__runTask(void );



static inline void SerialP__SerialFlush__flushDone(void );




static inline void SerialP__defaultSerialFlushTask__runTask(void );


static inline void SerialP__SerialFlush__default__flush(void );



static inline error_t SerialP__SplitControl__start(void );








static void SerialP__testOff(void );
#line 394
static inline void SerialP__SerialFrameComm__delimiterReceived(void );


static inline void SerialP__SerialFrameComm__dataReceived(uint8_t data);



static inline bool SerialP__valid_rx_proto(uint8_t proto);










static void SerialP__rx_state_machine(bool isDelimeter, uint8_t data);
#line 518
static void SerialP__MaybeScheduleTx(void );










static inline error_t SerialP__SendBytePacket__completeSend(void );








static inline error_t SerialP__SendBytePacket__startSend(uint8_t b);
#line 559
static inline void SerialP__RunTx__runTask(void );
#line 668
static inline void SerialP__SerialFrameComm__putDone(void );
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__postTask(void );
# 100 "/home/ezio/tinyos-main-read-only/tos/interfaces/Send.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(
# 51 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f344752e110, 
# 96 "/home/ezio/tinyos-main-read-only/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__postTask(void );
# 78 "/home/ezio/tinyos-main-read-only/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(
# 50 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f344752f570, 
# 71 "/home/ezio/tinyos-main-read-only/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 31 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(
# 54 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f344752c4d0, 
# 31 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(
# 54 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f344752c4d0);
# 23 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(
# 54 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x7f344752c4d0, 
# 23 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t upperLen);
# 71 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SendBytePacket.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__completeSend(void );
#line 62
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__startSend(uint8_t first_byte);
# 158 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4322 {
#line 158
  SerialDispatcherP__0__signalSendDone = 10U
};
#line 158
typedef int /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_sillytask_signalSendDone[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone];
#line 275
enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4323 {
#line 275
  SerialDispatcherP__0__receiveTask = 11U
};
#line 275
typedef int /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_sillytask_receiveTask[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask];
#line 66
#line 62
typedef enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4324 {
  SerialDispatcherP__0__SEND_STATE_IDLE = 0, 
  SerialDispatcherP__0__SEND_STATE_BEGIN = 1, 
  SerialDispatcherP__0__SEND_STATE_DATA = 2
} /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__send_state_t;

enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4325 {
  SerialDispatcherP__0__RECV_STATE_IDLE = 0, 
  SerialDispatcherP__0__RECV_STATE_BEGIN = 1, 
  SerialDispatcherP__0__RECV_STATE_DATA = 2
};






#line 74
typedef struct /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4326 {
  uint8_t which : 1;
  uint8_t bufZeroLocked : 1;
  uint8_t bufOneLocked : 1;
  uint8_t state : 2;
} /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recv_state_t;



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recv_state_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState = { 0, 0, 0, /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_IDLE };
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType = TOS_SERIAL_UNKNOWN_ID;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex = 0;


message_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[2];
message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messagePtrs[2] = { &/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[0], &/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[1] };




uint8_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer = (uint8_t * )&/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[0];

uint8_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer = (void *)0;
/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__send_state_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendLen = 0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex = 0;
error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError = SUCCESS;
bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendCancelled = FALSE;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendId = 0;


uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending = FALSE;
uart_id_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskType = 0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskWhich;
message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskBuf = (void *)0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskSize = 0;

static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(uint8_t id, message_t *msg, uint8_t len);
#line 158
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask(void );
#line 178
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte(void );
#line 194
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error_t error);




static inline bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__isCurrentBufferLocked(void );



static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__lockCurrentBuffer(void );








static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(uint8_t which);








static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBufferSwap(void );




static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket(void );
#line 244
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(uint8_t b);
#line 275
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask(void );
#line 296
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(error_t result);
#line 358
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(uart_id_t id);


static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(uart_id_t id, message_t *msg, 
uint8_t upperLen);


static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(uart_id_t id, message_t *msg, 
uint8_t dataLinkLen);




static inline message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(uart_id_t idxxx, message_t *msg, 
void *payload, 
uint8_t len);


static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(uart_id_t idxxx, message_t *msg, error_t error);
# 48 "/home/ezio/tinyos-main-read-only/tos/interfaces/UartStream.nc"
static error_t HdlcTranslateC__UartStream__send(
#line 44
uint8_t * buf, 



uint16_t len);
# 94 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialFrameComm.nc"
static void HdlcTranslateC__SerialFrameComm__dataReceived(uint8_t data);





static void HdlcTranslateC__SerialFrameComm__putDone(void );
#line 85
static void HdlcTranslateC__SerialFrameComm__delimiterReceived(void );
# 59 "/home/ezio/tinyos-main-read-only/tos/lib/serial/HdlcTranslateC.nc"
#line 56
typedef struct HdlcTranslateC____nesc_unnamed4327 {
  uint8_t sendEscape : 1;
  uint8_t receiveEscape : 1;
} HdlcTranslateC__HdlcState;


HdlcTranslateC__HdlcState HdlcTranslateC__state = { 0, 0 };
uint8_t HdlcTranslateC__txTemp;
uint8_t HdlcTranslateC__m_data;


static inline void HdlcTranslateC__SerialFrameComm__resetReceive(void );





static inline void HdlcTranslateC__UartStream__receivedByte(uint8_t data);
#line 98
static error_t HdlcTranslateC__SerialFrameComm__putDelimiter(void );







static error_t HdlcTranslateC__SerialFrameComm__putData(uint8_t data);
#line 118
static void HdlcTranslateC__UartStream__sendDone(uint8_t *buf, uint16_t len, 
error_t error);
#line 132
static inline void HdlcTranslateC__UartStream__receiveDone(uint8_t *buf, uint16_t len, error_t error);
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(
# 49 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f344741e020);
# 97 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__resetUsart(bool reset);
#line 179
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableIntr(void );


static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableIntr(void );
#line 224
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(uint8_t data);
#line 128
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableUart(void );
#line 174
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__setModeUart(msp430_uart_union_config_t *config);
# 79 "/home/ezio/tinyos-main-read-only/tos/interfaces/UartStream.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receivedByte(
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f3447423840, 
# 79 "/home/ezio/tinyos-main-read-only/tos/interfaces/UartStream.nc"
uint8_t byte);
#line 99
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receiveDone(
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f3447423840, 
# 95 "/home/ezio/tinyos-main-read-only/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f3447423840, 
# 53 "/home/ezio/tinyos-main-read-only/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
# 120 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__release(
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f3447421cb0);
# 97 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__immediateRequest(
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f3447421cb0);
# 128 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f3447421cb0);
# 102 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__granted(
# 43 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x7f3447425790);
#line 59
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len;
#line 59
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_len;
uint8_t * /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf;
#line 60
uint8_t * /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf;
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos;
#line 61
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_pos;
uint8_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_byte_time;
uint8_t /*Msp430Uart1P.UartP*/Msp430UartP__0__current_owner;

static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(uint8_t id);
#line 77
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__release(uint8_t id);







static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(uint8_t id);






static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(uint8_t id);








static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(uint8_t id);
#line 134
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(uint8_t id, uint8_t data);
#line 147
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__send(uint8_t id, uint8_t *buf, uint16_t len);
#line 162
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(uint8_t id);
#line 208
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow(void );

static inline bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(uint8_t id);

static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(uint8_t id);
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__release(uint8_t id);
static inline msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(uint8_t id);



static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(uint8_t id);

static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error);
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(uint8_t id, uint8_t byte);
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error);
# 99 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart1P__UCLK__selectIOFunc(void );
# 54 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void HplMsp430Usart1P__Interrupts__rxDone(uint8_t data);
#line 49
static void HplMsp430Usart1P__Interrupts__txDone(void );
# 99 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart1P__URXD__selectIOFunc(void );
#line 92
static void HplMsp430Usart1P__URXD__selectModuleFunc(void );






static void HplMsp430Usart1P__UTXD__selectIOFunc(void );
#line 92
static void HplMsp430Usart1P__UTXD__selectModuleFunc(void );
# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/PlatformInterrupt.nc"
static void HplMsp430Usart1P__PlatformInterrupt__postAmble(void );
# 99 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart1P__SOMI__selectIOFunc(void );
#line 99
static void HplMsp430Usart1P__SIMO__selectIOFunc(void );
# 88 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/chips/msp430/HplMsp430Usart1P.nc"
static volatile uint8_t HplMsp430Usart1P__IE2 __asm ("0x0001");
static volatile uint8_t HplMsp430Usart1P__ME2 __asm ("0x0005");
static volatile uint8_t HplMsp430Usart1P__IFG2 __asm ("0x0003");
static volatile uint8_t HplMsp430Usart1P__U1TCTL __asm ("0x0079");
static volatile uint8_t HplMsp430Usart1P__U1RCTL __asm ("0x007A");
static volatile uint8_t HplMsp430Usart1P__U1TXBUF __asm ("0x007F");



void sig_UART1RX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0006)))  ;





void sig_UART1TX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0004)))  ;




static inline error_t HplMsp430Usart1P__AsyncStdControl__start(void );



static inline error_t HplMsp430Usart1P__AsyncStdControl__stop(void );
#line 143
static inline void HplMsp430Usart1P__Usart__setUbr(uint16_t control);










static inline void HplMsp430Usart1P__Usart__setUmctl(uint8_t control);







static inline void HplMsp430Usart1P__Usart__resetUsart(bool reset);
#line 206
static inline void HplMsp430Usart1P__Usart__enableUart(void );







static void HplMsp430Usart1P__Usart__disableUart(void );








static inline void HplMsp430Usart1P__Usart__enableUartTx(void );




static inline void HplMsp430Usart1P__Usart__disableUartTx(void );





static inline void HplMsp430Usart1P__Usart__enableUartRx(void );




static inline void HplMsp430Usart1P__Usart__disableUartRx(void );
#line 254
static void HplMsp430Usart1P__Usart__disableSpi(void );
#line 286
static inline void HplMsp430Usart1P__configUart(msp430_uart_union_config_t *config);









static inline void HplMsp430Usart1P__Usart__setModeUart(msp430_uart_union_config_t *config);
#line 350
static inline void HplMsp430Usart1P__Usart__clrIntr(void );
#line 362
static inline void HplMsp430Usart1P__Usart__disableIntr(void );
#line 380
static inline void HplMsp430Usart1P__Usart__enableIntr(void );






static inline void HplMsp430Usart1P__Usart__tx(uint8_t data);
# 90 "/home/ezio/tinyos-main-read-only/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void );
# 54 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x7f34472d5060, 
# 54 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(
# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x7f34472d5060);









static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );




static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);









static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data);
# 49 "/home/ezio/tinyos-main-read-only/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0____nesc_unnamed4328 {
#line 49
  FcfsResourceQueueC__0__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[1U];
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

static inline error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void );




static inline bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );







static inline resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 61 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(
# 55 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
uint8_t arg_0x7f3447293840);
# 65 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(
# 60 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
uint8_t arg_0x7f3447290c40);
# 59 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(
# 60 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
uint8_t arg_0x7f3447290c40);
# 53 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceQueue.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void );
#line 70
static resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void );
# 46 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void );
# 102 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(
# 54 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
uint8_t arg_0x7f34472944b0);
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void );
# 75 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4329 {
#line 75
  ArbiterP__0__grantedTask = 12U
};
#line 75
typedef int /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_sillytask_grantedTask[/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask];
#line 67
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4330 {
#line 67
  ArbiterP__0__RES_CONTROLLED, ArbiterP__0__RES_GRANTING, ArbiterP__0__RES_IMM_GRANTING, ArbiterP__0__RES_BUSY
};
#line 68
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4331 {
#line 68
  ArbiterP__0__default_owner_id = 1U
};
#line 69
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4332 {
#line 69
  ArbiterP__0__NO_RES = 0xFF
};
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
#line 93
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id);
#line 111
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id);
#line 133
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 153
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );
#line 166
static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );










static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id);
#line 190
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
#line 202
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id);



static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id);









static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id);
# 62 "/home/ezio/tinyos-main-read-only/tos/lib/power/PowerDownCleanup.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__cleanup(void );
# 56 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release(void );
# 95 "/home/ezio/tinyos-main-read-only/tos/interfaces/AsyncStdControl.nc"
static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start(void );









static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__stop(void );
# 74 "/home/ezio/tinyos-main-read-only/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void );




static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted(void );




static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup(void );
# 120 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
static error_t TelosSerialP__Resource__release(void );
#line 97
static error_t TelosSerialP__Resource__immediateRequest(void );
# 8 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/platforms/telosa/TelosSerialP.nc"
msp430_uart_union_config_t TelosSerialP__msp430_uart_telos_config = { { .ubr = UBR_1MHZ_57600, .umctl = UMCTL_1MHZ_57600, .ssel = 0x02, .pena = 0, .pev = 0, .spb = 0, .clen = 1, .listen = 0, .mm = 0, .ckpl = 0, .urxse = 0, .urxeie = 1, .urxwie = 0, .utxe = 1, .urxe = 1 } };

static inline error_t TelosSerialP__StdControl__start(void );


static inline error_t TelosSerialP__StdControl__stop(void );



static inline void TelosSerialP__Resource__granted(void );

static inline msp430_uart_union_config_t *TelosSerialP__Msp430UartConfigure__getConfig(void );
# 51 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__offset(void );


static inline uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen);


static inline uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen);
# 104 "/home/ezio/tinyos-main-read-only/tos/interfaces/SplitControl.nc"
static error_t BlockingStdControlImplP__SplitControl__start(
# 42 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingStdControlImplP.nc"
uint8_t arg_0x7f34471dba90);
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCall.nc"
static error_t BlockingStdControlImplP__SystemCall__finish(syscall_t *s);
#line 39
static error_t BlockingStdControlImplP__SystemCall__start(void *syscall_ptr, syscall_t *s, syscall_id_t id, void *params);
# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCallQueue.nc"
static void BlockingStdControlImplP__SystemCallQueue__enqueue(syscall_queue_t *q, syscall_t *t);
#line 40
static void BlockingStdControlImplP__SystemCallQueue__init(syscall_queue_t *q);


static syscall_t *BlockingStdControlImplP__SystemCallQueue__remove(syscall_queue_t *q, syscall_t *t);

static syscall_t *BlockingStdControlImplP__SystemCallQueue__find(syscall_queue_t *q, syscall_id_t id);
# 53 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingStdControlImplP.nc"
#line 51
typedef struct BlockingStdControlImplP__params {
  error_t error;
} BlockingStdControlImplP__params_t;

syscall_queue_t BlockingStdControlImplP__std_cntrl_queue;

static inline error_t BlockingStdControlImplP__Init__init(void );





static inline void BlockingStdControlImplP__startTask(syscall_t *s);






static inline error_t BlockingStdControlImplP__BlockingStdControl__start(uint8_t id);
#line 87
static inline void BlockingStdControlImplP__SplitControl__startDone(uint8_t id, error_t error);
#line 119
static inline void BlockingStdControlImplP__SplitControl__stopDone(uint8_t id, error_t error);





static inline error_t BlockingStdControlImplP__SplitControl__default__start(uint8_t id);
# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
static thread_t */*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__ThreadScheduler__threadInfo(thread_id_t id);
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCall.nc"
static error_t /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__SystemCall__finish(syscall_t *s);
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCallQueue.nc"
static void /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__SystemCallQueue__init(syscall_queue_t *q);




static syscall_t */*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__SystemCallQueue__find(syscall_queue_t *q, syscall_id_t id);
# 78 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static void /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__Timer__stop(
# 44 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingAMReceiverImplP.nc"
uint8_t arg_0x7f34471a47a0);
#line 58
#line 54
typedef struct /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__params {
  uint32_t *timeout;
  message_t *msg;
  error_t error;
} /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__params_t;


syscall_queue_t /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__am_queue;


bool /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__blockForAny = FALSE;






static inline error_t /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__Init__init(void );
#line 133
static inline message_t */*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__Receive__receive(uint8_t am_id, message_t *m, void *payload, uint8_t len);
#line 153
static inline void /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__Timer__fired(uint8_t id);
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCall.nc"
static error_t /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__SystemCall__finish(syscall_t *s);
#line 39
static error_t /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__SystemCall__start(void *syscall_ptr, syscall_t *s, syscall_id_t id, void *params);
# 80 "/home/ezio/tinyos-main-read-only/tos/interfaces/AMSend.nc"
static error_t /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__AMSend__send(
# 45 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingAMSenderImplP.nc"
am_id_t arg_0x7f3447145ce0, 
# 80 "/home/ezio/tinyos-main-read-only/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/Mutex.nc"
static error_t /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__Mutex__unlock(mutex_t *m);
#line 40
static error_t /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__Mutex__lock(mutex_t *m);
#line 39
static void /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__Mutex__init(mutex_t *m);
# 57 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingAMSenderImplP.nc"
#line 52
typedef struct /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__params {
  am_addr_t addr;
  message_t *msg;
  uint8_t len;
  error_t error;
} /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__params_t;

syscall_t */*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__send_call = (void *)0;
mutex_t /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__my_mutex;

static inline void /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__sendTask(syscall_t *s);






static inline error_t /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__Init__init(void );




static inline error_t /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__BlockingAMSend__send(am_id_t am_id, am_addr_t addr, message_t *msg, uint8_t len);
#line 104
static inline void /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__AMSend__sendDone(am_id_t am_id, message_t *m, error_t error);
# 47 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
static error_t MutexP__ThreadScheduler__suspendCurrentThread(void );
#line 40
static thread_t *MutexP__ThreadScheduler__currentThreadInfo(void );









static error_t MutexP__ThreadScheduler__wakeupThread(thread_id_t id);
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadQueue.nc"
static void MutexP__ThreadQueue__enqueue(thread_queue_t *q, thread_t *t);
#line 39
static void MutexP__ThreadQueue__init(thread_queue_t *q);

static thread_t *MutexP__ThreadQueue__dequeue(thread_queue_t *q);
# 47 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/MutexP.nc"
static inline void MutexP__Mutex__init(mutex_t *m);




static inline error_t MutexP__Mutex__lock(mutex_t *m);
#line 67
static inline error_t MutexP__Mutex__unlock(mutex_t *m);
# 196 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void )
{
}

# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void ){
#line 48
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow();
#line 48
}
#line 48
# 137 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n)
{
}

# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0x7f3447f5e110){
#line 39
  switch (arg_0x7f3447f5e110) {
#line 39
    case 0:
#line 39
      /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired();
#line 39
      break;
#line 39
    case 1:
#line 39
      /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired();
#line 39
      break;
#line 39
    case 2:
#line 39
      /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired();
#line 39
      break;
#line 39
    case 5:
#line 39
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired();
#line 39
      break;
#line 39
    default:
#line 39
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0x7f3447f5e110);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 126 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(0);
}

# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA0__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired();
#line 39
}
#line 39
# 58 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4333 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(* (volatile uint16_t * )354U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void )
{
}

# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired();
#line 45
}
#line 45
# 58 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4334 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(* (volatile uint16_t * )356U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void )
{
}

# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired();
#line 45
}
#line 45
# 58 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4335 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(* (volatile uint16_t * )358U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void )
{
}

# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired();
#line 45
}
#line 45
# 98 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP__TaskScheduler__hasTasks(void )
#line 98
{
  /* atomic removed: atomic calls only */
#line 99
  {
    unsigned char __nesc_temp = 
#line 99
    SchedulerBasicP__m_head != SchedulerBasicP__NO_TASK;

#line 99
    return __nesc_temp;
  }
}

# 73 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/TaskScheduler.nc"
inline static bool TOSThreadsInterruptP__TaskScheduler__hasTasks(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = SchedulerBasicP__TaskScheduler__hasTasks();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 52 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TOSThreadsInterruptP.nc"
static __inline void TOSThreadsInterruptP__PlatformInterrupt__postAmble(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 53
  {
    if (TOSThreadsInterruptP__TaskScheduler__hasTasks() == TRUE) {
      TOSThreadsInterruptP__interruptThread();
      }
  }
}

# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/PlatformInterrupt.nc"
inline static void Msp430TimerCommonP__PlatformInterrupt__postAmble(void ){
#line 37
  TOSThreadsInterruptP__PlatformInterrupt__postAmble();
#line 37
}
#line 37
# 50 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static error_t TOSThreadsInterruptP__ThreadScheduler__wakeupThread(thread_id_t id){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = TinyThreadSchedulerP__ThreadScheduler__wakeupThread(id);
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 307 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static inline uint8_t TinyThreadSchedulerP__ThreadScheduler__currentThreadId(void )
#line 307
{
  /* atomic removed: atomic calls only */
#line 308
  {
    unsigned char __nesc_temp = 
#line 308
    TinyThreadSchedulerP__current_thread->id;

#line 308
    return __nesc_temp;
  }
}

# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static uint8_t TOSThreadsInterruptP__ThreadScheduler__currentThreadId(void ){
#line 39
  unsigned char __nesc_result;
#line 39

#line 39
  __nesc_result = TinyThreadSchedulerP__ThreadScheduler__currentThreadId();
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39









inline static error_t TOSThreadsInterruptP__ThreadScheduler__interruptCurrentThread(void ){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = TinyThreadSchedulerP__ThreadScheduler__interruptCurrentThread();
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 131 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/LinkedListC.nc"
static inline list_element_t *LinkedListC__LinkedList__removeLast(linked_list_t *l)
#line 131
{
  return LinkedListC__LinkedList__removeAt(l, l->size - 1);
}

# 56 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/LinkedList.nc"
inline static list_element_t *ThreadQueueP__LinkedList__removeLast(linked_list_t *l){
#line 56
  struct list_element *__nesc_result;
#line 56

#line 56
  __nesc_result = LinkedListC__LinkedList__removeLast(l);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 51 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadQueueP.nc"
static inline thread_t *ThreadQueueP__ThreadQueue__dequeue(thread_queue_t *q)
#line 51
{
  return (thread_t *)ThreadQueueP__LinkedList__removeLast(& q->l);
}

# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadQueue.nc"
inline static thread_t *TinyThreadSchedulerP__ThreadQueue__dequeue(thread_queue_t *q){
#line 41
  struct thread *__nesc_result;
#line 41

#line 41
  __nesc_result = ThreadQueueP__ThreadQueue__dequeue(q);
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 121 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/LinkedListC.nc"
static inline list_element_t *LinkedListC__LinkedList__removeFirst(linked_list_t *l)
#line 121
{
  if (l->head == (void *)0) {
#line 122
    return (void *)0;
    }
  else {
#line 123
    return LinkedListC__remove_element(l, & l->head);
    }
}

# 131 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

#line 134
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(n >> 1);
}

# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA1__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired();
#line 39
}
#line 39
# 126 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(0);
}

# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB0__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired();
#line 39
}
#line 39
# 196 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void )
{
}

# 114 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void )
{
}

# 58 "/home/ezio/tinyos-main-read-only/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void )
{
}

# 177 "/home/ezio/tinyos-main-read-only/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void )
{
}

# 82 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void ){
#line 82
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow();
#line 82
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow();
#line 82
}
#line 82
# 133 "/home/ezio/tinyos-main-read-only/tos/lib/timer/TransformCounterC.nc"
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC__0__m_upper & /*CounterMilli32C.Transform*/TransformCounterC__0__OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow();
      }
  }
}

# 208 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow(void )
#line 208
{
}

# 82 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void ){
#line 82
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow();
#line 82
  /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow();
#line 82
}
#line 82
# 64 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow();
}

# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void ){
#line 48
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow();
#line 48
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow();
#line 48
}
#line 48
# 137 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow();
}

# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 81 "/home/ezio/tinyos-main-read-only/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void )
{
#line 82
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask();
}

# 78 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired();
#line 78
}
#line 78
# 162 "/home/ezio/tinyos-main-read-only/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
      }
  }
}

# 78 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void ){
#line 78
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired();
#line 78
}
#line 78
# 135 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

# 58 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents();
#line 58
}
#line 58
# 70 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired();
}

# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void ){
#line 45
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired();
#line 45
}
#line 45
# 150 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4336 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(* (volatile uint16_t * )386U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired();
    }
}

# 94 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP__isWaiting(uint8_t id)
#line 94
{
  return SchedulerBasicP__m_next[id] != SchedulerBasicP__NO_TASK || SchedulerBasicP__m_tail == id;
}





static inline bool SchedulerBasicP__pushTask(uint8_t id)
#line 102
{
  if (!SchedulerBasicP__isWaiting(id)) {
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) {
          SchedulerBasicP__m_head = id;
          SchedulerBasicP__m_tail = id;
        }
      else {
          SchedulerBasicP__m_next[SchedulerBasicP__m_tail] = id;
          SchedulerBasicP__m_tail = id;
        }
      return TRUE;
    }
  else {
      return FALSE;
    }
}

# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 49 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get();
}

# 64 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Counter.nc"
inline static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 81 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void )
{
  return * (volatile uint16_t * )384U & 1U;
}

# 46 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void ){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending();
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 54 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending();
}

# 71 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Counter.nc"
inline static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 130 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void )
{
  * (volatile uint16_t * )386U |= 0x0010;
}

# 57 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents();
#line 57
}
#line 57
# 95 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )386U &= ~0x0001;
}

# 44 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 155 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )402U = x;
}

# 41 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time){
#line 41
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(time);
#line 41
}
#line 41
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 165 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get() + x;
}

# 43 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta){
#line 43
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(delta);
#line 43
}
#line 43
# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 81 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 87
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 94
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 97
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 99
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents();
  }
}

# 103 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt){
#line 103
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 192 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void )
{
}

# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void )
{
  return * (volatile uint16_t * )404U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4337 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(* (volatile uint16_t * )388U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void )
{
}

# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void )
{
  return * (volatile uint16_t * )406U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4338 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(* (volatile uint16_t * )390U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void )
{
}

# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void )
{
  return * (volatile uint16_t * )408U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4339 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(* (volatile uint16_t * )392U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void )
{
}

# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void )
{
  return * (volatile uint16_t * )410U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7____nesc_unnamed4340 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(* (volatile uint16_t * )394U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void )
{
}

# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void )
{
  return * (volatile uint16_t * )412U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8____nesc_unnamed4341 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(* (volatile uint16_t * )396U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void )
{
}

# 45 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void )
{
  return * (volatile uint16_t * )414U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9____nesc_unnamed4342 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(* (volatile uint16_t * )398U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired();
    }
}

# 131 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

#line 134
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(n >> 1);
}

# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB1__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired();
#line 39
}
#line 39
# 47 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static error_t SchedulerBasicP__ThreadScheduler__suspendCurrentThread(void ){
#line 47
  unsigned char __nesc_result;
#line 47

#line 47
  __nesc_result = TinyThreadSchedulerP__ThreadScheduler__suspendCurrentThread();
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
# 79 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP__popTask(void )
#line 79
{
  if (SchedulerBasicP__m_head != SchedulerBasicP__NO_TASK) {
      uint8_t id = SchedulerBasicP__m_head;

#line 82
      SchedulerBasicP__m_head = SchedulerBasicP__m_next[SchedulerBasicP__m_head];
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) {
          SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
        }
      SchedulerBasicP__m_next[id] = SchedulerBasicP__NO_TASK;
      return id;
    }
  else {
      return SchedulerBasicP__NO_TASK;
    }
}

#line 139
static inline void SchedulerBasicP__TaskScheduler__taskLoop(void )
#line 139
{
  for (; ; ) {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 143
        {
          while ((nextTask = SchedulerBasicP__popTask()) == SchedulerBasicP__NO_TASK) {
              SchedulerBasicP__ThreadScheduler__suspendCurrentThread();
            }
        }
#line 147
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP__TaskBasic__runTask(nextTask);
    }
}

# 80 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/TaskScheduler.nc"
inline static void TinyOSMainP__TaskScheduler__taskLoop(void ){
#line 80
  SchedulerBasicP__TaskScheduler__taskLoop();
#line 80
}
#line 80
# 108 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
static inline void StaticThreadP__ThreadNotification__default__justCreated(uint8_t id)
#line 108
{
}

# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadNotification.nc"
inline static void StaticThreadP__ThreadNotification__justCreated(uint8_t arg_0x7f3447868240){
#line 37
    StaticThreadP__ThreadNotification__default__justCreated(arg_0x7f3447868240);
#line 37
}
#line 37
# 114 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/LinkedListC.nc"
static inline error_t LinkedListC__LinkedList__addFirst(linked_list_t *l, list_element_t *e)
#line 114
{
  return LinkedListC__insert_element(l, & l->head, e);
}

# 43 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/LinkedList.nc"
inline static error_t ThreadQueueP__LinkedList__addFirst(linked_list_t *l, list_element_t *e){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = LinkedListC__LinkedList__addFirst(l, e);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 48 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadQueueP.nc"
static inline void ThreadQueueP__ThreadQueue__enqueue(thread_queue_t *q, thread_t *t)
#line 48
{
  ThreadQueueP__LinkedList__addFirst(& q->l, (list_element_t *)t);
}

# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadQueue.nc"
inline static void TinyThreadSchedulerP__ThreadQueue__enqueue(thread_queue_t *q, thread_t *t){
#line 40
  ThreadQueueP__ThreadQueue__enqueue(q, t);
#line 40
}
#line 40
# 73 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static void TinyThreadSchedulerP__PreemptionAlarm__startOneShot(uint32_t dt){
#line 73
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(0U, dt);
#line 73
}
#line 73
# 118 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyOSMainP.nc"
static inline thread_t *TinyOSMainP__ThreadInfo__get(void )
#line 118
{
  return &TinyOSMainP__thread_info;
}

# 75 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadInfoP.nc"
static inline thread_t */*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__ThreadInfo__get(void )
#line 75
{
  return &/*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__thread_info;
}

# 56 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadMapP.nc"
static inline thread_t *ThreadMapP__StaticThreadInfo__default__get(uint8_t id)
#line 56
{
  return ThreadMapP__DynamicThreadInfo__get(id);
}

# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
inline static thread_t *ThreadMapP__StaticThreadInfo__get(uint8_t arg_0x7f3447831b60){
#line 40
  struct thread *__nesc_result;
#line 40

#line 40
  switch (arg_0x7f3447831b60) {
#line 40
    case /*TestSineSensorAppC.MainThread*/ThreadC__0__THREAD_ID:
#line 40
      __nesc_result = /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__ThreadInfo__get();
#line 40
      break;
#line 40
    case TOSTHREAD_TOS_THREAD_ID:
#line 40
      __nesc_result = TinyOSMainP__ThreadInfo__get();
#line 40
      break;
#line 40
    default:
#line 40
      __nesc_result = ThreadMapP__StaticThreadInfo__default__get(arg_0x7f3447831b60);
#line 40
      break;
#line 40
    }
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 50 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadMapP.nc"
static inline thread_t *ThreadMapP__ThreadInfo__get(uint8_t id)
#line 50
{
  return ThreadMapP__StaticThreadInfo__get(id);
}

# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
inline static thread_t *TinyThreadSchedulerP__ThreadInfo__get(uint8_t arg_0x7f3447e62af0){
#line 40
  struct thread *__nesc_result;
#line 40

#line 40
  __nesc_result = ThreadMapP__ThreadInfo__get(arg_0x7f3447e62af0);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 222 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static inline error_t TinyThreadSchedulerP__ThreadScheduler__startThread(uint8_t id)
#line 222
{
  /* atomic removed: atomic calls only */
#line 223
  {
    thread_t *t = TinyThreadSchedulerP__ThreadInfo__get(id);

#line 225
    if (t->state == TOSTHREAD_STATE_INACTIVE) {
        TinyThreadSchedulerP__num_runnable_threads++;



        if (TinyThreadSchedulerP__num_runnable_threads == 2) {
          TinyThreadSchedulerP__PreemptionAlarm__startOneShot(TOSTHREAD_PREEMPTION_PERIOD);
          }
        t->state = TOSTHREAD_STATE_READY;
        TinyThreadSchedulerP__ThreadQueue__enqueue(&TinyThreadSchedulerP__ready_queue, t);
        {
          unsigned char __nesc_temp = 
#line 235
          SUCCESS;

#line 235
          return __nesc_temp;
        }
      }
  }
#line 238
  return FAIL;
}

# 44 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static error_t StaticThreadP__ThreadScheduler__startThread(thread_id_t id){
#line 44
  unsigned char __nesc_result;
#line 44

#line 44
  __nesc_result = TinyThreadSchedulerP__ThreadScheduler__startThread(id);
#line 44

#line 44
  return __nesc_result;
#line 44
}
#line 44
# 69 "/home/ezio/tinyos-main-read-only/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 49 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BitArrayUtilsC.nc"
static inline void BitArrayUtilsC__BitArrayUtils__clrArray(uint8_t *array, uint8_t size)
#line 49
{
  memset(array, 0, size);
}

# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/BitArrayUtils.nc"
inline static void TinyThreadSchedulerP__BitArrayUtils__clrArray(uint8_t *array, uint8_t numBytes){
#line 39
  BitArrayUtilsC__BitArrayUtils__clrArray(array, numBytes);
#line 39
}
#line 39
# 213 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static inline error_t TinyThreadSchedulerP__ThreadScheduler__initThread(uint8_t id)
#line 213
{
  thread_t *t = TinyThreadSchedulerP__ThreadInfo__get(id);

#line 215
  t->state = TOSTHREAD_STATE_INACTIVE;
  t->init_block = TinyThreadSchedulerP__current_thread->init_block;
  TinyThreadSchedulerP__BitArrayUtils__clrArray(t->joinedOnMe, sizeof  t->joinedOnMe);
  * t->stack_ptr = (uint16_t )&TinyThreadSchedulerP__threadWrapper;
#line 218
   __asm ("mov.w r2,%0" : "=r"(t->regs.status));
  return SUCCESS;
}

# 43 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static error_t StaticThreadP__ThreadScheduler__initThread(thread_id_t id){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = TinyThreadSchedulerP__ThreadScheduler__initThread(id);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 114 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyOSMainP.nc"
static inline error_t TinyOSMainP__ThreadInfo__reset(void )
#line 114
{
  return FAIL;
}

# 71 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadInfoP.nc"
static inline error_t /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__ThreadInfo__reset(void )
#line 71
{
  return /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__init();
}

# 107 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
static inline error_t StaticThreadP__ThreadInfo__default__reset(uint8_t id)
#line 107
{
#line 107
  return FAIL;
}

# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
inline static error_t StaticThreadP__ThreadInfo__reset(uint8_t arg_0x7f34478659f0){
#line 39
  unsigned char __nesc_result;
#line 39

#line 39
  switch (arg_0x7f34478659f0) {
#line 39
    case /*TestSineSensorAppC.MainThread*/ThreadC__0__THREAD_ID:
#line 39
      __nesc_result = /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__ThreadInfo__reset();
#line 39
      break;
#line 39
    case TOSTHREAD_TOS_THREAD_ID:
#line 39
      __nesc_result = TinyOSMainP__ThreadInfo__reset();
#line 39
      break;
#line 39
    default:
#line 39
      __nesc_result = StaticThreadP__ThreadInfo__default__reset(arg_0x7f34478659f0);
#line 39
      break;
#line 39
    }
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 106 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
static inline thread_t *StaticThreadP__ThreadInfo__default__get(uint8_t id)
#line 106
{
#line 106
  return (void *)0;
}

# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
inline static thread_t *StaticThreadP__ThreadInfo__get(uint8_t arg_0x7f34478659f0){
#line 40
  struct thread *__nesc_result;
#line 40

#line 40
  switch (arg_0x7f34478659f0) {
#line 40
    case /*TestSineSensorAppC.MainThread*/ThreadC__0__THREAD_ID:
#line 40
      __nesc_result = /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__ThreadInfo__get();
#line 40
      break;
#line 40
    case TOSTHREAD_TOS_THREAD_ID:
#line 40
      __nesc_result = TinyOSMainP__ThreadInfo__get();
#line 40
      break;
#line 40
    default:
#line 40
      __nesc_result = StaticThreadP__ThreadInfo__default__get(arg_0x7f34478659f0);
#line 40
      break;
#line 40
    }
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 52 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
static inline error_t StaticThreadP__init(uint8_t id, void *arg)
#line 52
{
  error_t r1;
#line 53
  error_t r2;
  thread_t *thread_info = StaticThreadP__ThreadInfo__get(id);

#line 55
  thread_info->start_arg_ptr = arg;
  thread_info->mutex_count = 0;
  thread_info->next_thread = (void *)0;
  r1 = StaticThreadP__ThreadInfo__reset(id);
  r2 = StaticThreadP__ThreadScheduler__initThread(id);
  return ecombine(r1, r2);
}

static inline error_t StaticThreadP__Thread__start(uint8_t id, void *arg)
#line 63
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 64
    {
      if (StaticThreadP__init(id, arg) == SUCCESS) {
          error_t e = StaticThreadP__ThreadScheduler__startThread(id);

#line 67
          if (e == SUCCESS) {
            StaticThreadP__ThreadNotification__justCreated(id);
            }
#line 69
          {
            unsigned char __nesc_temp = 
#line 69
            e;

            {
#line 69
              __nesc_atomic_end(__nesc_atomic); 
#line 69
              return __nesc_temp;
            }
          }
        }
    }
#line 73
    __nesc_atomic_end(__nesc_atomic); }
#line 72
  return FAIL;
}

# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/Thread.nc"
inline static error_t TestSineSensorC__MainThread__start(void *arg){
#line 37
  unsigned char __nesc_result;
#line 37

#line 37
  __nesc_result = StaticThreadP__Thread__start(/*TestSineSensorAppC.MainThread*/ThreadC__0__THREAD_ID, arg);
#line 37

#line 37
  return __nesc_result;
#line 37
}
#line 37
# 56 "TestSineSensorC.nc"
static inline void TestSineSensorC__Boot__booted(void )
#line 56
{
  TestSineSensorC__MainThread__start((void *)0);
}

# 60 "/home/ezio/tinyos-main-read-only/tos/interfaces/Boot.nc"
inline static void TinyOSMainP__Boot__booted(void ){
#line 60
  TestSineSensorC__Boot__booted();
#line 60
}
#line 60
# 397 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
  __eint();
}

# 66 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/TaskScheduler.nc"
inline static bool TinyOSMainP__TaskScheduler__runNextTask(void ){
#line 66
  unsigned char __nesc_result;
#line 66

#line 66
  __nesc_result = SchedulerBasicP__TaskScheduler__runNextTask();
#line 66

#line 66
  return __nesc_result;
#line 66
}
#line 66
# 28 "/home/ezio/tinyos-main-read-only/tos/system/SineSensorC.nc"
static inline error_t /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__Init__init(void )
#line 28
{
  /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__counter = TOS_NODE_ID * 40;
  return SUCCESS;
}

# 57 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4343 {
#line 57
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t f;
#line 57
    uint16_t t;
  } 
#line 57
  c = { .f = x };

#line 57
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(x);
}

#line 105
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )386U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl();
}

# 47 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare();
#line 47
}
#line 47
# 53 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 67 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadInfoP.nc"
static inline error_t /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__Init__init(void )
#line 67
{
  return /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__init();
}

# 216 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialP.nc"
static __inline void SerialP__ackInit(void )
#line 216
{
  SerialP__ackQ.writePtr = SerialP__ackQ.readPtr = 0;
}

#line 207
static __inline void SerialP__rxInit(void )
#line 207
{
  SerialP__rxBuf.writePtr = SerialP__rxBuf.readPtr = 0;
  SerialP__rxState = SerialP__RXSTATE_INACTIVE;
  SerialP__rxByteCnt = 0;
  SerialP__rxProto = 0;
  SerialP__rxSeqno = 0;
  SerialP__rxCRC = 0;
}

#line 195
static __inline void SerialP__txInit(void )
#line 195
{
  uint8_t i;

  /* atomic removed: atomic calls only */
#line 197
  for (i = 0; i < SerialP__TX_BUFFER_COUNT; i++) SerialP__txBuf[i].state = SerialP__BUFFER_AVAILABLE;
  SerialP__txState = SerialP__TXSTATE_INACTIVE;
  SerialP__txByteCnt = 0;
  SerialP__txProto = 0;
  SerialP__txSeqno = 0;
  SerialP__txCRC = 0;
  SerialP__txPending = FALSE;
  SerialP__txIndex = 0;
}

#line 220
static inline error_t SerialP__Init__init(void )
#line 220
{

  SerialP__txInit();
  SerialP__rxInit();
  SerialP__ackInit();

  return SUCCESS;
}

# 55 "/home/ezio/tinyos-main-read-only/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void )
#line 55
{
  memset(/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ, /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY, sizeof /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ);
  return SUCCESS;
}

# 100 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/LinkedListC.nc"
static inline void LinkedListC__LinkedList__init(linked_list_t *l)
#line 100
{
  l->head = (void *)0;
  l->size = 0;
}

# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/LinkedList.nc"
inline static void SystemCallQueueP__LinkedList__init(linked_list_t *l){
#line 39
  LinkedListC__LinkedList__init(l);
#line 39
}
#line 39
# 46 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/SystemCallQueueP.nc"
static inline void SystemCallQueueP__SystemCallQueue__init(syscall_queue_t *q)
#line 46
{
  SystemCallQueueP__LinkedList__init(& q->l);
}

# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCallQueue.nc"
inline static void BlockingStdControlImplP__SystemCallQueue__init(syscall_queue_t *q){
#line 40
  SystemCallQueueP__SystemCallQueue__init(q);
#line 40
}
#line 40
# 57 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingStdControlImplP.nc"
static inline error_t BlockingStdControlImplP__Init__init(void )
#line 57
{
  BlockingStdControlImplP__SystemCallQueue__init(&BlockingStdControlImplP__std_cntrl_queue);
  return SUCCESS;
}

# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCallQueue.nc"
inline static void /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__SystemCallQueue__init(syscall_queue_t *q){
#line 40
  SystemCallQueueP__SystemCallQueue__init(q);
#line 40
}
#line 40
# 71 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingAMReceiverImplP.nc"
static inline error_t /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__Init__init(void )
#line 71
{
  /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__SystemCallQueue__init(&/*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__am_queue);
  /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__blockForAny = FALSE;
  return SUCCESS;
}

# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/LinkedList.nc"
inline static void ThreadQueueP__LinkedList__init(linked_list_t *l){
#line 39
  LinkedListC__LinkedList__init(l);
#line 39
}
#line 39
# 45 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadQueueP.nc"
static inline void ThreadQueueP__ThreadQueue__init(thread_queue_t *q)
#line 45
{
  ThreadQueueP__LinkedList__init(& q->l);
}

# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadQueue.nc"
inline static void MutexP__ThreadQueue__init(thread_queue_t *q){
#line 39
  ThreadQueueP__ThreadQueue__init(q);
#line 39
}
#line 39
# 47 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/MutexP.nc"
static inline void MutexP__Mutex__init(mutex_t *m)
#line 47
{
  m->lock = FALSE;
  MutexP__ThreadQueue__init(& m->thread_queue);
}

# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/Mutex.nc"
inline static void /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__Mutex__init(mutex_t *m){
#line 39
  MutexP__Mutex__init(m);
#line 39
}
#line 39
# 69 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingAMSenderImplP.nc"
static inline error_t /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__Init__init(void )
#line 69
{
  /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__Mutex__init(&/*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__my_mutex);
  return SUCCESS;
}

# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/Init.nc"
inline static error_t TinyOSMainP__SoftwareInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__Init__init();
#line 62
  __nesc_result = ecombine(__nesc_result, /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, BlockingStdControlImplP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, SerialP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__Init__init());
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 56 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )49U |= 0x01 << 6;
}

# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set();
#line 48
}
#line 48
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set();
}

# 40 "/home/ezio/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__set(void ){
#line 40
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set();
#line 40
}
#line 40
# 56 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )49U |= 0x01 << 5;
}

# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set();
#line 48
}
#line 48
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set();
}

# 40 "/home/ezio/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__set(void ){
#line 40
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set();
#line 40
}
#line 40
# 56 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )49U |= 0x01 << 4;
}

# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set();
#line 48
}
#line 48
# 48 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set();
}

# 40 "/home/ezio/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__set(void ){
#line 40
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set();
#line 40
}
#line 40
# 63 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 6;
}

# 85 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput();
}

# 46 "/home/ezio/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 5;
}

# 85 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput();
}

# 46 "/home/ezio/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 4;
}

# 85 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput();
}

# 46 "/home/ezio/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput();
#line 46
}
#line 46
# 56 "/home/ezio/tinyos-main-read-only/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 57
  {
    ;
    LedsP__Led0__makeOutput();
    LedsP__Led1__makeOutput();
    LedsP__Led2__makeOutput();
    LedsP__Led0__set();
    LedsP__Led1__set();
    LedsP__Led2__set();
  }
  return SUCCESS;
}

# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/Init.nc"
inline static error_t PlatformP__LedsInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = LedsP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 36 "/home/ezio/tinyos-main-read-only/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r |= 1 << 1;
}

#line 37
static inline  void TOSH_SET_UCLK0_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0019");

#line 37
  r |= 1 << 3;
}

#line 88
static inline  void TOSH_SET_FLASH_CS_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001D");

#line 88
  r |= 1 << 4;
}

#line 37
static inline  void TOSH_CLR_UCLK0_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0019");

#line 37
  r &= ~(1 << 3);
}

#line 88
static inline  void TOSH_CLR_FLASH_CS_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001D");

#line 88
  r &= ~(1 << 4);
}

# 11 "/home/ezio/tinyos-main-read-only/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__TOSH_wait(void )
#line 11
{
  __nop();
#line 12
  __nop();
}

# 89 "/home/ezio/tinyos-main-read-only/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_FLASH_HOLD_PIN()
#line 89
{
#line 89
  static volatile uint8_t r __asm ("0x001D");

#line 89
  r |= 1 << 7;
}

#line 88
static inline  void TOSH_MAKE_FLASH_CS_OUTPUT()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001E");

#line 88
  r |= 1 << 4;
}

#line 89
static inline  void TOSH_MAKE_FLASH_HOLD_OUTPUT()
#line 89
{
#line 89
  static volatile uint8_t r __asm ("0x001E");

#line 89
  r |= 1 << 7;
}

#line 37
static inline  void TOSH_MAKE_UCLK0_OUTPUT()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x001A");

#line 37
  r |= 1 << 3;
}

#line 36
static inline  void TOSH_MAKE_SIMO0_OUTPUT()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x001A");

#line 36
  r |= 1 << 1;
}

# 27 "/home/ezio/tinyos-main-read-only/tos/platforms/telosb/MotePlatformC.nc"
static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void )
#line 27
{

  TOSH_MAKE_SIMO0_OUTPUT();
  TOSH_MAKE_UCLK0_OUTPUT();
  TOSH_MAKE_FLASH_HOLD_OUTPUT();
  TOSH_MAKE_FLASH_CS_OUTPUT();
  TOSH_SET_FLASH_HOLD_PIN();
  TOSH_SET_FLASH_CS_PIN();

  MotePlatformC__TOSH_wait();


  TOSH_CLR_FLASH_CS_PIN();
  TOSH_CLR_UCLK0_PIN();

  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);

  TOSH_SET_FLASH_CS_PIN();
  TOSH_SET_UCLK0_PIN();
  TOSH_SET_SIMO0_PIN();
}

#line 6
static __inline void MotePlatformC__uwait(uint16_t u)
#line 6
{
  uint16_t t0 = TAR;

#line 8
  while (TAR - t0 <= u) ;
}

#line 56
static inline error_t MotePlatformC__Init__init(void )
#line 56
{
  /* atomic removed: atomic calls only */

  {
    P1SEL = 0;
    P2SEL = 0;
    P3SEL = 0;
    P4SEL = 0;
    P5SEL = 0;
    P6SEL = 0;

    P1OUT = 0x00;
    P1DIR = 0xe0;

    P2OUT = 0x30;
    P2DIR = 0x7b;

    P3OUT = 0x00;
    P3DIR = 0xf1;

    P4OUT = 0xdd;
    P4DIR = 0xfd;

    P5OUT = 0xff;
    P5DIR = 0xff;

    P6OUT = 0x00;
    P6DIR = 0xff;

    P1IE = 0;
    P2IE = 0;






    MotePlatformC__uwait(1024 * 10);

    MotePlatformC__TOSH_FLASH_M25P_DP();
  }

  return SUCCESS;
}

# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = MotePlatformC__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 163 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__startTimerB(void )
{

  Msp430ClockP__TBCTL = 0x0020 | (Msp430ClockP__TBCTL & ~(0x0020 | 0x0010));
}

#line 151
static inline void Msp430ClockP__startTimerA(void )
{

  Msp430ClockP__TACTL = 0x0020 | (Msp430ClockP__TACTL & ~(0x0020 | 0x0010));
}

#line 115
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void )
{
  TBR = 0;









  Msp430ClockP__TBCTL = 0x0100 | 0x0002;
}

#line 145
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerB();
}

# 43 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerB(void ){
#line 43
  Msp430ClockP__Msp430ClockInit__default__initTimerB();
#line 43
}
#line 43
# 100 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void )
{
  TAR = 0;









  Msp430ClockP__TACTL = 0x0200 | 0x0002;
}

#line 140
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerA();
}

# 42 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerA(void ){
#line 42
  Msp430ClockP__Msp430ClockInit__default__initTimerA();
#line 42
}
#line 42
# 79 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void )
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));







  BCSCTL2 = 0x04;


  Msp430ClockP__IE1 &= ~0x02;
}

#line 135
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitClocks();
}

# 41 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initClocks(void ){
#line 41
  Msp430ClockP__Msp430ClockInit__default__initClocks();
#line 41
}
#line 41
# 181 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib)
{
  int8_t aclk_count = 2;
  uint16_t dco_prev = 0;
  uint16_t dco_curr = 0;

  Msp430ClockP__set_dco_calib(calib);

  while (aclk_count-- > 0) 
    {
      TBCCR0 = TBR + Msp430ClockP__ACLK_CALIB_PERIOD;
      TBCCTL0 &= ~0x0001;
      while ((TBCCTL0 & 0x0001) == 0) ;
      dco_prev = dco_curr;
      dco_curr = TAR;
    }

  return dco_curr - dco_prev;
}




static inline void Msp430ClockP__busyCalibrateDco(void )
{

  int calib;
  int step;






  for (calib = 0, step = 0x800; step != 0; step >>= 1) 
    {

      if (Msp430ClockP__test_calib_busywait_delta(calib | step) <= Msp430ClockP__TARGET_DCO_DELTA) {
        calib |= step;
        }
    }

  if ((calib & 0x0e0) == 0x0e0) {
    calib &= ~0x01f;
    }
  Msp430ClockP__set_dco_calib(calib);
}

#line 67
static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void )
{



  Msp430ClockP__TACTL = 0x0200 | 0x0020;
  Msp430ClockP__TBCTL = 0x0100 | 0x0020;
  BCSCTL1 = 0x80 | 0x04;
  BCSCTL2 = 0;
  TBCCTL0 = 0x4000;
}

#line 130
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void )
{
  Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate();
}

# 40 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void ){
#line 40
  Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate();
#line 40
}
#line 40
# 229 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline error_t Msp430ClockP__Init__init(void )
{

  Msp430ClockP__TACTL = 0x0004;
  Msp430ClockP__TAIV = 0;
  Msp430ClockP__TBCTL = 0x0004;
  Msp430ClockP__TBIV = 0;
  /* atomic removed: atomic calls only */

  {
    Msp430ClockP__Msp430ClockInit__setupDcoCalibrate();
    Msp430ClockP__busyCalibrateDco();
    Msp430ClockP__Msp430ClockInit__initClocks();
    Msp430ClockP__Msp430ClockInit__initTimerA();
    Msp430ClockP__Msp430ClockInit__initTimerB();
    Msp430ClockP__startTimerA();
    Msp430ClockP__startTimerB();
  }

  return SUCCESS;
}

# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteClockInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = Msp430ClockP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 10 "/home/ezio/tinyos-main-read-only/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void )
#line 10
{
  WDTCTL = 0x5A00 + 0x0080;
  PlatformP__MoteClockInit__init();
  PlatformP__MoteInit__init();
  PlatformP__LedsInit__init();
  return SUCCESS;
}

# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/Init.nc"
inline static error_t TinyOSMainP__PlatformInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = PlatformP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 119 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__TaskScheduler__init(void )
#line 119
{
  /* atomic removed: atomic calls only */
#line 120
  {
    memset((void *)SchedulerBasicP__m_next, SchedulerBasicP__NO_TASK, sizeof SchedulerBasicP__m_next);
    SchedulerBasicP__m_head = SchedulerBasicP__NO_TASK;
    SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
  }
}

# 58 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/TaskScheduler.nc"
inline static void TinyOSMainP__TaskScheduler__init(void ){
#line 58
  SchedulerBasicP__TaskScheduler__init();
#line 58
}
#line 58
# 78 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyOSMainP.nc"
static inline void TinyOSMainP__TinyOSBoot__booted(void )
#line 78
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 79
    {




      {
      }
#line 84
      ;


      TinyOSMainP__TaskScheduler__init();





      TinyOSMainP__PlatformInit__init();
      while (TinyOSMainP__TaskScheduler__runNextTask()) ;





      TinyOSMainP__SoftwareInit__init();
      while (TinyOSMainP__TaskScheduler__runNextTask()) ;
    }
#line 102
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  TinyOSMainP__Boot__booted();


  TinyOSMainP__TaskScheduler__taskLoop();
}

# 60 "/home/ezio/tinyos-main-read-only/tos/interfaces/Boot.nc"
inline static void TinyThreadSchedulerP__TinyOSBoot__booted(void ){
#line 60
  TinyOSMainP__TinyOSBoot__booted();
#line 60
}
#line 60
# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadQueue.nc"
inline static void TinyThreadSchedulerP__ThreadQueue__init(thread_queue_t *q){
#line 39
  ThreadQueueP__ThreadQueue__init(q);
#line 39
}
#line 39
# 201 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static inline void TinyThreadSchedulerP__ThreadSchedulerBoot__booted(void )
#line 201
{
  TinyThreadSchedulerP__num_runnable_threads = 0;
  TinyThreadSchedulerP__tos_thread = TinyThreadSchedulerP__ThreadInfo__get(TOSTHREAD_TOS_THREAD_ID);
  TinyThreadSchedulerP__tos_thread->id = TOSTHREAD_TOS_THREAD_ID;
  TinyThreadSchedulerP__ThreadQueue__init(&TinyThreadSchedulerP__ready_queue);

  TinyThreadSchedulerP__current_thread = TinyThreadSchedulerP__tos_thread;
  TinyThreadSchedulerP__current_thread->state = TOSTHREAD_STATE_ACTIVE;
  TinyThreadSchedulerP__current_thread->init_block = (void *)0;
  TinyThreadSchedulerP__TinyOSBoot__booted();
}

# 60 "/home/ezio/tinyos-main-read-only/tos/interfaces/Boot.nc"
inline static void RealMainImplP__ThreadSchedulerBoot__booted(void ){
#line 60
  TinyThreadSchedulerP__ThreadSchedulerBoot__booted();
#line 60
}
#line 60
# 36 "/home/ezio/tinyos-main-read-only/tos/platforms/telosb/hardware.h"
static inline  void TOSH_CLR_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r &= ~(1 << 1);
}

# 17 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/platforms/telosa/TelosSerialP.nc"
static inline void TelosSerialP__Resource__granted(void )
#line 17
{
}

# 218 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(uint8_t id)
#line 218
{
}

# 102 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__granted(uint8_t arg_0x7f3447425790){
#line 102
  switch (arg_0x7f3447425790) {
#line 102
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 102
      TelosSerialP__Resource__granted();
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(arg_0x7f3447425790);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 101 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(uint8_t id)
#line 101
{
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__granted(id);
}

# 202 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id)
#line 202
{
}

# 102 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(uint8_t arg_0x7f34472944b0){
#line 102
  switch (arg_0x7f34472944b0) {
#line 102
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 102
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(arg_0x7f34472944b0);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 216 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 216
{
}

# 59 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(uint8_t arg_0x7f3447290c40){
#line 59
  switch (arg_0x7f3447290c40) {
#line 59
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 59
      /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 59
      break;
#line 59
    default:
#line 59
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(arg_0x7f3447290c40);
#line 59
      break;
#line 59
    }
#line 59
}
#line 59
# 190 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void )
#line 190
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 191
    {
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
    }
#line 194
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
}

# 19 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/platforms/telosa/TelosSerialP.nc"
static inline msp430_uart_union_config_t *TelosSerialP__Msp430UartConfigure__getConfig(void )
#line 19
{
  return &TelosSerialP__msp430_uart_telos_config;
}

# 214 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
static inline msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(uint8_t id)
#line 214
{
  return &msp430_uart_default_config;
}

# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartConfigure.nc"
inline static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(uint8_t arg_0x7f344741e020){
#line 39
  union __nesc_unnamed4282 *__nesc_result;
#line 39

#line 39
  switch (arg_0x7f344741e020) {
#line 39
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 39
      __nesc_result = TelosSerialP__Msp430UartConfigure__getConfig();
#line 39
      break;
#line 39
    default:
#line 39
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(arg_0x7f344741e020);
#line 39
      break;
#line 39
    }
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 362 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/chips/msp430/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__disableIntr(void )
#line 362
{
  HplMsp430Usart1P__IE2 &= ~(0x20 | 0x10);
}

#line 350
static inline void HplMsp430Usart1P__Usart__clrIntr(void )
#line 350
{
  HplMsp430Usart1P__IFG2 &= ~(0x20 | 0x10);
}

#line 162
static inline void HplMsp430Usart1P__Usart__resetUsart(bool reset)
#line 162
{
  if (reset) {
    U1CTL = 0x01;
    }
  else {
#line 166
    U1CTL &= ~0x01;
    }
}

# 65 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 6;
}

# 92 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UTXD__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc();
#line 92
}
#line 92
# 223 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/chips/msp430/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableUartTx(void )
#line 223
{
  HplMsp430Usart1P__UTXD__selectModuleFunc();
  HplMsp430Usart1P__ME2 |= 0x20;
}

# 67 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 7);
}

# 99 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__URXD__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc();
#line 99
}
#line 99
# 239 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/chips/msp430/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__disableUartRx(void )
#line 239
{
  HplMsp430Usart1P__ME2 &= ~0x10;
  HplMsp430Usart1P__URXD__selectIOFunc();
}

# 65 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 7;
}

# 92 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__URXD__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc();
#line 92
}
#line 92
# 234 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/chips/msp430/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableUartRx(void )
#line 234
{
  HplMsp430Usart1P__URXD__selectModuleFunc();
  HplMsp430Usart1P__ME2 |= 0x10;
}

# 67 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 6);
}

# 99 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UTXD__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc();
#line 99
}
#line 99
# 228 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/chips/msp430/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__disableUartTx(void )
#line 228
{
  HplMsp430Usart1P__ME2 &= ~0x20;
  HplMsp430Usart1P__UTXD__selectIOFunc();
}

#line 206
static inline void HplMsp430Usart1P__Usart__enableUart(void )
#line 206
{
  /* atomic removed: atomic calls only */
#line 207
  {
    HplMsp430Usart1P__UTXD__selectModuleFunc();
    HplMsp430Usart1P__URXD__selectModuleFunc();
  }
  HplMsp430Usart1P__ME2 |= 0x20 | 0x10;
}

#line 154
static inline void HplMsp430Usart1P__Usart__setUmctl(uint8_t control)
#line 154
{
  U1MCTL = control;
}

#line 143
static inline void HplMsp430Usart1P__Usart__setUbr(uint16_t control)
#line 143
{
  /* atomic removed: atomic calls only */
#line 144
  {
    U1BR0 = control & 0x00FF;
    U1BR1 = (control >> 8) & 0x00FF;
  }
}

#line 286
static inline void HplMsp430Usart1P__configUart(msp430_uart_union_config_t *config)
#line 286
{

  U1CTL = (config->uartRegisters.uctl & ~0x04) | 0x01;
  HplMsp430Usart1P__U1TCTL = config->uartRegisters.utctl;
  HplMsp430Usart1P__U1RCTL = config->uartRegisters.urctl;

  HplMsp430Usart1P__Usart__setUbr(config->uartRegisters.ubr);
  HplMsp430Usart1P__Usart__setUmctl(config->uartRegisters.umctl);
}

static inline void HplMsp430Usart1P__Usart__setModeUart(msp430_uart_union_config_t *config)
#line 296
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 298
    {
      HplMsp430Usart1P__Usart__resetUsart(TRUE);
      HplMsp430Usart1P__Usart__disableSpi();
      HplMsp430Usart1P__configUart(config);
      if (config->uartConfig.utxe == 1 && config->uartConfig.urxe == 1) {
          HplMsp430Usart1P__Usart__enableUart();
        }
      else {
#line 304
        if (config->uartConfig.utxe == 0 && config->uartConfig.urxe == 1) {
            HplMsp430Usart1P__Usart__disableUartTx();
            HplMsp430Usart1P__Usart__enableUartRx();
          }
        else {
#line 307
          if (config->uartConfig.utxe == 1 && config->uartConfig.urxe == 0) {
              HplMsp430Usart1P__Usart__disableUartRx();
              HplMsp430Usart1P__Usart__enableUartTx();
            }
          else 
#line 310
            {
              HplMsp430Usart1P__Usart__disableUart();
            }
          }
        }
#line 313
      HplMsp430Usart1P__Usart__resetUsart(FALSE);
      HplMsp430Usart1P__Usart__clrIntr();
      HplMsp430Usart1P__Usart__disableIntr();
    }
#line 316
    __nesc_atomic_end(__nesc_atomic); }

  return;
}

# 174 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__setModeUart(msp430_uart_union_config_t *config){
#line 174
  HplMsp430Usart1P__Usart__setModeUart(config);
#line 174
}
#line 174
# 67 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )51U &= ~(0x01 << 1);
}

# 99 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__SIMO__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )51U &= ~(0x01 << 2);
}

# 99 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__SOMI__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )51U &= ~(0x01 << 3);
}

# 99 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UCLK__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc();
#line 99
}
#line 99
# 380 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/chips/msp430/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableIntr(void )
#line 380
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 381
    {
      HplMsp430Usart1P__IFG2 &= ~(0x20 | 0x10);
      HplMsp430Usart1P__IE2 |= 0x20 | 0x10;
    }
#line 384
    __nesc_atomic_end(__nesc_atomic); }
}

# 182 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableIntr(void ){
#line 182
  HplMsp430Usart1P__Usart__enableIntr();
#line 182
}
#line 182
# 281 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_uint8(const void * source)
#line 281
{
  const uint8_t *base = source;

#line 283
  return base[0];
}

# 61 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/lib/serial/SerialActiveMessageP.nc"
static inline serial_header_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(message_t * msg)
#line 61
{
  return (serial_header_t * )((uint8_t *)msg + (unsigned short )& ((message_t *)0)->data - sizeof(serial_header_t ));
}

#line 172
static inline am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(message_t *amsg)
#line 172
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 174
  return __nesc_ntoh_uint8(header->type.nxdata);
}

# 50 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static error_t SystemCallP__ThreadScheduler__wakeupThread(thread_id_t id){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = TinyThreadSchedulerP__ThreadScheduler__wakeupThread(id);
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 82 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/SystemCallP.nc"
static inline error_t SystemCallP__SystemCall__finish(syscall_t *s)
#line 82
{
  return SystemCallP__ThreadScheduler__wakeupThread(s->thread->id);
}

# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCall.nc"
inline static error_t /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__SystemCall__finish(syscall_t *s){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = SystemCallP__SystemCall__finish(s);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 104 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingAMSenderImplP.nc"
static inline void /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__AMSend__sendDone(am_id_t am_id, message_t *m, error_t error)
#line 104
{
  if (/*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__send_call != (void *)0) {
      if (/*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__send_call->id == am_id) {
          /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__params_t *p;

#line 108
          p = /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__send_call->params;
          p->error = error;
          /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__SystemCall__finish(/*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__send_call);
        }
    }
}

# 110 "/home/ezio/tinyos-main-read-only/tos/interfaces/AMSend.nc"
inline static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(am_id_t arg_0x7f344767ab30, message_t * msg, error_t error){
#line 110
  /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__AMSend__sendDone(arg_0x7f344767ab30, msg, error);
#line 110
}
#line 110
# 97 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(message_t *msg, error_t result)
#line 97
{
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(msg), msg, result);
}

# 376 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(uart_id_t idxxx, message_t *msg, error_t error)
#line 376
{
  return;
}

# 100 "/home/ezio/tinyos-main-read-only/tos/interfaces/Send.nc"
inline static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(uart_id_t arg_0x7f344752e110, message_t * msg, error_t error){
#line 100
  switch (arg_0x7f344752e110) {
#line 100
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 100
      /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(msg, error);
#line 100
      break;
#line 100
    default:
#line 100
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(arg_0x7f344752e110, msg, error);
#line 100
      break;
#line 100
    }
#line 100
}
#line 100
# 158 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask(void )
#line 158
{
  error_t error;

  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 162
    error = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError;
#line 162
    __nesc_atomic_end(__nesc_atomic); }

  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendCancelled) {
#line 164
    error = ECANCEL;
    }
#line 165
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendId, (message_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer, error);
}

#line 212
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(uint8_t which)
#line 212
{
  if (which) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufOneLocked = 0;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufZeroLocked = 0;
    }
}

# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCall.nc"
inline static error_t /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__SystemCall__finish(syscall_t *s){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = SystemCallP__SystemCall__finish(s);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 164 "/home/ezio/tinyos-main-read-only/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__Timer__stop(uint8_t num)
{
  /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__m_timers[num].isrunning = FALSE;
}

# 78 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static void /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__Timer__stop(uint8_t arg_0x7f34471a47a0){
#line 78
  /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__Timer__stop(arg_0x7f34471a47a0);
#line 78
}
#line 78
# 45 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCallQueue.nc"
inline static syscall_t */*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__SystemCallQueue__find(syscall_queue_t *q, syscall_id_t id){
#line 45
  struct syscall *__nesc_result;
#line 45

#line 45
  __nesc_result = SystemCallQueueP__SystemCallQueue__find(q, id);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 133 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingAMReceiverImplP.nc"
static inline message_t */*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__Receive__receive(uint8_t am_id, message_t *m, void *payload, uint8_t len)
#line 133
{
  syscall_t *s;
  /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__params_t *p;

  if (/*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__blockForAny == TRUE) {
    s = /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__SystemCallQueue__find(&/*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__am_queue, INVALID_ID);
    }
  else {
#line 140
    s = /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__SystemCallQueue__find(&/*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__am_queue, am_id);
    }
#line 141
  if (s == (void *)0) {
#line 141
    return m;
    }
  p = s->params;
  if (p->error == EBUSY) {
      /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__Timer__stop(s->thread->id);
      * p->msg = *m;
      p->error = SUCCESS;
      /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__SystemCall__finish(s);
    }
  return m;
}

# 78 "/home/ezio/tinyos-main-read-only/tos/interfaces/Receive.nc"
inline static message_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__ReceiveDefault__receive(am_id_t arg_0x7f3447675aa0, message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__Receive__receive(arg_0x7f3447675aa0, msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 105 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/lib/serial/SerialActiveMessageP.nc"
static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(uint8_t id, message_t *msg, void *payload, uint8_t len)
#line 105
{
  return /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__ReceiveDefault__receive(id, msg, payload, len);
}

# 78 "/home/ezio/tinyos-main-read-only/tos/interfaces/Receive.nc"
inline static message_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(am_id_t arg_0x7f3447677d10, message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
    __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(arg_0x7f3447677d10, msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 113 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/lib/serial/SerialActiveMessageP.nc"
static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 113
{
  return /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(msg), msg, msg->data, len);
}

# 371 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
static inline message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(uart_id_t idxxx, message_t *msg, 
void *payload, 
uint8_t len)
#line 373
{
  return msg;
}

# 78 "/home/ezio/tinyos-main-read-only/tos/interfaces/Receive.nc"
inline static message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(uart_id_t arg_0x7f344752f570, message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  switch (arg_0x7f344752f570) {
#line 78
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 78
      __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(msg, payload, len);
#line 78
      break;
#line 78
    default:
#line 78
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(arg_0x7f344752f570, msg, payload, len);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 57 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen)
#line 57
{
  return dataLinkLen - sizeof(serial_header_t );
}

# 365 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(uart_id_t id, message_t *msg, 
uint8_t dataLinkLen)
#line 366
{
  return 0;
}

# 31 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(uart_id_t arg_0x7f344752c4d0, message_t *msg, uint8_t dataLinkLen){
#line 31
  unsigned char __nesc_result;
#line 31

#line 31
  switch (arg_0x7f344752c4d0) {
#line 31
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 31
      __nesc_result = SerialPacketInfoActiveMessageP__Info__upperLength(msg, dataLinkLen);
#line 31
      break;
#line 31
    default:
#line 31
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(arg_0x7f344752c4d0, msg, dataLinkLen);
#line 31
      break;
#line 31
    }
#line 31

#line 31
  return __nesc_result;
#line 31
}
#line 31
# 51 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__offset(void )
#line 51
{
  return (uint8_t )(sizeof(message_header_t ) - sizeof(serial_header_t ));
}

# 358 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(uart_id_t id)
#line 358
{
  return 0;
}

# 15 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(uart_id_t arg_0x7f344752c4d0){
#line 15
  unsigned char __nesc_result;
#line 15

#line 15
  switch (arg_0x7f344752c4d0) {
#line 15
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 15
      __nesc_result = SerialPacketInfoActiveMessageP__Info__offset();
#line 15
      break;
#line 15
    default:
#line 15
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(arg_0x7f344752c4d0);
#line 15
      break;
#line 15
    }
#line 15

#line 15
  return __nesc_result;
#line 15
}
#line 15
# 275 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask(void )
#line 275
{
  uart_id_t myType;
  message_t *myBuf;
  uint8_t mySize;
  uint8_t myWhich;

#line 280
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 280
    {
      myType = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskType;
      myBuf = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskBuf;
      mySize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskSize;
      myWhich = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskWhich;
    }
#line 285
    __nesc_atomic_end(__nesc_atomic); }
  mySize -= /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(myType);
  mySize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(myType, myBuf, mySize);
  myBuf = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(myType, myBuf, myBuf, mySize);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 289
    {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messagePtrs[myWhich] = myBuf;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(myWhich);
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending = FALSE;
    }
#line 293
    __nesc_atomic_end(__nesc_atomic); }
}

# 48 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/LinkedList.nc"
inline static list_element_t *SystemCallQueueP__LinkedList__getFirst(linked_list_t *l){
#line 48
  struct list_element *__nesc_result;
#line 48

#line 48
  __nesc_result = LinkedListC__LinkedList__getFirst(l);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48


inline static list_element_t *SystemCallQueueP__LinkedList__getAfter(linked_list_t *l, list_element_t *e){
#line 50
  struct list_element *__nesc_result;
#line 50

#line 50
  __nesc_result = LinkedListC__LinkedList__getAfter(l, e);
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 54 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/LinkedListC.nc"
static inline list_element_t *LinkedListC__get_element(linked_list_t *l, list_element_t *e)
#line 54
{
  list_element_t *temp = l->head;

#line 56
  while (temp != (void *)0) {
      if (temp == e) {
#line 57
        return temp;
        }
#line 58
      temp = temp->next;
    }
  return (void *)0;
}

# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCall.nc"
inline static error_t BlockingStdControlImplP__SystemCall__finish(syscall_t *s){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = SystemCallP__SystemCall__finish(s);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 45 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCallQueue.nc"
inline static syscall_t *BlockingStdControlImplP__SystemCallQueue__find(syscall_queue_t *q, syscall_id_t id){
#line 45
  struct syscall *__nesc_result;
#line 45

#line 45
  __nesc_result = SystemCallQueueP__SystemCallQueue__find(q, id);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 119 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingStdControlImplP.nc"
static inline void BlockingStdControlImplP__SplitControl__stopDone(uint8_t id, error_t error)
#line 119
{
  syscall_t *s = BlockingStdControlImplP__SystemCallQueue__find(&BlockingStdControlImplP__std_cntrl_queue, id);
  BlockingStdControlImplP__params_t *p = s->params;

#line 122
  p->error = error;
  BlockingStdControlImplP__SystemCall__finish(s);
}

# 138 "/home/ezio/tinyos-main-read-only/tos/interfaces/SplitControl.nc"
inline static void SerialP__SplitControl__stopDone(error_t error){
#line 138
  BlockingStdControlImplP__SplitControl__stopDone(/*BlockingSerialActiveMessageC.BlockingStdControlC*/BlockingStdControlC__0__CLIENT_ID, error);
#line 138
}
#line 138
# 112 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/chips/msp430/HplMsp430Usart1P.nc"
static inline error_t HplMsp430Usart1P__AsyncStdControl__stop(void )
#line 112
{
  HplMsp430Usart1P__Usart__disableSpi();
  HplMsp430Usart1P__Usart__disableUart();
  return SUCCESS;
}

# 105 "/home/ezio/tinyos-main-read-only/tos/interfaces/AsyncStdControl.nc"
inline static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__stop(void ){
#line 105
  unsigned char __nesc_result;
#line 105

#line 105
  __nesc_result = HplMsp430Usart1P__AsyncStdControl__stop();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 84 "/home/ezio/tinyos-main-read-only/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup(void )
#line 84
{
}

# 62 "/home/ezio/tinyos-main-read-only/tos/lib/power/PowerDownCleanup.nc"
inline static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__cleanup(void ){
#line 62
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup();
#line 62
}
#line 62
# 79 "/home/ezio/tinyos-main-read-only/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted(void )
#line 79
{
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__cleanup();
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__stop();
}

# 46 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void ){
#line 46
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted();
#line 46
}
#line 46
# 128 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableUart(void ){
#line 128
  HplMsp430Usart1P__Usart__disableUart();
#line 128
}
#line 128
#line 179
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableIntr(void ){
#line 179
  HplMsp430Usart1P__Usart__disableIntr();
#line 179
}
#line 179
#line 97
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__resetUsart(bool reset){
#line 97
  HplMsp430Usart1P__Usart__resetUsart(reset);
#line 97
}
#line 97
# 92 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(uint8_t id)
#line 92
{
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__resetUsart(TRUE);
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableIntr();
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableUart();
}

# 218 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id)
#line 218
{
}

# 65 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(uint8_t arg_0x7f3447290c40){
#line 65
  switch (arg_0x7f3447290c40) {
#line 65
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 65
      /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 65
      break;
#line 65
    default:
#line 65
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(arg_0x7f3447290c40);
#line 65
      break;
#line 65
    }
#line 65
}
#line 65
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 68 "/home/ezio/tinyos-main-read-only/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void )
#line 68
{
  /* atomic removed: atomic calls only */
#line 69
  {
    if (/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead != /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
        uint8_t id = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead;

#line 72
        /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead];
        if (/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead == /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
          /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
          }
#line 75
        /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[id] = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 76
          id;

#line 76
          return __nesc_temp;
        }
      }
#line 78
    {
      unsigned char __nesc_temp = 
#line 78
      /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

#line 78
      return __nesc_temp;
    }
  }
}

# 70 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 60 "/home/ezio/tinyos-main-read-only/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void )
#line 60
{
  /* atomic removed: atomic calls only */
#line 61
  {
    unsigned char __nesc_temp = 
#line 61
    /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead == /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

#line 61
    return __nesc_temp;
  }
}

# 53 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 111 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id)
#line 111
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 112
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY && /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
          if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty() == FALSE) {
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue();
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
            }
          else {
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted();
            }
          {
            unsigned char __nesc_temp = 
#line 127
            SUCCESS;

            {
#line 127
              __nesc_atomic_end(__nesc_atomic); 
#line 127
              return __nesc_temp;
            }
          }
        }
    }
#line 131
    __nesc_atomic_end(__nesc_atomic); }
#line 130
  return FAIL;
}

# 213 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__release(uint8_t id)
#line 213
{
#line 213
  return FAIL;
}

# 120 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
inline static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__release(uint8_t arg_0x7f3447421cb0){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  switch (arg_0x7f3447421cb0) {
#line 120
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 120
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(/*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 120
      break;
#line 120
    default:
#line 120
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__release(arg_0x7f3447421cb0);
#line 120
      break;
#line 120
    }
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 210 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
static inline bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(uint8_t id)
#line 210
{
#line 210
  return FALSE;
}

# 128 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
inline static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(uint8_t arg_0x7f3447421cb0){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  switch (arg_0x7f3447421cb0) {
#line 128
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 128
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(/*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 128
      break;
#line 128
    default:
#line 128
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(arg_0x7f3447421cb0);
#line 128
      break;
#line 128
    }
#line 128

#line 128
  return __nesc_result;
#line 128
}
#line 128
# 77 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__release(uint8_t id)
#line 77
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(id) == FALSE) {
    return FAIL;
    }
#line 80
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf || /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf) {
    return EBUSY;
    }
#line 82
  return /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__release(id);
}

# 120 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
inline static error_t TelosSerialP__Resource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__release(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 13 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/platforms/telosa/TelosSerialP.nc"
static inline error_t TelosSerialP__StdControl__stop(void )
#line 13
{
  TelosSerialP__Resource__release();
  return SUCCESS;
}

# 105 "/home/ezio/tinyos-main-read-only/tos/interfaces/StdControl.nc"
inline static error_t SerialP__SerialControl__stop(void ){
#line 105
  unsigned char __nesc_result;
#line 105

#line 105
  __nesc_result = TelosSerialP__StdControl__stop();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 336 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFlush__flushDone(void )
#line 336
{
  SerialP__SerialControl__stop();
  SerialP__SplitControl__stopDone(SUCCESS);
}

static inline void SerialP__defaultSerialFlushTask__runTask(void )
#line 341
{
  SerialP__SerialFlush__flushDone();
}

# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__defaultSerialFlushTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__defaultSerialFlushTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 344 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFlush__default__flush(void )
#line 344
{
  SerialP__defaultSerialFlushTask__postTask();
}

# 49 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialFlush.nc"
inline static void SerialP__SerialFlush__flush(void ){
#line 49
  SerialP__SerialFlush__default__flush();
#line 49
}
#line 49
# 332 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialP.nc"
static inline void SerialP__stopDoneTask__runTask(void )
#line 332
{
  SerialP__SerialFlush__flush();
}

# 87 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingStdControlImplP.nc"
static inline void BlockingStdControlImplP__SplitControl__startDone(uint8_t id, error_t error)
#line 87
{
  syscall_t *s = BlockingStdControlImplP__SystemCallQueue__find(&BlockingStdControlImplP__std_cntrl_queue, id);
  BlockingStdControlImplP__params_t *p = s->params;

#line 90
  p->error = error;
  BlockingStdControlImplP__SystemCall__finish(s);
}

# 113 "/home/ezio/tinyos-main-read-only/tos/interfaces/SplitControl.nc"
inline static void SerialP__SplitControl__startDone(error_t error){
#line 113
  BlockingStdControlImplP__SplitControl__startDone(/*BlockingSerialActiveMessageC.BlockingStdControlC*/BlockingStdControlC__0__CLIENT_ID, error);
#line 113
}
#line 113
# 133 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void )
#line 133
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 134
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id) {
          if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING) {
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
              {
                unsigned char __nesc_temp = 
#line 138
                SUCCESS;

                {
#line 138
                  __nesc_atomic_end(__nesc_atomic); 
#line 138
                  return __nesc_temp;
                }
              }
            }
          else {
#line 140
            if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING) {
                /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
                /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
                {
                  unsigned char __nesc_temp = 
#line 143
                  SUCCESS;

                  {
#line 143
                    __nesc_atomic_end(__nesc_atomic); 
#line 143
                    return __nesc_temp;
                  }
                }
              }
            }
        }
    }
#line 149
    __nesc_atomic_end(__nesc_atomic); }
#line 147
  return FAIL;
}

# 56 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceDefaultOwner.nc"
inline static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release();
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 108 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/chips/msp430/HplMsp430Usart1P.nc"
static inline error_t HplMsp430Usart1P__AsyncStdControl__start(void )
#line 108
{
  return SUCCESS;
}

# 95 "/home/ezio/tinyos-main-read-only/tos/interfaces/AsyncStdControl.nc"
inline static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = HplMsp430Usart1P__AsyncStdControl__start();
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 74 "/home/ezio/tinyos-main-read-only/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void )
#line 74
{
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start();
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release();
}

# 81 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void ){
#line 81
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested();
#line 81
}
#line 81
# 206 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id)
#line 206
{
}

# 61 "/home/ezio/tinyos-main-read-only/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(uint8_t arg_0x7f3447293840){
#line 61
    /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(arg_0x7f3447293840);
#line 61
}
#line 61
# 93 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id)
#line 93
{
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 95
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) {
          /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING;
          /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = id;
        }
      else {
          unsigned char __nesc_temp = 
#line 100
          FAIL;

          {
#line 100
            __nesc_atomic_end(__nesc_atomic); 
#line 100
            return __nesc_temp;
          }
        }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
#line 102
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested();
  if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
      return SUCCESS;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 107
    /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
#line 107
    __nesc_atomic_end(__nesc_atomic); }
  return FAIL;
}

# 212 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(uint8_t id)
#line 212
{
#line 212
  return FAIL;
}

# 97 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
inline static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__immediateRequest(uint8_t arg_0x7f3447421cb0){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  switch (arg_0x7f3447421cb0) {
#line 97
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 97
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(/*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 97
      break;
#line 97
    default:
#line 97
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(arg_0x7f3447421cb0);
#line 97
      break;
#line 97
    }
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 65 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(uint8_t id)
#line 65
{
  return /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__immediateRequest(id);
}

# 97 "/home/ezio/tinyos-main-read-only/tos/interfaces/Resource.nc"
inline static error_t TelosSerialP__Resource__immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 10 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/platforms/telosa/TelosSerialP.nc"
static inline error_t TelosSerialP__StdControl__start(void )
#line 10
{
  return TelosSerialP__Resource__immediateRequest();
}

# 95 "/home/ezio/tinyos-main-read-only/tos/interfaces/StdControl.nc"
inline static error_t SerialP__SerialControl__start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = TelosSerialP__StdControl__start();
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 322 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialP.nc"
static inline void SerialP__startDoneTask__runTask(void )
#line 322
{
  SerialP__SerialControl__start();
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 324
    {
      SerialP__txState = SerialP__TXSTATE_IDLE;
      SerialP__rxState = SerialP__RXSTATE_NOSYNC;
    }
#line 327
    __nesc_atomic_end(__nesc_atomic); }
  SerialP__SplitControl__startDone(SUCCESS);
}

# 56 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialFrameComm.nc"
inline static error_t SerialP__SerialFrameComm__putDelimiter(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = HdlcTranslateC__SerialFrameComm__putDelimiter();
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 194 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error_t error)
#line 194
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 195
    /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError = error;
#line 195
    __nesc_atomic_end(__nesc_atomic); }
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__postTask();
}

# 91 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SendBytePacket.nc"
inline static void SerialP__SendBytePacket__sendCompleted(error_t error){
#line 91
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error);
#line 91
}
#line 91
# 244 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialP.nc"
static __inline bool SerialP__ack_queue_is_empty(void )
#line 244
{
  bool ret;

#line 246
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 246
    ret = SerialP__ackQ.writePtr == SerialP__ackQ.readPtr;
#line 246
    __nesc_atomic_end(__nesc_atomic); }
  return ret;
}











static __inline uint8_t SerialP__ack_queue_top(void )
#line 260
{
  uint8_t tmp = 0;

  /* atomic removed: atomic calls only */
#line 262
  {
    if (!SerialP__ack_queue_is_empty()) {
        tmp = SerialP__ackQ.buf[SerialP__ackQ.readPtr];
      }
  }
  return tmp;
}

static inline uint8_t SerialP__ack_queue_pop(void )
#line 270
{
  uint8_t retval = 0;

  /* atomic removed: atomic calls only */
#line 272
  {
    if (SerialP__ackQ.writePtr != SerialP__ackQ.readPtr) {
        retval = SerialP__ackQ.buf[SerialP__ackQ.readPtr];
        if (++ SerialP__ackQ.readPtr > SerialP__ACK_QUEUE_SIZE) {
#line 275
          SerialP__ackQ.readPtr = 0;
          }
      }
  }
#line 278
  return retval;
}

#line 559
static inline void SerialP__RunTx__runTask(void )
#line 559
{
  uint8_t idle;
  uint8_t done;
  uint8_t fail;









  error_t result = SUCCESS;
  bool send_completed = FALSE;
  bool start_it = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 576
    {
      SerialP__txPending = 0;
      idle = SerialP__txState == SerialP__TXSTATE_IDLE;
      done = SerialP__txState == SerialP__TXSTATE_FINISH;
      fail = SerialP__txState == SerialP__TXSTATE_ERROR;
      if (done || fail) {
          SerialP__txState = SerialP__TXSTATE_IDLE;
          SerialP__txBuf[SerialP__txIndex].state = SerialP__BUFFER_AVAILABLE;
        }
    }
#line 585
    __nesc_atomic_end(__nesc_atomic); }


  if (done || fail) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 589
        {
          SerialP__txSeqno++;
          if (SerialP__txProto == SERIAL_PROTO_ACK) {
              SerialP__ack_queue_pop();
            }
          else {
              result = done ? SUCCESS : FAIL;
              send_completed = TRUE;
            }
        }
#line 598
        __nesc_atomic_end(__nesc_atomic); }
      idle = TRUE;
    }


  if (idle) {
      bool goInactive;

#line 605
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 605
        goInactive = SerialP__offPending;
#line 605
        __nesc_atomic_end(__nesc_atomic); }
      if (goInactive) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 607
            SerialP__txState = SerialP__TXSTATE_INACTIVE;
#line 607
            __nesc_atomic_end(__nesc_atomic); }
        }
      else {

          uint8_t myAckState;
          uint8_t myDataState;

#line 613
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 613
            {
              myAckState = SerialP__txBuf[SerialP__TX_ACK_INDEX].state;
              myDataState = SerialP__txBuf[SerialP__TX_DATA_INDEX].state;
            }
#line 616
            __nesc_atomic_end(__nesc_atomic); }
          if (!SerialP__ack_queue_is_empty() && myAckState == SerialP__BUFFER_AVAILABLE) {
              { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 618
                {
                  SerialP__txBuf[SerialP__TX_ACK_INDEX].state = SerialP__BUFFER_COMPLETE;
                  SerialP__txBuf[SerialP__TX_ACK_INDEX].buf = SerialP__ack_queue_top();

                  SerialP__txProto = SERIAL_PROTO_ACK;
                  SerialP__txIndex = SerialP__TX_ACK_INDEX;
                  start_it = TRUE;
                }
#line 625
                __nesc_atomic_end(__nesc_atomic); }
            }
          else {
#line 627
            if (myDataState == SerialP__BUFFER_FILLING || myDataState == SerialP__BUFFER_COMPLETE) {
                { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 628
                  {
                    SerialP__txProto = SERIAL_PROTO_PACKET_NOACK;
                    SerialP__txIndex = SerialP__TX_DATA_INDEX;
                    start_it = TRUE;
                  }
#line 632
                  __nesc_atomic_end(__nesc_atomic); }
              }
            else {
              }
            }
        }
    }
  else {
    }


  if (send_completed) {
      SerialP__SendBytePacket__sendCompleted(result);
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 646
    {
      if (SerialP__txState == SerialP__TXSTATE_INACTIVE) {
          SerialP__testOff();
          {
#line 649
            __nesc_atomic_end(__nesc_atomic); 
#line 649
            return;
          }
        }
    }
#line 652
    __nesc_atomic_end(__nesc_atomic); }
  if (start_it) {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 655
        {
          SerialP__txCRC = 0;
          SerialP__txByteCnt = 0;
          SerialP__txState = SerialP__TXSTATE_PROTO;
        }
#line 659
        __nesc_atomic_end(__nesc_atomic); }
      if (SerialP__SerialFrameComm__putDelimiter() != SUCCESS) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 661
            SerialP__txState = SerialP__TXSTATE_ERROR;
#line 661
            __nesc_atomic_end(__nesc_atomic); }
          SerialP__MaybeScheduleTx();
        }
    }
}

# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__stopDoneTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__stopDoneTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 48 "/home/ezio/tinyos-main-read-only/tos/interfaces/UartStream.nc"
inline static error_t HdlcTranslateC__UartStream__send(uint8_t * buf, uint16_t len){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__send(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID, buf, len);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__RunTx__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__RunTx);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCall.nc"
inline static error_t /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__SystemCall__finish(syscall_t *s){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = SystemCallP__SystemCall__finish(s);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 45 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCallQueue.nc"
inline static syscall_t */*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__SystemCallQueue__find(syscall_queue_t *q, syscall_id_t id){
#line 45
  struct syscall *__nesc_result;
#line 45

#line 45
  __nesc_result = SystemCallQueueP__SystemCallQueue__find(q, id);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 89 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingReadP.nc"
static inline void /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__Read__readDone(uint8_t id, error_t result, uint16_t val)
#line 89
{
  syscall_t *s = /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__SystemCallQueue__find(&/*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__read_queue, id);
  /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__read_params_t *p = s->params;

#line 92
  * p->val = val;
  p->error = result;
  /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__SystemCall__finish(s);
}

# 63 "/home/ezio/tinyos-main-read-only/tos/interfaces/Read.nc"
inline static void /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__Read__readDone(error_t result, /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__Read__val_t val){
#line 63
  /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__Read__readDone(/*TestSineSensorAppC.BlockingSineSensorC*/BlockingSineSensorC__0__ID, result, val);
#line 63
}
#line 63
# 33 "/home/ezio/tinyos-main-read-only/tos/system/SineSensorC.nc"
static inline void /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__readTask__runTask(void )
#line 33
{
  float val = (float )/*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__counter;

#line 35
  val = val / 20.0;
  val = sin(val) * 32768.0;
  val += 32768.0;
  /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__counter++;
  /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__Read__readDone(SUCCESS, (uint16_t )val);
}

# 48 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/SystemCallP.nc"
static inline void SystemCallP__threadTask__runTask(void )
#line 48
{
  (* SystemCallP__current_call->syscall_ptr)(SystemCallP__current_call);
}

# 184 "/home/ezio/tinyos-main-read-only/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShotAt(uint8_t num, uint32_t t0, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, t0, dt, TRUE);
}

# 129 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt){
#line 129
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShotAt(1U, t0, dt);
#line 129
}
#line 129
# 164 "/home/ezio/tinyos-main-read-only/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num].isrunning = FALSE;
}

# 78 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__TimerFrom__stop(void ){
#line 78
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(1U);
#line 78
}
#line 78
# 64 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 86 "/home/ezio/tinyos-main-read-only/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
}

# 109 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void ){
#line 109
  unsigned long __nesc_result;
#line 109

#line 109
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 96 "/home/ezio/tinyos-main-read-only/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void )
{
#line 97
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow();
}

# 136 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void ){
#line 136
  unsigned long __nesc_result;
#line 136

#line 136
  __nesc_result = /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow();
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 189 "/home/ezio/tinyos-main-read-only/tos/lib/timer/VirtualizeTimerC.nc"
static inline uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__getNow(uint8_t num)
{
  return /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow();
}

# 136 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static uint32_t /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__TimerFrom__getNow(void ){
#line 136
  unsigned long __nesc_result;
#line 136

#line 136
  __nesc_result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__getNow(1U);
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 100 "/home/ezio/tinyos-main-read-only/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__updateFromTimer__runTask(void )
{




  uint32_t now = /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__TimerFrom__getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint16_t num;

  /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__TimerFrom__stop();

  for (num = 0; num < /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__NUM_TIMERS; num++) 
    {
      /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__Timer_t *timer = &/*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__fireTimers(now);
        }
      else {
#line 135
        /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__TimerFrom__startOneShotAt(now, min_remaining);
        }
    }
}

# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCall.nc"
inline static error_t ThreadSleepP__SystemCall__finish(syscall_t *s){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = SystemCallP__SystemCall__finish(s);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
inline static error_t /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__updateFromTimer__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__updateFromTimer);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 144 "/home/ezio/tinyos-main-read-only/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__Timer_t *timer = &/*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__m_timers[num];

#line 147
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__updateFromTimer__postTask();
}






static inline void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__Timer__startOneShot(uint8_t num, uint32_t dt)
{
  /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__startTimer(num, /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__TimerFrom__getNow(), dt, TRUE);
}

# 73 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static void ThreadSleepP__TimerMilli__startOneShot(uint8_t arg_0x7f34477edb70, uint32_t dt){
#line 73
  /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__Timer__startOneShot(arg_0x7f34477edb70, dt);
#line 73
}
#line 73
# 52 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadSleepP.nc"
static inline void ThreadSleepP__sleepTask(syscall_t *s)
#line 52
{
  ThreadSleepP__sleep_params_t *p = s->params;

#line 54
  ThreadSleepP__TimerMilli__startOneShot(s->thread->id, * p->milli);
}

# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static thread_t *ThreadSleepP__ThreadScheduler__threadInfo(thread_id_t id){
#line 41
  struct thread *__nesc_result;
#line 41

#line 41
  __nesc_result = TinyThreadSchedulerP__ThreadScheduler__threadInfo(id);
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 76 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadSleepP.nc"
static inline void ThreadSleepP__TimerMilli__fired(uint8_t id)
#line 76
{
  thread_t *t = ThreadSleepP__ThreadScheduler__threadInfo(id);

#line 78
  if (t->syscall->syscall_ptr == ThreadSleepP__sleepTask) {
    ThreadSleepP__SystemCall__finish(t->syscall);
    }
}

# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static thread_t */*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__ThreadScheduler__threadInfo(thread_id_t id){
#line 41
  struct thread *__nesc_result;
#line 41

#line 41
  __nesc_result = TinyThreadSchedulerP__ThreadScheduler__threadInfo(id);
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 153 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingAMReceiverImplP.nc"
static inline void /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__Timer__fired(uint8_t id)
#line 153
{
  thread_t *t = /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__ThreadScheduler__threadInfo(id);
  /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__params_t *p = t->syscall->params;

#line 156
  if (p->error == EBUSY) {
      p->error = FAIL;
      /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__SystemCall__finish(t->syscall);
    }
}

# 83 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__Timer__fired(uint8_t arg_0x7f3447ba8020){
#line 83
  /*BlockingSerialAMReceiverP.BlockingAMReceiverImplP*/BlockingAMReceiverImplP__0__Timer__fired(arg_0x7f3447ba8020);
#line 83
  ThreadSleepP__TimerMilli__fired(arg_0x7f3447ba8020);
#line 83
}
#line 83
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 103 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt){
#line 103
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 58 "/home/ezio/tinyos-main-read-only/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(t0, dt);
}

#line 93
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 94
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(t0, dt, TRUE);
}

# 129 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt){
#line 129
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(t0, dt);
#line 129
}
#line 129
# 65 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
}

# 73 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void ){
#line 73
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop();
#line 73
}
#line 73
# 102 "/home/ezio/tinyos-main-read-only/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop();
}

# 73 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void ){
#line 73
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop();
#line 73
}
#line 73
# 71 "/home/ezio/tinyos-main-read-only/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void )
{
#line 72
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop();
}

# 78 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop();
#line 78
}
#line 78
# 100 "/home/ezio/tinyos-main-read-only/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void )
{




  uint32_t now = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint16_t num;

  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop();

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(now);
        }
      else {
#line 135
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(now, min_remaining);
        }
    }
}

# 111 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/LinkedListC.nc"
static inline uint8_t LinkedListC__LinkedList__size(linked_list_t *l)
#line 111
{
  return l->size;
}

# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/LinkedList.nc"
inline static uint8_t ThreadQueueP__LinkedList__size(linked_list_t *l){
#line 41
  unsigned char __nesc_result;
#line 41

#line 41
  __nesc_result = LinkedListC__LinkedList__size(l);
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 57 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadQueueP.nc"
static inline bool ThreadQueueP__ThreadQueue__isEmpty(thread_queue_t *q)
#line 57
{
  return ThreadQueueP__LinkedList__size(& q->l) == 0;
}

# 43 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadQueue.nc"
inline static bool TinyThreadSchedulerP__ThreadQueue__isEmpty(thread_queue_t *q){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = ThreadQueueP__ThreadQueue__isEmpty(q);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 319 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static inline void TinyThreadSchedulerP__PreemptionAlarm__fired(void )
#line 319
{
  TinyThreadSchedulerP__PreemptionAlarm__startOneShot(TOSTHREAD_PREEMPTION_PERIOD);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 321
    {
      if (TinyThreadSchedulerP__ThreadQueue__isEmpty(&TinyThreadSchedulerP__ready_queue) == FALSE) {
          TinyThreadSchedulerP__ThreadScheduler__interruptCurrentThread();
        }
    }
#line 325
    __nesc_atomic_end(__nesc_atomic); }
}

# 139 "/home/ezio/tinyos-main-read-only/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__TimerFrom__fired(void )
{
  /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__fireTimers(/*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__TimerFrom__getNow());
}

#line 204
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num)
{
}

# 83 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(uint8_t arg_0x7f3447ba8020){
#line 83
  switch (arg_0x7f3447ba8020) {
#line 83
    case 0U:
#line 83
      TinyThreadSchedulerP__PreemptionAlarm__fired();
#line 83
      break;
#line 83
    case 1U:
#line 83
      /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__TimerFrom__fired();
#line 83
      break;
#line 83
    default:
#line 83
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(arg_0x7f3447ba8020);
#line 83
      break;
#line 83
    }
#line 83
}
#line 83
# 139 "/home/ezio/tinyos-main-read-only/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow());
}

# 83 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void ){
#line 83
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired();
#line 83
}
#line 83
# 91 "/home/ezio/tinyos-main-read-only/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 93
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type __nesc_temp = 
#line 93
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

      {
#line 93
        __nesc_atomic_end(__nesc_atomic); 
#line 93
        return __nesc_temp;
      }
    }
#line 95
    __nesc_atomic_end(__nesc_atomic); }
}

# 116 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void ){
#line 116
  unsigned long __nesc_result;
#line 116

#line 116
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm();
#line 116

#line 116
  return __nesc_result;
#line 116
}
#line 116
# 74 "/home/ezio/tinyos-main-read-only/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt, FALSE);
    }
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired();
}

# 78 "/home/ezio/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static void TinyThreadSchedulerP__PreemptionAlarm__stop(void ){
#line 78
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(0U);
#line 78
}
#line 78
# 64 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static inline void TinyThreadSchedulerP__alarmTask__runTask(void )
#line 64
{
  uint8_t temp;

#line 66
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 66
    temp = TinyThreadSchedulerP__num_runnable_threads;
#line 66
    __nesc_atomic_end(__nesc_atomic); }
  if (temp <= 1) {
    TinyThreadSchedulerP__PreemptionAlarm__stop();
    }
  else {
#line 69
    if (temp > 1) {
      TinyThreadSchedulerP__PreemptionAlarm__startOneShot(TOSTHREAD_PREEMPTION_PERIOD);
      }
    }
}

# 58 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle(void )
#line 58
{
#line 58
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 58
    * (volatile uint8_t * )49U ^= 0x01 << 4;
#line 58
    __nesc_atomic_end(__nesc_atomic); }
}

# 58 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__toggle(void ){
#line 58
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle();
#line 58
}
#line 58
# 50 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle(void )
#line 50
{
#line 50
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__toggle();
}

# 42 "/home/ezio/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__toggle(void ){
#line 42
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle();
#line 42
}
#line 42
# 84 "/home/ezio/tinyos-main-read-only/tos/system/LedsP.nc"
static inline void LedsP__Leds__led0Toggle(void )
#line 84
{
  LedsP__Led0__toggle();
  ;
#line 86
  ;
}

# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/Leds.nc"
inline static void TestSineSensorC__Leds__led0Toggle(void ){
#line 67
  LedsP__Leds__led0Toggle();
#line 67
}
#line 67
# 50 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static error_t MutexP__ThreadScheduler__wakeupThread(thread_id_t id){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = TinyThreadSchedulerP__ThreadScheduler__wakeupThread(id);
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadQueue.nc"
inline static thread_t *MutexP__ThreadQueue__dequeue(thread_queue_t *q){
#line 41
  struct thread *__nesc_result;
#line 41

#line 41
  __nesc_result = ThreadQueueP__ThreadQueue__dequeue(q);
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 315 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static inline thread_t *TinyThreadSchedulerP__ThreadScheduler__currentThreadInfo(void )
#line 315
{
  /* atomic removed: atomic calls only */
#line 316
  {
    struct thread *__nesc_temp = 
#line 316
    TinyThreadSchedulerP__current_thread;

#line 316
    return __nesc_temp;
  }
}

# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static thread_t *MutexP__ThreadScheduler__currentThreadInfo(void ){
#line 40
  struct thread *__nesc_result;
#line 40

#line 40
  __nesc_result = TinyThreadSchedulerP__ThreadScheduler__currentThreadInfo();
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 67 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/MutexP.nc"
static inline error_t MutexP__Mutex__unlock(mutex_t *m)
#line 67
{
  /* atomic removed: atomic calls only */
#line 68
  {
    if (m->lock == TRUE) {
        thread_t *t = MutexP__ThreadScheduler__currentThreadInfo();

#line 71
        t->mutex_count--;
        if ((t = MutexP__ThreadQueue__dequeue(& m->thread_queue)) != (void *)0) {
          MutexP__ThreadScheduler__wakeupThread(t->id);
          }
        else {
#line 74
          m->lock = FALSE;
          }
      }
#line 76
    {
      unsigned char __nesc_temp = 
#line 76
      SUCCESS;

#line 76
      return __nesc_temp;
    }
  }
}

# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/Mutex.nc"
inline static error_t /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__Mutex__unlock(mutex_t *m){
#line 41
  unsigned char __nesc_result;
#line 41

#line 41
  __nesc_result = MutexP__Mutex__unlock(m);
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 286 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_hton_uint8(void * target, uint8_t value)
#line 286
{
  uint8_t *base = target;

#line 288
  base[0] = value;
  return value;
}

#line 315
static __inline  uint16_t __nesc_hton_uint16(void * target, uint16_t value)
#line 315
{
  uint8_t *base = target;

#line 317
  base[1] = value;
  base[0] = value >> 8;
  return value;
}

# 538 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialP.nc"
static inline error_t SerialP__SendBytePacket__startSend(uint8_t b)
#line 538
{
  bool not_busy = FALSE;

#line 540
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 540
    {
      if (SerialP__txState == SerialP__TXSTATE_INACTIVE) 
        {
          unsigned char __nesc_temp = 
#line 542
          EOFF;

          {
#line 542
            __nesc_atomic_end(__nesc_atomic); 
#line 542
            return __nesc_temp;
          }
        }
    }
#line 545
    __nesc_atomic_end(__nesc_atomic); }
#line 544
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 544
    {
      if (SerialP__txBuf[SerialP__TX_DATA_INDEX].state == SerialP__BUFFER_AVAILABLE) {
          SerialP__txBuf[SerialP__TX_DATA_INDEX].state = SerialP__BUFFER_FILLING;
          SerialP__txBuf[SerialP__TX_DATA_INDEX].buf = b;
          not_busy = TRUE;
        }
    }
#line 550
    __nesc_atomic_end(__nesc_atomic); }
  if (not_busy) {
      SerialP__MaybeScheduleTx();
      return SUCCESS;
    }
  return EBUSY;
}

# 62 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SendBytePacket.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__startSend(uint8_t first_byte){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = SerialP__SendBytePacket__startSend(first_byte);
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 54 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen)
#line 54
{
  return upperLen + sizeof(serial_header_t );
}

# 361 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(uart_id_t id, message_t *msg, 
uint8_t upperLen)
#line 362
{
  return 0;
}

# 23 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(uart_id_t arg_0x7f344752c4d0, message_t *msg, uint8_t upperLen){
#line 23
  unsigned char __nesc_result;
#line 23

#line 23
  switch (arg_0x7f344752c4d0) {
#line 23
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 23
      __nesc_result = SerialPacketInfoActiveMessageP__Info__dataLinkLength(msg, upperLen);
#line 23
      break;
#line 23
    default:
#line 23
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(arg_0x7f344752c4d0, msg, upperLen);
#line 23
      break;
#line 23
    }
#line 23

#line 23
  return __nesc_result;
#line 23
}
#line 23
# 111 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(uint8_t id, message_t *msg, uint8_t len)
#line 111
{
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState != /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE) {
      return EBUSY;
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 116
    {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(id);
      if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex > sizeof(message_header_t )) {
          {
            unsigned char __nesc_temp = 
#line 119
            ESIZE;

            {
#line 119
              __nesc_atomic_end(__nesc_atomic); 
#line 119
              return __nesc_temp;
            }
          }
        }
#line 122
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError = SUCCESS;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer = (uint8_t *)msg;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_DATA;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendId = id;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendCancelled = FALSE;






      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendLen = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(id, msg, len) + /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex;
    }
#line 134
    __nesc_atomic_end(__nesc_atomic); }
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__startSend(id) == SUCCESS) {
      return SUCCESS;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE;
      return FAIL;
    }
}

# 75 "/home/ezio/tinyos-main-read-only/tos/interfaces/Send.nc"
inline static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(TOS_SERIAL_ACTIVE_MESSAGE_ID, msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 69 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/lib/serial/SerialActiveMessageP.nc"
static inline error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(am_id_t id, am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 71
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(msg);

#line 73
  __nesc_hton_uint16(header->dest.nxdata, dest);





  __nesc_hton_uint8(header->type.nxdata, id);
  __nesc_hton_uint8(header->length.nxdata, len);

  return /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__send(msg, len);
}

# 80 "/home/ezio/tinyos-main-read-only/tos/interfaces/AMSend.nc"
inline static error_t /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__AMSend__send(am_id_t arg_0x7f3447145ce0, am_addr_t addr, message_t * msg, uint8_t len){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(arg_0x7f3447145ce0, addr, msg, len);
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 62 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingAMSenderImplP.nc"
static inline void /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__sendTask(syscall_t *s)
#line 62
{
  /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__params_t *p = s->params;

#line 64
  p->error = /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__AMSend__send(s->id, p->addr, p->msg, p->len);
  if (p->error != SUCCESS) {
    /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__SystemCall__finish(s);
    }
}

# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCall.nc"
inline static error_t /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__SystemCall__start(void *syscall_ptr, syscall_t *s, syscall_id_t id, void *params){
#line 39
  unsigned char __nesc_result;
#line 39

#line 39
  __nesc_result = SystemCallP__SystemCall__start(syscall_ptr, s, id, params);
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 47 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static error_t MutexP__ThreadScheduler__suspendCurrentThread(void ){
#line 47
  unsigned char __nesc_result;
#line 47

#line 47
  __nesc_result = TinyThreadSchedulerP__ThreadScheduler__suspendCurrentThread();
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadQueue.nc"
inline static void MutexP__ThreadQueue__enqueue(thread_queue_t *q, thread_t *t){
#line 40
  ThreadQueueP__ThreadQueue__enqueue(q, t);
#line 40
}
#line 40
# 52 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/MutexP.nc"
static inline error_t MutexP__Mutex__lock(mutex_t *m)
#line 52
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 53
    {
      thread_t *t = MutexP__ThreadScheduler__currentThreadInfo();

#line 55
      if (m->lock == FALSE) {
          m->lock = TRUE;
          t->mutex_count++;
        }
      else {
          MutexP__ThreadQueue__enqueue(& m->thread_queue, t);
          MutexP__ThreadScheduler__suspendCurrentThread();
        }
      {
        unsigned char __nesc_temp = 
#line 63
        SUCCESS;

        {
#line 63
          __nesc_atomic_end(__nesc_atomic); 
#line 63
          return __nesc_temp;
        }
      }
    }
#line 66
    __nesc_atomic_end(__nesc_atomic); }
}

# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/Mutex.nc"
inline static error_t /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__Mutex__lock(mutex_t *m){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = MutexP__Mutex__lock(m);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 74 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingAMSenderImplP.nc"
static inline error_t /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__BlockingAMSend__send(am_id_t am_id, am_addr_t addr, message_t *msg, uint8_t len)
#line 74
{
  syscall_t s;
  /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__params_t p;

#line 77
  /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__Mutex__lock(&/*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__my_mutex);
  if (/*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__send_call == (void *)0) {
      /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__send_call = &s;

      p.addr = addr;
      p.msg = msg;
      p.len = len;

      /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__SystemCall__start(&/*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__sendTask, &s, am_id, &p);
      /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__send_call = (void *)0;
    }
  else 
#line 87
    {
      p.error = EBUSY;
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 91
    {
      /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__Mutex__unlock(&/*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__my_mutex);
      {
        unsigned char __nesc_temp = 
#line 93
        p.error;

        {
#line 93
          __nesc_atomic_end(__nesc_atomic); 
#line 93
          return __nesc_temp;
        }
      }
    }
#line 96
    __nesc_atomic_end(__nesc_atomic); }
}

# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/BlockingAMSend.nc"
inline static error_t TestSineSensorC__BlockingAMSend__send(am_addr_t addr, message_t *msg, uint8_t len){
#line 41
  unsigned char __nesc_result;
#line 41

#line 41
  __nesc_result = /*BlockingSerialAMSenderP.BlockingAMSenderImplP*/BlockingAMSenderImplP__0__BlockingAMSend__send(228, addr, msg, len);
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 53 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/LinkedList.nc"
inline static list_element_t *SystemCallQueueP__LinkedList__remove(linked_list_t *l, list_element_t *e){
#line 53
  struct list_element *__nesc_result;
#line 53

#line 53
  __nesc_result = LinkedListC__LinkedList__remove(l, e);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 56 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/SystemCallQueueP.nc"
static inline syscall_t *SystemCallQueueP__SystemCallQueue__remove(syscall_queue_t *q, syscall_t *s)
#line 56
{
  return (syscall_t *)SystemCallQueueP__LinkedList__remove(& q->l, (list_element_t *)s);
}

# 43 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCallQueue.nc"
inline static syscall_t */*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__SystemCallQueue__remove(syscall_queue_t *q, syscall_t *t){
#line 43
  struct syscall *__nesc_result;
#line 43

#line 43
  __nesc_result = SystemCallQueueP__SystemCallQueue__remove(q, t);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
inline static error_t /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__readTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__readTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 41 "/home/ezio/tinyos-main-read-only/tos/system/SineSensorC.nc"
static inline error_t /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__Read__read(void )
#line 41
{
  /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__readTask__postTask();
  return SUCCESS;
}

# 96 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingReadP.nc"
static inline error_t /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__Read__default__read(uint8_t id)
#line 96
{
#line 96
  return FAIL;
}

# 55 "/home/ezio/tinyos-main-read-only/tos/interfaces/Read.nc"
inline static error_t /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__Read__read(uint8_t arg_0x7f3447713e80){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  switch (arg_0x7f3447713e80) {
#line 55
    case /*TestSineSensorAppC.BlockingSineSensorC*/BlockingSineSensorC__0__ID:
#line 55
      __nesc_result = /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__Read__read();
#line 55
      break;
#line 55
    default:
#line 55
      __nesc_result = /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__Read__default__read(arg_0x7f3447713e80);
#line 55
      break;
#line 55
    }
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 63 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingReadP.nc"
static inline void /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__readTask(syscall_t *s)
#line 63
{
  /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__read_params_t *p = s->params;

#line 65
  p->error = /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__Read__read(s->id);
  if (p->error != SUCCESS) {
      /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__SystemCall__finish(s);
    }
}

# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCall.nc"
inline static error_t /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__SystemCall__start(void *syscall_ptr, syscall_t *s, syscall_id_t id, void *params){
#line 39
  unsigned char __nesc_result;
#line 39

#line 39
  __nesc_result = SystemCallP__SystemCall__start(syscall_ptr, s, id, params);
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCallQueue.nc"
inline static void /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__SystemCallQueue__enqueue(syscall_queue_t *q, syscall_t *t){
#line 41
  SystemCallQueueP__SystemCallQueue__enqueue(q, t);
#line 41
}
#line 41
# 71 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingReadP.nc"
static inline error_t /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__BlockingRead__read(uint8_t id, uint16_t *val)
#line 71
{
  syscall_t s;
  /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__read_params_t p;

#line 74
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 74
    {
      if (/*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__SystemCallQueue__find(&/*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__read_queue, id) != (void *)0) 
        {
          unsigned char __nesc_temp = 
#line 76
          EBUSY;

          {
#line 76
            __nesc_atomic_end(__nesc_atomic); 
#line 76
            return __nesc_temp;
          }
        }
#line 77
      /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__SystemCallQueue__enqueue(&/*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__read_queue, &s);
    }
#line 78
    __nesc_atomic_end(__nesc_atomic); }

  p.val = val;
  /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__SystemCall__start(&/*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__readTask, &s, id, &p);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 83
    {
      /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__SystemCallQueue__remove(&/*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__read_queue, &s);
      {
        unsigned char __nesc_temp = 
#line 85
        p.error;

        {
#line 85
          __nesc_atomic_end(__nesc_atomic); 
#line 85
          return __nesc_temp;
        }
      }
    }
#line 88
    __nesc_atomic_end(__nesc_atomic); }
}

# 43 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/BlockingRead.nc"
inline static error_t TestSineSensorC__BlockingRead__read(TestSineSensorC__BlockingRead__val_t *val){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*BlockingSineSensorP.BlockingReadP*/BlockingReadP__0__BlockingRead__read(/*TestSineSensorAppC.BlockingSineSensorC*/BlockingSineSensorC__0__ID, val);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 43 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCallQueue.nc"
inline static syscall_t *BlockingStdControlImplP__SystemCallQueue__remove(syscall_queue_t *q, syscall_t *t){
#line 43
  struct syscall *__nesc_result;
#line 43

#line 43
  __nesc_result = SystemCallQueueP__SystemCallQueue__remove(q, t);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__startDoneTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__startDoneTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 348 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialP.nc"
static inline error_t SerialP__SplitControl__start(void )
#line 348
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 349
    {
      if (SerialP__txState != SerialP__TXSTATE_INACTIVE && SerialP__rxState != SerialP__RXSTATE_INACTIVE) 
        {
          unsigned char __nesc_temp = 
#line 351
          EALREADY;

          {
#line 351
            __nesc_atomic_end(__nesc_atomic); 
#line 351
            return __nesc_temp;
          }
        }
    }
#line 354
    __nesc_atomic_end(__nesc_atomic); }
#line 353
  SerialP__startDoneTask__postTask();
  return SUCCESS;
}

# 125 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingStdControlImplP.nc"
static inline error_t BlockingStdControlImplP__SplitControl__default__start(uint8_t id)
#line 125
{
#line 125
  return FAIL;
}

# 104 "/home/ezio/tinyos-main-read-only/tos/interfaces/SplitControl.nc"
inline static error_t BlockingStdControlImplP__SplitControl__start(uint8_t arg_0x7f34471dba90){
#line 104
  unsigned char __nesc_result;
#line 104

#line 104
  switch (arg_0x7f34471dba90) {
#line 104
    case /*BlockingSerialActiveMessageC.BlockingStdControlC*/BlockingStdControlC__0__CLIENT_ID:
#line 104
      __nesc_result = SerialP__SplitControl__start();
#line 104
      break;
#line 104
    default:
#line 104
      __nesc_result = BlockingStdControlImplP__SplitControl__default__start(arg_0x7f34471dba90);
#line 104
      break;
#line 104
    }
#line 104

#line 104
  return __nesc_result;
#line 104
}
#line 104
# 63 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingStdControlImplP.nc"
static inline void BlockingStdControlImplP__startTask(syscall_t *s)
#line 63
{
  BlockingStdControlImplP__params_t *p = s->params;

#line 65
  p->error = BlockingStdControlImplP__SplitControl__start(s->id);
  if (p->error != SUCCESS) {
    BlockingStdControlImplP__SystemCall__finish(s);
    }
}

# 39 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCall.nc"
inline static error_t BlockingStdControlImplP__SystemCall__start(void *syscall_ptr, syscall_t *s, syscall_id_t id, void *params){
#line 39
  unsigned char __nesc_result;
#line 39

#line 39
  __nesc_result = SystemCallP__SystemCall__start(syscall_ptr, s, id, params);
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/SystemCallQueue.nc"
inline static void BlockingStdControlImplP__SystemCallQueue__enqueue(syscall_queue_t *q, syscall_t *t){
#line 41
  SystemCallQueueP__SystemCallQueue__enqueue(q, t);
#line 41
}
#line 41
# 70 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/BlockingStdControlImplP.nc"
static inline error_t BlockingStdControlImplP__BlockingStdControl__start(uint8_t id)
#line 70
{
  syscall_t s;
  BlockingStdControlImplP__params_t p;

#line 73
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 73
    {
      if (BlockingStdControlImplP__SystemCallQueue__find(&BlockingStdControlImplP__std_cntrl_queue, id) != (void *)0) 
        {
          unsigned char __nesc_temp = 
#line 75
          EBUSY;

          {
#line 75
            __nesc_atomic_end(__nesc_atomic); 
#line 75
            return __nesc_temp;
          }
        }
#line 76
      BlockingStdControlImplP__SystemCallQueue__enqueue(&BlockingStdControlImplP__std_cntrl_queue, &s);
    }
#line 77
    __nesc_atomic_end(__nesc_atomic); }

  BlockingStdControlImplP__SystemCall__start(&BlockingStdControlImplP__startTask, &s, id, &p);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 81
    {
      BlockingStdControlImplP__SystemCallQueue__remove(&BlockingStdControlImplP__std_cntrl_queue, &s);
      {
        unsigned char __nesc_temp = 
#line 83
        p.error;

        {
#line 83
          __nesc_atomic_end(__nesc_atomic); 
#line 83
          return __nesc_temp;
        }
      }
    }
#line 86
    __nesc_atomic_end(__nesc_atomic); }
}

# 41 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/BlockingStdControl.nc"
inline static error_t TestSineSensorC__AMControl__start(void ){
#line 41
  unsigned char __nesc_result;
#line 41

#line 41
  __nesc_result = BlockingStdControlImplP__BlockingStdControl__start(/*BlockingSerialActiveMessageC.BlockingStdControlC*/BlockingStdControlC__0__CLIENT_ID);
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 131 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/lib/serial/SerialActiveMessageP.nc"
static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength(void )
#line 131
{
  return 28;
}

static inline void */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__getPayload(message_t *msg, uint8_t len)
#line 135
{
  if (len > /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength()) {
      return (void *)0;
    }
  else {
      return (void * )msg->data;
    }
}

# 126 "/home/ezio/tinyos-main-read-only/tos/interfaces/Packet.nc"
inline static void * TestSineSensorC__Packet__getPayload(message_t * msg, uint8_t len){
#line 126
  void *__nesc_result;
#line 126

#line 126
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__getPayload(msg, len);
#line 126

#line 126
  return __nesc_result;
#line 126
}
#line 126
# 60 "TestSineSensorC.nc"
static inline void TestSineSensorC__MainThread__run(void *arg)
#line 60
{
  uint16_t *var;
  message_t msg;

#line 63
  var = TestSineSensorC__Packet__getPayload(&msg, sizeof(uint16_t ));

  while (TestSineSensorC__AMControl__start() != SUCCESS) ;
  for (; ; ) {
      while (TestSineSensorC__BlockingRead__read(var) != SUCCESS) ;
      while (TestSineSensorC__BlockingAMSend__send(AM_BROADCAST_ADDR, &msg, sizeof(uint16_t )) != SUCCESS) ;
      TestSineSensorC__Leds__led0Toggle();
    }
}

# 105 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
static inline void StaticThreadP__Thread__default__run(uint8_t id, void *arg)
#line 105
{
}

# 42 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/Thread.nc"
inline static void StaticThreadP__Thread__run(uint8_t arg_0x7f344786ec60, void *arg){
#line 42
  switch (arg_0x7f344786ec60) {
#line 42
    case /*TestSineSensorAppC.MainThread*/ThreadC__0__THREAD_ID:
#line 42
      TestSineSensorC__MainThread__run(arg);
#line 42
      break;
#line 42
    default:
#line 42
      StaticThreadP__Thread__default__run(arg_0x7f344786ec60, arg);
#line 42
      break;
#line 42
    }
#line 42
}
#line 42
# 97 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
static inline void StaticThreadP__ThreadFunction__signalThreadRun(uint8_t id, void *arg)
#line 97
{
  StaticThreadP__Thread__run(id, arg);
}

# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadFunction.nc"
inline static void /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__ThreadFunction__signalThreadRun(void *arg){
#line 37
  StaticThreadP__ThreadFunction__signalThreadRun(/*TestSineSensorAppC.MainThread*/ThreadC__0__THREAD_ID, arg);
#line 37
}
#line 37
# 134 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/LinkedListC.nc"
static inline error_t LinkedListC__LinkedList__addAt(linked_list_t *l, list_element_t *e, uint8_t i)
#line 134
{
  if (i > l->size) {
#line 135
    return FAIL;
    }
  else {
#line 136
    if (i == 0) {
      return LinkedListC__insert_element(l, & l->head, e);
      }
    else 
#line 138
      {
        list_element_t *temp = LinkedListC__get_elementAt(l, i - 1);

#line 140
        return LinkedListC__insert_element(l, & temp->next, e);
      }
    }
}

#line 125
static inline error_t LinkedListC__LinkedList__addLast(linked_list_t *l, list_element_t *e)
#line 125
{
  return LinkedListC__LinkedList__addAt(l, e, l->size);
}

# 44 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/LinkedList.nc"
inline static error_t SystemCallQueueP__LinkedList__addLast(linked_list_t *l, list_element_t *e){
#line 44
  unsigned char __nesc_result;
#line 44

#line 44
  __nesc_result = LinkedListC__LinkedList__addLast(l, e);
#line 44

#line 44
  return __nesc_result;
#line 44
}
#line 44
# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static thread_t *SystemCallP__ThreadScheduler__currentThreadInfo(void ){
#line 40
  struct thread *__nesc_result;
#line 40

#line 40
  __nesc_result = TinyThreadSchedulerP__ThreadScheduler__currentThreadInfo();
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
inline static error_t SystemCallP__threadTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SystemCallP__threadTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 47 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static error_t SystemCallP__ThreadScheduler__suspendCurrentThread(void ){
#line 47
  unsigned char __nesc_result;
#line 47

#line 47
  __nesc_result = TinyThreadSchedulerP__ThreadScheduler__suspendCurrentThread();
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
# 138 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static inline void TinyThreadSchedulerP__suspend(thread_t *thread)
#line 138
{






  TinyThreadSchedulerP__sleepWhileIdle();
  TinyThreadSchedulerP__interrupt(thread);
}

# 391 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
  __dint();
  __nop();
}

# 63 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void )
#line 63
{
  return MSP430_POWER_LPM3;
}

# 62 "/home/ezio/tinyos-main-read-only/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = Msp430ClockP__McuPowerOverride__lowestState();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 74 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC__getPowerState(void )
#line 74
{
  mcu_power_t pState = MSP430_POWER_LPM4;









  if ((((((
#line 77
  TACCTL0 & 0x0010 || 
  TACCTL1 & 0x0010) || 
  TACCTL2 & 0x0010) && (
  TACTL & 0x0300) == 0x0200) || (
  ME1 & (0x80 | 0x40) && U0TCTL & 0x20)) || (
  ME2 & (0x20 | 0x10) && U1TCTL & 0x20))


   || (U0CTLnr & 0x01 && I2CTCTLnr & 0x20 && 
  I2CDCTLnr & 0x20 && U0CTLnr & 0x04 && U0CTLnr & 0x20)) {


    pState = MSP430_POWER_LPM1;
    }


  if (ADC12CTL0 & 0x010) {
      if (ADC12CTL1 & 0x0010) {

          if (ADC12CTL1 & 0x0008) {
            pState = MSP430_POWER_LPM1;
            }
          else {
#line 99
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 100
        if (ADC12CTL1 & 0x0400 && (TACTL & 0x0300) == 0x0200) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

# 379 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/msp430hardware.h"
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 379
{
  return m1 < m2 ? m1 : m2;
}

# 112 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/McuSleepC.nc"
static inline void McuSleepC__computePowerState(void )
#line 112
{
  McuSleepC__powerState = mcombine(McuSleepC__getPowerState(), 
  McuSleepC__McuPowerOverride__lowestState());
}

static inline void McuSleepC__McuSleep__sleep(void )
#line 117
{
  uint16_t temp;

#line 119
  if (McuSleepC__dirty) {
      McuSleepC__computePowerState();
    }

  temp = McuSleepC__msp430PowerBits[McuSleepC__powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
}

# 76 "/home/ezio/tinyos-main-read-only/tos/interfaces/McuSleep.nc"
inline static void TinyThreadSchedulerP__McuSleep__sleep(void ){
#line 76
  McuSleepC__McuSleep__sleep();
#line 76
}
#line 76
# 63 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/LinkedListC.nc"
static inline list_element_t *LinkedListC__get_element_before(linked_list_t *l, list_element_t *e)
#line 63
{
  list_element_t *temp = l->head;

#line 65
  if (temp == (void *)0) {
#line 65
    return (void *)0;
    }
#line 66
  while (temp->next != (void *)0) {
      if (temp->next == e) {
#line 67
        return temp;
        }
#line 67
      ;
      temp = temp->next;
    }
  return (void *)0;
}

# 109 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
static inline void StaticThreadP__ThreadNotification__default__aboutToDestroy(uint8_t id)
#line 109
{
}

# 38 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadNotification.nc"
inline static void StaticThreadP__ThreadNotification__aboutToDestroy(uint8_t arg_0x7f3447868240){
#line 38
    StaticThreadP__ThreadNotification__default__aboutToDestroy(arg_0x7f3447868240);
#line 38
}
#line 38
# 101 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/StaticThreadP.nc"
static inline void StaticThreadP__ThreadCleanup__cleanup(uint8_t id)
#line 101
{
  StaticThreadP__ThreadNotification__aboutToDestroy(id);
}

# 71 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadMapP.nc"
static inline void ThreadMapP__StaticThreadCleanup__default__cleanup(uint8_t id)
#line 71
{
  ThreadMapP__DynamicThreadCleanup__cleanup(id);
}

# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadCleanup.nc"
inline static void ThreadMapP__StaticThreadCleanup__cleanup(uint8_t arg_0x7f3447832110){
#line 37
  switch (arg_0x7f3447832110) {
#line 37
    case /*TestSineSensorAppC.MainThread*/ThreadC__0__THREAD_ID:
#line 37
      StaticThreadP__ThreadCleanup__cleanup(/*TestSineSensorAppC.MainThread*/ThreadC__0__THREAD_ID);
#line 37
      break;
#line 37
    default:
#line 37
      ThreadMapP__StaticThreadCleanup__default__cleanup(arg_0x7f3447832110);
#line 37
      break;
#line 37
    }
#line 37
}
#line 37
# 68 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadMapP.nc"
static inline void ThreadMapP__ThreadCleanup__cleanup(uint8_t id)
#line 68
{
  ThreadMapP__StaticThreadCleanup__cleanup(id);
}

# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadCleanup.nc"
inline static void TinyThreadSchedulerP__ThreadCleanup__cleanup(uint8_t arg_0x7f3447e63710){
#line 37
  ThreadMapP__ThreadCleanup__cleanup(arg_0x7f3447e63710);
#line 37
}
#line 37
# 149 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static inline void TinyThreadSchedulerP__wakeupJoined(thread_t *t)
#line 149
{
  int i;
#line 150
  int j;
#line 150
  int k;

#line 151
  k = 0;
  for (i = 0; i < sizeof  t->joinedOnMe; i++) {
      if (t->joinedOnMe[i] == 0) {
          k += 8;
          continue;
        }
      for (j = 0; j < 8; j++) {
          if (t->joinedOnMe[i] & 0x1) {
            TinyThreadSchedulerP__ThreadScheduler__wakeupThread(k);
            }
#line 160
          t->joinedOnMe[i] >>= 1;
          k++;
        }
    }
}






static inline void TinyThreadSchedulerP__stop(thread_t *t)
#line 171
{
  t->state = TOSTHREAD_STATE_INACTIVE;
  TinyThreadSchedulerP__num_runnable_threads--;
  TinyThreadSchedulerP__wakeupJoined(t);



  if (TinyThreadSchedulerP__num_runnable_threads == 1) {
    TinyThreadSchedulerP__PreemptionAlarm__stop();
    }
  TinyThreadSchedulerP__ThreadCleanup__cleanup(t->id);
}

# 98 "/home/ezio/tinyos-main-read-only/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void ){
#line 98
  unsigned char __nesc_result;
#line 98

#line 98
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 397 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFrameComm__dataReceived(uint8_t data)
#line 397
{
  SerialP__rx_state_machine(FALSE, data);
}

# 94 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__dataReceived(uint8_t data){
#line 94
  SerialP__SerialFrameComm__dataReceived(data);
#line 94
}
#line 94
# 394 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFrameComm__delimiterReceived(void )
#line 394
{
  SerialP__rx_state_machine(TRUE, 0);
}

# 85 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__delimiterReceived(void ){
#line 85
  SerialP__SerialFrameComm__delimiterReceived();
#line 85
}
#line 85
# 73 "/home/ezio/tinyos-main-read-only/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__UartStream__receivedByte(uint8_t data)
#line 73
{






  if (data == HDLC_FLAG_BYTE) {

      HdlcTranslateC__SerialFrameComm__delimiterReceived();
      return;
    }
  else {
#line 85
    if (data == HDLC_CTLESC_BYTE) {

        HdlcTranslateC__state.receiveEscape = 1;
        return;
      }
    else {
#line 90
      if (HdlcTranslateC__state.receiveEscape) {

          HdlcTranslateC__state.receiveEscape = 0;
          data = data ^ 0x20;
        }
      }
    }
#line 95
  HdlcTranslateC__SerialFrameComm__dataReceived(data);
}

# 221 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(uint8_t id, uint8_t byte)
#line 221
{
}

# 79 "/home/ezio/tinyos-main-read-only/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receivedByte(uint8_t arg_0x7f3447423840, uint8_t byte){
#line 79
  switch (arg_0x7f3447423840) {
#line 79
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 79
      HdlcTranslateC__UartStream__receivedByte(byte);
#line 79
      break;
#line 79
    default:
#line 79
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(arg_0x7f3447423840, byte);
#line 79
      break;
#line 79
    }
#line 79
}
#line 79
# 132 "/home/ezio/tinyos-main-read-only/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__UartStream__receiveDone(uint8_t *buf, uint16_t len, error_t error)
#line 132
{
}

# 222 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error)
#line 222
{
}

# 99 "/home/ezio/tinyos-main-read-only/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receiveDone(uint8_t arg_0x7f3447423840, uint8_t * buf, uint16_t len, error_t error){
#line 99
  switch (arg_0x7f3447423840) {
#line 99
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 99
      HdlcTranslateC__UartStream__receiveDone(buf, len, error);
#line 99
      break;
#line 99
    default:
#line 99
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(arg_0x7f3447423840, buf, len, error);
#line 99
      break;
#line 99
    }
#line 99
}
#line 99
# 134 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(uint8_t id, uint8_t data)
#line 134
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf) {
      /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf[/*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_pos++] = data;
      if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_pos >= /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_len) {
          uint8_t *buf = /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf;

#line 139
          /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf = (void *)0;
          /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receiveDone(id, buf, /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_len, SUCCESS);
        }
    }
  else 
#line 142
    {
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receivedByte(id, data);
    }
}

# 65 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 65
{
}

# 54 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(uint8_t arg_0x7f34472d5060, uint8_t data){
#line 54
  switch (arg_0x7f34472d5060) {
#line 54
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 54
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID, data);
#line 54
      break;
#line 54
    default:
#line 54
      /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(arg_0x7f34472d5060, data);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 90 "/home/ezio/tinyos-main-read-only/tos/interfaces/ArbiterInfo.nc"
inline static bool /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void ){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse();
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 54 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(), data);
    }
}

# 54 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart1P__Interrupts__rxDone(uint8_t data){
#line 54
  /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(data);
#line 54
}
#line 54
# 401 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialP.nc"
static inline bool SerialP__valid_rx_proto(uint8_t proto)
#line 401
{
  switch (proto) {
      case SERIAL_PROTO_PACKET_ACK: 
        return TRUE;
      case SERIAL_PROTO_ACK: 
        case SERIAL_PROTO_PACKET_NOACK: 
          default: 
            return FALSE;
    }
}

# 203 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__lockCurrentBuffer(void )
#line 203
{
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufOneLocked = 1;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufZeroLocked = 1;
    }
}

#line 199
static inline bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__isCurrentBufferLocked(void )
#line 199
{
  return /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which ? /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufOneLocked : /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufZeroLocked;
}

#line 226
static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket(void )
#line 226
{
  error_t result = SUCCESS;

  /* atomic removed: atomic calls only */
#line 228
  {
    if (!/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__isCurrentBufferLocked()) {


        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__lockCurrentBuffer();
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_BEGIN;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex = 0;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType = TOS_SERIAL_UNKNOWN_ID;
      }
    else {
        result = EBUSY;
      }
  }
  return result;
}

# 62 "/home/ezio/tinyos-main-read-only/tos/lib/serial/ReceiveBytePacket.nc"
inline static error_t SerialP__ReceiveBytePacket__startPacket(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 311 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialP.nc"
static __inline uint16_t SerialP__rx_current_crc(void )
#line 311
{
  uint16_t crc;
  uint8_t tmp = SerialP__rxBuf.writePtr;

#line 314
  tmp = tmp == 0 ? SerialP__RX_DATA_BUFFER_SIZE : tmp - 1;
  crc = SerialP__rxBuf.buf[tmp] & 0x00ff;
  crc = (crc << 8) & 0xFF00;
  tmp = tmp == 0 ? SerialP__RX_DATA_BUFFER_SIZE : tmp - 1;
  crc |= SerialP__rxBuf.buf[tmp] & 0x00FF;
  return crc;
}

# 80 "/home/ezio/tinyos-main-read-only/tos/lib/serial/ReceiveBytePacket.nc"
inline static void SerialP__ReceiveBytePacket__endPacket(error_t result){
#line 80
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(result);
#line 80
}
#line 80
# 221 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBufferSwap(void )
#line 221
{
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which ? 0 : 1;
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer = (uint8_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messagePtrs[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which];
}

# 67 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 234 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialP.nc"
static __inline bool SerialP__ack_queue_is_full(void )
#line 234
{
  uint8_t tmp;
#line 235
  uint8_t tmp2;

  /* atomic removed: atomic calls only */
#line 236
  {
    tmp = SerialP__ackQ.writePtr;
    tmp2 = SerialP__ackQ.readPtr;
  }
  if (++tmp > SerialP__ACK_QUEUE_SIZE) {
#line 240
    tmp = 0;
    }
#line 241
  return tmp == tmp2;
}







static __inline void SerialP__ack_queue_push(uint8_t token)
#line 250
{
  if (!SerialP__ack_queue_is_full()) {
      /* atomic removed: atomic calls only */
#line 252
      {
        SerialP__ackQ.buf[SerialP__ackQ.writePtr] = token;
        if (++ SerialP__ackQ.writePtr > SerialP__ACK_QUEUE_SIZE) {
#line 254
          SerialP__ackQ.writePtr = 0;
          }
      }
#line 256
      SerialP__MaybeScheduleTx();
    }
}

# 67 "/home/ezio/tinyos-main-read-only/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__SerialFrameComm__resetReceive(void )
#line 67
{
  HdlcTranslateC__state.receiveEscape = 0;
}

# 79 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialFrameComm.nc"
inline static void SerialP__SerialFrameComm__resetReceive(void ){
#line 79
  HdlcTranslateC__SerialFrameComm__resetReceive();
#line 79
}
#line 79
# 244 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(uint8_t b)
#line 244
{
  /* atomic removed: atomic calls only */
#line 245
  {
    switch (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state) {
        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_BEGIN: 
          /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_DATA;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(b);
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType = b;
        break;

        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_DATA: 
          if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex < sizeof(message_t )) {
              /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex] = b;
              /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex++;
            }
          else {
            }




        break;

        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_IDLE: 
          default: 
#line 266
            ;
      }
  }
}

# 69 "/home/ezio/tinyos-main-read-only/tos/lib/serial/ReceiveBytePacket.nc"
inline static void SerialP__ReceiveBytePacket__byteReceived(uint8_t data){
#line 69
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(data);
#line 69
}
#line 69
# 301 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialP.nc"
static __inline uint8_t SerialP__rx_buffer_top(void )
#line 301
{
  uint8_t tmp = SerialP__rxBuf.buf[SerialP__rxBuf.readPtr];

#line 303
  return tmp;
}

#line 305
static __inline uint8_t SerialP__rx_buffer_pop(void )
#line 305
{
  uint8_t tmp = SerialP__rxBuf.buf[SerialP__rxBuf.readPtr];

#line 307
  if (++ SerialP__rxBuf.readPtr > SerialP__RX_DATA_BUFFER_SIZE) {
#line 307
    SerialP__rxBuf.readPtr = 0;
    }
#line 308
  return tmp;
}

#line 297
static __inline void SerialP__rx_buffer_push(uint8_t data)
#line 297
{
  SerialP__rxBuf.buf[SerialP__rxBuf.writePtr] = data;
  if (++ SerialP__rxBuf.writePtr > SerialP__RX_DATA_BUFFER_SIZE) {
#line 299
    SerialP__rxBuf.writePtr = 0;
    }
}

# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/PlatformInterrupt.nc"
inline static void HplMsp430Usart1P__PlatformInterrupt__postAmble(void ){
#line 37
  TOSThreadsInterruptP__PlatformInterrupt__postAmble();
#line 37
}
#line 37
# 220 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error)
#line 220
{
}

# 57 "/home/ezio/tinyos-main-read-only/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(uint8_t arg_0x7f3447423840, uint8_t * buf, uint16_t len, error_t error){
#line 57
  switch (arg_0x7f3447423840) {
#line 57
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 57
      HdlcTranslateC__UartStream__sendDone(buf, len, error);
#line 57
      break;
#line 57
    default:
#line 57
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(arg_0x7f3447423840, buf, len, error);
#line 57
      break;
#line 57
    }
#line 57
}
#line 57
# 387 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/chips/msp430/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__tx(uint8_t data)
#line 387
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 388
    HplMsp430Usart1P__U1TXBUF = data;
#line 388
    __nesc_atomic_end(__nesc_atomic); }
}

# 224 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(uint8_t data){
#line 224
  HplMsp430Usart1P__Usart__tx(data);
#line 224
}
#line 224
# 162 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(uint8_t id)
#line 162
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__current_owner != id) {
      uint8_t *buf = /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf;

#line 165
      /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf = (void *)0;
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(id, buf, /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len, FAIL);
    }
  else {
#line 168
    if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos < /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len) {
        /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf[/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos++]);
      }
    else {
        uint8_t *buf = /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf;

#line 173
        /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf = (void *)0;
        /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(id, buf, /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len, SUCCESS);
      }
    }
}

# 64 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id)
#line 64
{
}

# 49 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(uint8_t arg_0x7f34472d5060){
#line 49
  switch (arg_0x7f34472d5060) {
#line 49
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 49
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(arg_0x7f34472d5060);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 49 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void )
#line 49
{
  if (/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId());
    }
}

# 49 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart1P__Interrupts__txDone(void ){
#line 49
  /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone();
#line 49
}
#line 49
# 65 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialFrameComm.nc"
inline static error_t SerialP__SerialFrameComm__putData(uint8_t data){
#line 65
  unsigned char __nesc_result;
#line 65

#line 65
  __nesc_result = HdlcTranslateC__SerialFrameComm__putData(data);
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 529 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialP.nc"
static inline error_t SerialP__SendBytePacket__completeSend(void )
#line 529
{
  bool ret = FAIL;

  /* atomic removed: atomic calls only */
#line 531
  {
    SerialP__txBuf[SerialP__TX_DATA_INDEX].state = SerialP__BUFFER_COMPLETE;
    ret = SUCCESS;
  }
  return ret;
}

# 71 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SendBytePacket.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__completeSend(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = SerialP__SendBytePacket__completeSend();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 178 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte(void )
#line 178
{
  uint8_t b;
  uint8_t indx;

  /* atomic removed: atomic calls only */
#line 181
  {
    b = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex];
    /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex++;
    indx = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex;
  }
  if (indx > /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendLen) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__completeSend();
      return 0;
    }
  else {
      return b;
    }
}

# 81 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SendBytePacket.nc"
inline static uint8_t SerialP__SendBytePacket__nextByte(void ){
#line 81
  unsigned char __nesc_result;
#line 81

#line 81
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte();
#line 81

#line 81
  return __nesc_result;
#line 81
}
#line 81
# 668 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFrameComm__putDone(void )
#line 668
{
  {
    error_t txResult = SUCCESS;

    /* atomic removed: atomic calls only */
#line 671
    {
      switch (SerialP__txState) {

          case SerialP__TXSTATE_PROTO: 

            txResult = SerialP__SerialFrameComm__putData(SerialP__txProto);

          SerialP__txState = SerialP__TXSTATE_INFO;



          SerialP__txCRC = crcByte(SerialP__txCRC, SerialP__txProto);
          break;

          case SerialP__TXSTATE_SEQNO: 
            txResult = SerialP__SerialFrameComm__putData(SerialP__txSeqno);
          SerialP__txState = SerialP__TXSTATE_INFO;
          SerialP__txCRC = crcByte(SerialP__txCRC, SerialP__txSeqno);
          break;

          case SerialP__TXSTATE_INFO: 
            {
              txResult = SerialP__SerialFrameComm__putData(SerialP__txBuf[SerialP__txIndex].buf);
              SerialP__txCRC = crcByte(SerialP__txCRC, SerialP__txBuf[SerialP__txIndex].buf);
              ++SerialP__txByteCnt;

              if (SerialP__txIndex == SerialP__TX_DATA_INDEX) {
                  uint8_t nextByte;

#line 699
                  nextByte = SerialP__SendBytePacket__nextByte();
                  if (SerialP__txBuf[SerialP__txIndex].state == SerialP__BUFFER_COMPLETE || SerialP__txByteCnt >= SerialP__SERIAL_MTU) {
                      SerialP__txState = SerialP__TXSTATE_FCS1;
                    }
                  else {
                      SerialP__txBuf[SerialP__txIndex].buf = nextByte;
                    }
                }
              else {
                  SerialP__txState = SerialP__TXSTATE_FCS1;
                }
            }
          break;

          case SerialP__TXSTATE_FCS1: 
            txResult = SerialP__SerialFrameComm__putData(SerialP__txCRC & 0xff);
          SerialP__txState = SerialP__TXSTATE_FCS2;
          break;

          case SerialP__TXSTATE_FCS2: 
            txResult = SerialP__SerialFrameComm__putData((SerialP__txCRC >> 8) & 0xff);
          SerialP__txState = SerialP__TXSTATE_ENDFLAG;
          break;

          case SerialP__TXSTATE_ENDFLAG: 
            txResult = SerialP__SerialFrameComm__putDelimiter();
          SerialP__txState = SerialP__TXSTATE_ENDWAIT;
          break;

          case SerialP__TXSTATE_ENDWAIT: 
            SerialP__txState = SerialP__TXSTATE_FINISH;
          case SerialP__TXSTATE_FINISH: 
            SerialP__MaybeScheduleTx();
          break;
          case SerialP__TXSTATE_ERROR: 
            default: 
              txResult = FAIL;
          break;
        }

      if (txResult != SUCCESS) {
          SerialP__txState = SerialP__TXSTATE_ERROR;
          SerialP__MaybeScheduleTx();
        }
    }
  }
}

# 100 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__putDone(void ){
#line 100
  SerialP__SerialFrameComm__putDone();
#line 100
}
#line 100
# 411 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/msp430hardware.h"
  __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (__read_status_register() & 0x0008) != 0;

#line 414
  __nesc_disable_interrupt();
   __asm volatile ("" :  :  : "memory");
  return result;
}

  void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
   __asm volatile ("" :  :  : "memory");
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

# 12 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/chips/msp430/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x000C)))  void sig_TIMERA0_VECTOR(void )
#line 12
{
  Msp430TimerCommonP__VectorTimerA0__fired();
  Msp430TimerCommonP__PlatformInterrupt__postAmble();
}

# 180 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired();
    }
}

#line 180
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired();
    }
}

#line 180
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired();
    }
}

# 46 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TOSThreadsInterruptP.nc"
static __attribute((noinline)) void TOSThreadsInterruptP__interruptThread(void )
#line 46
{
  if (TOSThreadsInterruptP__ThreadScheduler__wakeupThread(TOSTHREAD_TOS_THREAD_ID) == SUCCESS) {
    if (TOSThreadsInterruptP__ThreadScheduler__currentThreadId() != TOSTHREAD_TOS_THREAD_ID) {
      TOSThreadsInterruptP__ThreadScheduler__interruptCurrentThread();
      }
    }
}

# 291 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static error_t TinyThreadSchedulerP__ThreadScheduler__wakeupThread(uint8_t id)
#line 291
{
  thread_t *t = TinyThreadSchedulerP__ThreadInfo__get(id);

#line 293
  if (t->state == TOSTHREAD_STATE_SUSPENDED) {
      t->state = TOSTHREAD_STATE_READY;
      if (t != TinyThreadSchedulerP__tos_thread) {
          TinyThreadSchedulerP__ThreadQueue__enqueue(&TinyThreadSchedulerP__ready_queue, TinyThreadSchedulerP__ThreadInfo__get(id));
        }




      return SUCCESS;
    }
  return FAIL;
}

# 62 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadMapP.nc"
static thread_t *ThreadMapP__DynamicThreadInfo__default__get(uint8_t id)
#line 62
{
  return ThreadMapP__StaticThreadInfo__get(id);
}

# 40 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t *ThreadMapP__DynamicThreadInfo__get(uint8_t arg_0x7f34478309b0){
#line 40
  struct thread *__nesc_result;
#line 40

#line 40
    __nesc_result = ThreadMapP__DynamicThreadInfo__default__get(arg_0x7f34478309b0);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 84 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/LinkedListC.nc"
static error_t LinkedListC__insert_element(linked_list_t *l, list_element_t **previous_next, list_element_t *e)
#line 84
{
  if (e == (void *)0) {
#line 85
    return FAIL;
    }
#line 86
  e->next = *previous_next;
  *previous_next = e;
  l->size++;
  return SUCCESS;
}

# 264 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static error_t TinyThreadSchedulerP__ThreadScheduler__interruptCurrentThread(void )
#line 264
{
  /* atomic removed: atomic calls only */
#line 265
  {
    if (TinyThreadSchedulerP__current_thread->state == TOSTHREAD_STATE_ACTIVE) {
        TinyThreadSchedulerP__current_thread->state = TOSTHREAD_STATE_READY;
        if (TinyThreadSchedulerP__current_thread != TinyThreadSchedulerP__tos_thread) {
          TinyThreadSchedulerP__ThreadQueue__enqueue(&TinyThreadSchedulerP__ready_queue, TinyThreadSchedulerP__current_thread);
          }
#line 270
        TinyThreadSchedulerP__interrupt(TinyThreadSchedulerP__current_thread);
        {
          unsigned char __nesc_temp = 
#line 271
          SUCCESS;

#line 271
          return __nesc_temp;
        }
      }
#line 273
    {
      unsigned char __nesc_temp = 
#line 273
      FAIL;

#line 273
      return __nesc_temp;
    }
  }
}

#line 124
static void TinyThreadSchedulerP__interrupt(thread_t *thread)
#line 124
{
  TinyThreadSchedulerP__yielding_thread = thread;
  TinyThreadSchedulerP__scheduleNextThread();
  if (TinyThreadSchedulerP__current_thread != TinyThreadSchedulerP__yielding_thread) {
      TinyThreadSchedulerP__switchThreads();
    }
}

#line 111
static void TinyThreadSchedulerP__scheduleNextThread(void )
#line 111
{
  if (TinyThreadSchedulerP__tos_thread->state == TOSTHREAD_STATE_READY) {
    TinyThreadSchedulerP__current_thread = TinyThreadSchedulerP__tos_thread;
    }
  else {
#line 115
    TinyThreadSchedulerP__current_thread = TinyThreadSchedulerP__ThreadQueue__dequeue(&TinyThreadSchedulerP__ready_queue);
    }
  TinyThreadSchedulerP__current_thread->state = TOSTHREAD_STATE_ACTIVE;
}

# 148 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/LinkedListC.nc"
static list_element_t *LinkedListC__LinkedList__removeAt(linked_list_t *l, uint8_t i)
#line 148
{
  if (i == 0) {
    return LinkedListC__LinkedList__removeFirst(l);
    }
  else 
#line 151
    {
      list_element_t *temp = LinkedListC__get_elementAt(l, i - 1);

#line 153
      if (temp == (void *)0) {
#line 153
        return (void *)0;
        }
      else {
#line 154
        return LinkedListC__remove_element(l, & temp->next);
        }
    }
}

#line 92
static list_element_t *LinkedListC__remove_element(linked_list_t *l, list_element_t **previous_next)
#line 92
{
  list_element_t *e = *previous_next;

#line 94
  *previous_next = (*previous_next)->next;
  e->next = (void *)0;
  l->size--;
  return e;
}

#line 42
static list_element_t *LinkedListC__get_elementAt(linked_list_t *l, uint8_t i)
#line 42
{
  if (i >= l->size) {
#line 43
    return (void *)0;
    }
  else {
#line 44
    if (l->head == (void *)0) {
#line 44
      return (void *)0;
      }
    else 
#line 45
      {
        list_element_t *temp = l->head;

#line 47
        while (i-- > 0) {
            temp = temp->next;
          }
        return temp;
      }
    }
}

# 84 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static __attribute((noinline)) void TinyThreadSchedulerP__switchThreads(void )
#line 84
{
   __asm ("mov.w r4,%0" : "=m"(TinyThreadSchedulerP__yielding_thread->regs.r4)); __asm ("mov.w r5,%0" : "=m"(TinyThreadSchedulerP__yielding_thread->regs.r5)); __asm ("mov.w r6,%0" : "=m"(TinyThreadSchedulerP__yielding_thread->regs.r6)); __asm ("mov.w r7,%0" : "=m"(TinyThreadSchedulerP__yielding_thread->regs.r7)); __asm ("mov.w r8,%0" : "=m"(TinyThreadSchedulerP__yielding_thread->regs.r8)); __asm ("mov.w r9,%0" : "=m"(TinyThreadSchedulerP__yielding_thread->regs.r9)); __asm ("mov.w r10,%0" : "=m"(TinyThreadSchedulerP__yielding_thread->regs.r10)); __asm ("mov.w r11,%0" : "=m"(TinyThreadSchedulerP__yielding_thread->regs.r11)); __asm ("mov.w r12,%0" : "=m"(TinyThreadSchedulerP__yielding_thread->regs.r12)); __asm ("mov.w r13,%0" : "=m"(TinyThreadSchedulerP__yielding_thread->regs.r13)); __asm ("mov.w r14,%0" : "=m"(TinyThreadSchedulerP__yielding_thread->regs.r14)); __asm ("mov.w r15,%0" : "=m"(TinyThreadSchedulerP__yielding_thread->regs.r15)); __asm ("mov.w r2,%0" : "=r"(TinyThreadSchedulerP__yielding_thread->regs.status)); __asm ("mov.w r1,%0" : "=m"(TinyThreadSchedulerP__yielding_thread->stack_ptr)); __asm ("mov.w %0,r1" :  : "m"(TinyThreadSchedulerP__current_thread->stack_ptr)); __asm ("mov.w %0,r2" :  : "r"(TinyThreadSchedulerP__current_thread->regs.status)); __asm ("mov.w %0,r4" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r4)); __asm ("mov.w %0,r5" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r5)); __asm ("mov.w %0,r6" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r6)); __asm ("mov.w %0,r7" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r7)); __asm ("mov.w %0,r8" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r8)); __asm ("mov.w %0,r9" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r9)); __asm ("mov.w %0,r10" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r10)); __asm ("mov.w %0,r11" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r11)); __asm ("mov.w %0,r12" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r12)); __asm ("mov.w %0,r13" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r13)); __asm ("mov.w %0,r14" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r14)); __asm ("mov.w %0,r15" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r15));}

# 16 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/chips/msp430/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x000A)))  void sig_TIMERA1_VECTOR(void )
#line 16
{
  Msp430TimerCommonP__VectorTimerA1__fired();
  Msp430TimerCommonP__PlatformInterrupt__postAmble();
}

#line 20
__attribute((wakeup)) __attribute((interrupt(0x001A)))  void sig_TIMERB0_VECTOR(void )
#line 20
{
  Msp430TimerCommonP__VectorTimerB0__fired();
  Msp430TimerCommonP__PlatformInterrupt__postAmble();
}

# 146 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n)
{
}

# 39 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0x7f3447f5e110){
#line 39
  switch (arg_0x7f3447f5e110) {
#line 39
    case 0:
#line 39
      /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired();
#line 39
      break;
#line 39
    case 1:
#line 39
      /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired();
#line 39
      break;
#line 39
    case 2:
#line 39
      /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired();
#line 39
      break;
#line 39
    case 3:
#line 39
      /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired();
#line 39
      break;
#line 39
    case 4:
#line 39
      /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired();
#line 39
      break;
#line 39
    case 5:
#line 39
      /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired();
#line 39
      break;
#line 39
    case 6:
#line 39
      /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired();
#line 39
      break;
#line 39
    case 7:
#line 39
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired();
#line 39
      break;
#line 39
    default:
#line 39
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0x7f3447f5e110);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 156 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id)
#line 156
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 157
    {
#line 157
      {
        unsigned char __nesc_temp = 
#line 157
        SchedulerBasicP__pushTask(id) ? SUCCESS : EBUSY;

        {
#line 157
          __nesc_atomic_end(__nesc_atomic); 
#line 157
          return __nesc_temp;
        }
      }
    }
#line 160
    __nesc_atomic_end(__nesc_atomic); }
}

# 107 "/home/ezio/tinyos-main-read-only/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
#line 109
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type expires;
#line 109
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 132
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
    }
  else 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type )now << 5, 
  (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type )remaining << 5);
}

# 80 "/home/ezio/tinyos-main-read-only/tos/lib/timer/TransformCounterC.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void )
{
  /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type rv = 0;

#line 83
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();

#line 87
      if (/*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT;

#line 101
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC__0__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 62 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void )
{




  if (1) {
      /* atomic removed: atomic calls only */
#line 69
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )400U;

#line 72
        do {
#line 72
            t0 = t1;
#line 72
            t1 = * (volatile uint16_t * )400U;
          }
        while (
#line 72
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 73
          t1;

#line 73
          return __nesc_temp;
        }
      }
    }
  else 
#line 76
    {
      return * (volatile uint16_t * )400U;
    }
}

# 24 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/chips/msp430/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0018)))  void sig_TIMERB1_VECTOR(void )
#line 24
{
  Msp430TimerCommonP__VectorTimerB1__fired();
  Msp430TimerCommonP__PlatformInterrupt__postAmble();
}

# 60 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/RealMainImplP.nc"
  int main(void )
#line 60
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 61
    {

      RealMainImplP__ThreadSchedulerBoot__booted();
    }
#line 64
    __nesc_atomic_end(__nesc_atomic); }




  return -1;
}

# 175 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/timer/Msp430ClockP.nc"
static void Msp430ClockP__set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

# 16 "/home/ezio/tinyos-main-read-only/tos/platforms/telosb/MotePlatformC.nc"
static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set)
#line 16
{
  if (set) {
    TOSH_SET_SIMO0_PIN();
    }
  else {
#line 20
    TOSH_CLR_SIMO0_PIN();
    }
#line 21
  TOSH_SET_UCLK0_PIN();
  TOSH_CLR_UCLK0_PIN();
}

# 127 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/SchedulerBasicP.nc"
static bool SchedulerBasicP__TaskScheduler__runNextTask(void )
#line 127
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 129
  {
    nextTask = SchedulerBasicP__popTask();
    if (nextTask == SchedulerBasicP__NO_TASK) {
        {
          unsigned char __nesc_temp = 
#line 132
          FALSE;

#line 132
          return __nesc_temp;
        }
      }
  }
#line 135
  SchedulerBasicP__TaskBasic__runTask(nextTask);
  return TRUE;
}

#line 160
static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id)
#line 160
{
}

# 75 "/home/ezio/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x7f3448052020){
#line 75
  switch (arg_0x7f3448052020) {
#line 75
    case TinyThreadSchedulerP__alarmTask:
#line 75
      TinyThreadSchedulerP__alarmTask__runTask();
#line 75
      break;
#line 75
    case /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired:
#line 75
      /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask();
#line 75
      break;
#line 75
    case /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer:
#line 75
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask();
#line 75
      break;
#line 75
    case /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__updateFromTimer:
#line 75
      /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__updateFromTimer__runTask();
#line 75
      break;
#line 75
    case SystemCallP__threadTask:
#line 75
      SystemCallP__threadTask__runTask();
#line 75
      break;
#line 75
    case /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__readTask:
#line 75
      /*TestSineSensorAppC.BlockingSineSensorC.SineSensorC*/SineSensorC__0__readTask__runTask();
#line 75
      break;
#line 75
    case SerialP__RunTx:
#line 75
      SerialP__RunTx__runTask();
#line 75
      break;
#line 75
    case SerialP__startDoneTask:
#line 75
      SerialP__startDoneTask__runTask();
#line 75
      break;
#line 75
    case SerialP__stopDoneTask:
#line 75
      SerialP__stopDoneTask__runTask();
#line 75
      break;
#line 75
    case SerialP__defaultSerialFlushTask:
#line 75
      SerialP__defaultSerialFlushTask__runTask();
#line 75
      break;
#line 75
    case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone:
#line 75
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask();
#line 75
      break;
#line 75
    case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask:
#line 75
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask();
#line 75
      break;
#line 75
    case /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask:
#line 75
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask();
#line 75
      break;
#line 75
    default:
#line 75
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x7f3448052020);
#line 75
      break;
#line 75
    }
#line 75
}
#line 75
# 85 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(uint8_t id)
#line 85
{
  msp430_uart_union_config_t *config = /*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(id);

#line 87
  /*Msp430Uart1P.UartP*/Msp430UartP__0__m_byte_time = config->uartConfig.ubr / 2;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__setModeUart(config);
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableIntr();
}

# 254 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/chips/msp430/HplMsp430Usart1P.nc"
static void HplMsp430Usart1P__Usart__disableSpi(void )
#line 254
{
  /* atomic removed: atomic calls only */
#line 255
  {
    HplMsp430Usart1P__ME2 &= ~0x10;
    HplMsp430Usart1P__SIMO__selectIOFunc();
    HplMsp430Usart1P__SOMI__selectIOFunc();
    HplMsp430Usart1P__UCLK__selectIOFunc();
  }
}

#line 214
static void HplMsp430Usart1P__Usart__disableUart(void )
#line 214
{
  /* atomic removed: atomic calls only */
#line 215
  {
    HplMsp430Usart1P__ME2 &= ~(0x20 | 0x10);
    HplMsp430Usart1P__UTXD__selectIOFunc();
    HplMsp430Usart1P__URXD__selectIOFunc();
  }
}

# 59 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/SystemCallQueueP.nc"
static syscall_t *SystemCallQueueP__SystemCallQueue__find(syscall_queue_t *q, uint8_t id)
#line 59
{
  syscall_t *s;

#line 61
  for (s = (syscall_t *)SystemCallQueueP__LinkedList__getFirst(& q->l); 
  s != (void *)0; 
  s = (syscall_t *)SystemCallQueueP__LinkedList__getAfter(& q->l, (list_element_t *)s)) {
      if (s->id == id) {
#line 64
        return s;
        }
    }
#line 66
  return (void *)0;
}

# 117 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/LinkedListC.nc"
static list_element_t *LinkedListC__LinkedList__getFirst(linked_list_t *l)
#line 117
{
  if (l->head == (void *)0) {
#line 118
    return (void *)0;
    }
#line 119
  return l->head;
}

#line 171
static list_element_t *LinkedListC__LinkedList__getAfter(linked_list_t *l, list_element_t *e)
#line 171
{
  list_element_t *temp = LinkedListC__get_element(l, e);

#line 173
  if (temp == (void *)0) {
#line 173
    return (void *)0;
    }
#line 174
  if (temp->next == (void *)0) {
#line 174
    return (void *)0;
    }
#line 175
  return temp->next;
}

# 177 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id)
#line 177
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 178
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId == id && /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) {
          unsigned char __nesc_temp = 
#line 179
          TRUE;

          {
#line 179
            __nesc_atomic_end(__nesc_atomic); 
#line 179
            return __nesc_temp;
          }
        }
      else 
#line 180
        {
          unsigned char __nesc_temp = 
#line 180
          FALSE;

          {
#line 180
            __nesc_atomic_end(__nesc_atomic); 
#line 180
            return __nesc_temp;
          }
        }
    }
#line 183
    __nesc_atomic_end(__nesc_atomic); }
}

# 357 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialP.nc"
static void SerialP__testOff(void )
#line 357
{
  bool turnOff = FALSE;

  /* atomic removed: atomic calls only */
#line 359
  {
    if (SerialP__txState == SerialP__TXSTATE_INACTIVE && 
    SerialP__rxState == SerialP__RXSTATE_INACTIVE) {
        turnOff = TRUE;
      }
  }
  if (turnOff) {
      SerialP__stopDoneTask__postTask();
      /* atomic removed: atomic calls only */
#line 367
      SerialP__offPending = FALSE;
    }
  else {
      /* atomic removed: atomic calls only */
#line 370
      SerialP__offPending = TRUE;
    }
}

# 98 "/home/ezio/tinyos-main-read-only/tos/lib/serial/HdlcTranslateC.nc"
static error_t HdlcTranslateC__SerialFrameComm__putDelimiter(void )
#line 98
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 99
    {
      HdlcTranslateC__state.sendEscape = 0;
      HdlcTranslateC__m_data = HDLC_FLAG_BYTE;
    }
#line 102
    __nesc_atomic_end(__nesc_atomic); }
  return HdlcTranslateC__UartStream__send(&HdlcTranslateC__m_data, 1);
}

# 147 "/home/ezio/tinyos-main-read-only/tos/chips/msp430/usart/Msp430UartP.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__send(uint8_t id, uint8_t *buf, uint16_t len)
#line 147
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(id) == FALSE) {
    return FAIL;
    }
#line 150
  if (len == 0) {
    return FAIL;
    }
  else {
#line 152
    if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf) {
      return EBUSY;
      }
    }
#line 154
  /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf = buf;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len = len;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos = 0;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__current_owner = id;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(buf[/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos++]);
  return SUCCESS;
}

# 518 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialP.nc"
static void SerialP__MaybeScheduleTx(void )
#line 518
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 519
    {
      if (SerialP__txPending == 0) {
          if (SerialP__RunTx__postTask() == SUCCESS) {
              SerialP__txPending = 1;
            }
        }
    }
#line 525
    __nesc_atomic_end(__nesc_atomic); }
}

# 73 "/home/ezio/tinyos-main-read-only/tos/lib/timer/VirtualizeTimerC.nc"
static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__fireTimers(uint32_t now)
{
  uint16_t num;

  for (num = 0; num < /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__NUM_TIMERS; num++) 
    {
      /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__Timer_t *timer = &/*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 90
                timer->t0 += timer->dt;
                }
              /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__Timer__fired(num);
              break;
            }
        }
    }
  /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC__1__updateFromTimer__postTask();
}

# 311 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static thread_t *TinyThreadSchedulerP__ThreadScheduler__threadInfo(uint8_t id)
#line 311
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 312
    {
      struct thread *__nesc_temp = 
#line 312
      TinyThreadSchedulerP__ThreadInfo__get(id);

      {
#line 312
        __nesc_atomic_end(__nesc_atomic); 
#line 312
        return __nesc_temp;
      }
    }
#line 314
    __nesc_atomic_end(__nesc_atomic); }
}

# 144 "/home/ezio/tinyos-main-read-only/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

#line 147
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

#line 73
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now)
{
  uint16_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 90
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(num);
              break;
            }
        }
    }
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

#line 159
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, TRUE);
}

# 147 "/home/ezio/tinyos-main-read-only/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
    }
#line 154
    __nesc_atomic_end(__nesc_atomic); }
}

# 54 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadInfoP.nc"
static error_t /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__init(void )
#line 54
{
  /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__thread_info.next_thread = (void *)0;
  /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__thread_info.id = 0;
  /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__thread_info.init_block = (void *)0;
  /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__thread_info.stack_ptr = (stack_ptr_t )&((uint8_t *)/*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__stack)[sizeof /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__stack - sizeof(stack_ptr_t )];
  /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__thread_info.state = TOSTHREAD_STATE_INACTIVE;
  /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__thread_info.mutex_count = 0;
  /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__thread_info.start_ptr = /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__run_thread;
  /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__thread_info.start_arg_ptr = (void *)0;
  /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__thread_info.syscall = (void *)0;
  return SUCCESS;
}

#line 50
static __attribute((noinline)) void /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__run_thread(void *arg)
#line 50
{
  /*TestSineSensorAppC.MainThread.ThreadInfoP*/ThreadInfoP__0__ThreadFunction__signalThreadRun(arg);
}

# 49 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/SystemCallQueueP.nc"
static void SystemCallQueueP__SystemCallQueue__enqueue(syscall_queue_t *q, syscall_t *s)
#line 49
{
  s->next_call = (void *)0;
  SystemCallQueueP__LinkedList__addLast(& q->l, (list_element_t *)s);
}

# 63 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/SystemCallP.nc"
static error_t SystemCallP__SystemCall__start(void *syscall_ptr, syscall_t *s, syscall_id_t id, void *p)
#line 63
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 64
    {

      SystemCallP__current_call = s;
      SystemCallP__current_call->id = id;
      SystemCallP__current_call->thread = SystemCallP__ThreadScheduler__currentThreadInfo();
      SystemCallP__current_call->thread->syscall = s;
      SystemCallP__current_call->params = p;

      if (syscall_ptr != SYSCALL_WAIT_ON_EVENT) {
          SystemCallP__current_call->syscall_ptr = syscall_ptr;
          SystemCallP__threadTask__postTask();
          SystemCallP__ThreadScheduler__wakeupThread(TOSTHREAD_TOS_THREAD_ID);
        }

      {
        unsigned char __nesc_temp = 
#line 78
        SystemCallP__ThreadScheduler__suspendCurrentThread();

        {
#line 78
          __nesc_atomic_end(__nesc_atomic); 
#line 78
          return __nesc_temp;
        }
      }
    }
#line 81
    __nesc_atomic_end(__nesc_atomic); }
}

# 253 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static error_t TinyThreadSchedulerP__ThreadScheduler__suspendCurrentThread(void )
#line 253
{
  /* atomic removed: atomic calls only */
#line 254
  {
    if (TinyThreadSchedulerP__current_thread->state == TOSTHREAD_STATE_ACTIVE) {
        TinyThreadSchedulerP__current_thread->state = TOSTHREAD_STATE_SUSPENDED;
        TinyThreadSchedulerP__suspend(TinyThreadSchedulerP__current_thread);
        {
          unsigned char __nesc_temp = 
#line 258
          SUCCESS;

#line 258
          return __nesc_temp;
        }
      }
#line 260
    {
      unsigned char __nesc_temp = 
#line 260
      FAIL;

#line 260
      return __nesc_temp;
    }
  }
}

#line 97
static void TinyThreadSchedulerP__sleepWhileIdle(void )
#line 97
{
  while (TRUE) {
      bool mt;

      /* atomic removed: atomic calls only */
#line 100
      mt = TinyThreadSchedulerP__ThreadQueue__isEmpty(&TinyThreadSchedulerP__ready_queue) == TRUE;
      if (!mt || TinyThreadSchedulerP__tos_thread->state == TOSTHREAD_STATE_READY) {
#line 101
        break;
        }
#line 102
      TinyThreadSchedulerP__McuSleep__sleep();
    }
}

# 182 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/LinkedListC.nc"
static list_element_t *LinkedListC__LinkedList__remove(linked_list_t *l, list_element_t *e)
#line 182
{
  list_element_t *temp;

#line 184
  if (l->head == (void *)0) {
#line 184
    return (void *)0;
    }
#line 185
  if (l->head == e) {
#line 185
    return LinkedListC__remove_element(l, & l->head);
    }
  temp = LinkedListC__get_element_before(l, e);
  if (temp == (void *)0) {
#line 188
    return (void *)0;
    }
  else {
#line 189
    return LinkedListC__remove_element(l, & temp->next);
    }
}

# 186 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static __attribute((noinline)) __attribute((naked)) void TinyThreadSchedulerP__threadWrapper(void )
#line 186
{
  thread_t *t;

#line 188
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 188
    t = TinyThreadSchedulerP__current_thread;
#line 188
    __nesc_atomic_end(__nesc_atomic); }

  __nesc_enable_interrupt();
  (* t->start_ptr)(t->start_arg_ptr);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 193
    {
      TinyThreadSchedulerP__stop(t);
      TinyThreadSchedulerP__sleepWhileIdle();
      TinyThreadSchedulerP__scheduleNextThread();
      TinyThreadSchedulerP__restoreThread();
    }
#line 198
    __nesc_atomic_end(__nesc_atomic); }
}

# 74 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/ThreadMapP.nc"
static void ThreadMapP__DynamicThreadCleanup__default__cleanup(uint8_t id)
#line 74
{
  ThreadMapP__StaticThreadCleanup__cleanup(id);
}

# 37 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/interfaces/ThreadCleanup.nc"
static void ThreadMapP__DynamicThreadCleanup__cleanup(uint8_t arg_0x7f3447832dc0){
#line 37
    ThreadMapP__DynamicThreadCleanup__default__cleanup(arg_0x7f3447832dc0);
#line 37
}
#line 37
# 87 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static __attribute((noinline)) void TinyThreadSchedulerP__restoreThread(void )
#line 87
{
   __asm ("mov.w %0,r1" :  : "m"(TinyThreadSchedulerP__current_thread->stack_ptr)); __asm ("mov.w %0,r2" :  : "r"(TinyThreadSchedulerP__current_thread->regs.status)); __asm ("mov.w %0,r4" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r4)); __asm ("mov.w %0,r5" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r5)); __asm ("mov.w %0,r6" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r6)); __asm ("mov.w %0,r7" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r7)); __asm ("mov.w %0,r8" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r8)); __asm ("mov.w %0,r9" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r9)); __asm ("mov.w %0,r10" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r10)); __asm ("mov.w %0,r11" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r11)); __asm ("mov.w %0,r12" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r12)); __asm ("mov.w %0,r13" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r13)); __asm ("mov.w %0,r14" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r14)); __asm ("mov.w %0,r15" :  : "m"(TinyThreadSchedulerP__current_thread->regs.r15));}

# 97 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/chips/msp430/HplMsp430Usart1P.nc"
__attribute((wakeup)) __attribute((interrupt(0x0006)))  void sig_UART1RX_VECTOR(void )
#line 97
{
  uint8_t temp = U1RXBUF;

#line 99
  HplMsp430Usart1P__Interrupts__rxDone(temp);
  HplMsp430Usart1P__PlatformInterrupt__postAmble();
}

# 153 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void )
#line 153
{
  /* atomic removed: atomic calls only */
#line 154
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
#line 156
        FALSE;

#line 156
        return __nesc_temp;
      }
  }
#line 158
  return TRUE;
}

# 412 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialP.nc"
static void SerialP__rx_state_machine(bool isDelimeter, uint8_t data)
#line 412
{

  switch (SerialP__rxState) {

      case SerialP__RXSTATE_NOSYNC: 
        if (isDelimeter) {
            SerialP__rxInit();
            SerialP__rxState = SerialP__RXSTATE_PROTO;
          }
      break;

      case SerialP__RXSTATE_PROTO: 
        if (!isDelimeter) {
            SerialP__rxCRC = crcByte(SerialP__rxCRC, data);
            SerialP__rxState = SerialP__RXSTATE_TOKEN;
            SerialP__rxProto = data;
            if (!SerialP__valid_rx_proto(SerialP__rxProto)) {
              goto nosync;
              }
            if (SerialP__rxProto != SERIAL_PROTO_PACKET_ACK) {
                goto nosync;
              }
            if (SerialP__ReceiveBytePacket__startPacket() != SUCCESS) {
                goto nosync;
              }
          }
      break;

      case SerialP__RXSTATE_TOKEN: 
        if (isDelimeter) {
            goto nosync;
          }
        else {
            SerialP__rxSeqno = data;
            SerialP__rxCRC = crcByte(SerialP__rxCRC, SerialP__rxSeqno);
            SerialP__rxState = SerialP__RXSTATE_INFO;
          }
      break;

      case SerialP__RXSTATE_INFO: 
        if (SerialP__rxByteCnt < SerialP__SERIAL_MTU) {
            if (isDelimeter) {
                if (SerialP__rxByteCnt >= 2) {
                    if (SerialP__rx_current_crc() == SerialP__rxCRC) {
                        SerialP__ReceiveBytePacket__endPacket(SUCCESS);
                        SerialP__ack_queue_push(SerialP__rxSeqno);
                        SerialP__rxInit();
                        SerialP__SerialFrameComm__resetReceive();
                        if (SerialP__offPending) {
                            SerialP__rxState = SerialP__RXSTATE_INACTIVE;
                            SerialP__testOff();
                          }
                        goto done;
                      }
                    else {
                        goto nosync;
                      }
                  }
                else {
                    goto nosync;
                  }
              }
            else {
                if (SerialP__rxByteCnt >= 2) {
                    SerialP__ReceiveBytePacket__byteReceived(SerialP__rx_buffer_top());
                    SerialP__rxCRC = crcByte(SerialP__rxCRC, SerialP__rx_buffer_pop());
                  }
                SerialP__rx_buffer_push(data);
                SerialP__rxByteCnt++;
              }
          }
        else 

          {
            goto nosync;
          }
      break;

      default: 
        goto nosync;
    }
  goto done;

  nosync: 

    SerialP__rxInit();
  SerialP__SerialFrameComm__resetReceive();
  SerialP__ReceiveBytePacket__endPacket(FAIL);
  if (SerialP__offPending) {
      SerialP__rxState = SerialP__RXSTATE_INACTIVE;
      SerialP__testOff();
    }
  else {
    if (isDelimeter) {
        SerialP__rxState = SerialP__RXSTATE_PROTO;
      }
    }
  done: ;
}

# 91 "/home/ezio/tinyos-main-read-only/tos/system/crc.h"
static uint16_t crcByte(uint16_t crc, uint8_t b)
#line 91
{
  crc = (uint8_t )(crc >> 8) | (crc << 8);
  crc ^= b;
  crc ^= (uint8_t )(crc & 0xff) >> 4;
  crc ^= crc << 12;
  crc ^= (crc & 0xff) << 5;
  return crc;
}

# 296 "/home/ezio/tinyos-main-read-only/tos/lib/serial/SerialDispatcherP.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(error_t result)
#line 296
{
  uint8_t postsignalreceive = FALSE;

  /* atomic removed: atomic calls only */
#line 298
  {
    if (!/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending && result == SUCCESS) {
        postsignalreceive = TRUE;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending = TRUE;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskType = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskWhich = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskBuf = (message_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskSize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBufferSwap();
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_IDLE;
      }
    else 
#line 308
      {

        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which);
      }
  }
  if (postsignalreceive) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__postTask();
    }
}

# 166 "/home/ezio/tinyos-main-read-only/tos/system/ArbiterP.nc"
static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void )
#line 166
{
  /* atomic removed: atomic calls only */
#line 167
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state != /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 169
        /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;

#line 169
        return __nesc_temp;
      }
#line 170
    {
      unsigned char __nesc_temp = 
#line 170
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId;

#line 170
      return __nesc_temp;
    }
  }
}

# 103 "/home/ezio/tinyos-main-read-only/tos/lib/tosthreads/chips/msp430/HplMsp430Usart1P.nc"
__attribute((wakeup)) __attribute((interrupt(0x0004)))  void sig_UART1TX_VECTOR(void )
#line 103
{
  HplMsp430Usart1P__Interrupts__txDone();
  HplMsp430Usart1P__PlatformInterrupt__postAmble();
}

# 118 "/home/ezio/tinyos-main-read-only/tos/lib/serial/HdlcTranslateC.nc"
static void HdlcTranslateC__UartStream__sendDone(uint8_t *buf, uint16_t len, 
error_t error)
#line 119
{
  /* atomic removed: atomic calls only */
#line 120
  {
    if (HdlcTranslateC__state.sendEscape) {
        HdlcTranslateC__state.sendEscape = 0;
        HdlcTranslateC__m_data = HdlcTranslateC__txTemp;
        HdlcTranslateC__UartStream__send(&HdlcTranslateC__m_data, 1);
      }
    else {
        HdlcTranslateC__SerialFrameComm__putDone();
      }
  }
}

#line 106
static error_t HdlcTranslateC__SerialFrameComm__putData(uint8_t data)
#line 106
{
  if (data == HDLC_CTLESC_BYTE || data == HDLC_FLAG_BYTE) {
      HdlcTranslateC__state.sendEscape = 1;
      HdlcTranslateC__txTemp = data ^ 0x20;
      HdlcTranslateC__m_data = HDLC_CTLESC_BYTE;
    }
  else {
      HdlcTranslateC__m_data = data;
    }
  return HdlcTranslateC__UartStream__send(&HdlcTranslateC__m_data, 1);
}

