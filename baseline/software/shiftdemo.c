#include <stdio.h>
#include <sys/stat.h> 
#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <assert.h>
#include <string.h>

#define _GNU_SOURCE
#include <sys/types.h>
#include <sys/syscall.h>
#include <sys/mman.h>
#include <linux/perf_event.h>
#include "hps_0.h"
#include "socal/hps.h"

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

static int fddev = -1;
__attribute__((constructor)) static void init(void) {
        static struct perf_event_attr attr;
        attr.type = PERF_TYPE_HARDWARE;
        attr.config = PERF_COUNT_HW_CPU_CYCLES;
        fddev = syscall(__NR_perf_event_open, &attr, 0, -1, -1, 0);
}

__attribute__((destructor)) static void fini(void) {
  close(fddev);
}

static inline long long cpucycles(void) {
  long long result = 0;
  if (read(fddev, &result, sizeof(result)) < sizeof(result)) return 0;
  return result;
}

typedef signed char           sample_t;
typedef volatile signed char *sample_hptr;
typedef unsigned              index_t;

#define DMA_REG_STATUS  0
#define DMA_REG_READ    1
#define DMA_REG_WRITE   2
#define DMA_REG_LENGTH  3
#define DMA_REG_CONTROL 6

#define DMA_STATUS_DONE 0x1
#define DMA_STATUS_BUSY 0x2

#define DMA_CONTROL_BYTE   0x1
#define DMA_CONTROL_WORD   0x3
#define DMA_CONTROL_START  0x8
#define DMA_CONTROL_ENDLEN 0x80
#define DMA_CONTROL_RCONST 0x100
#define DMA_CONTROL_WCONST 0x200
#define DMA_CONTROL_RESET  0x1000

int dmacomplete(volatile unsigned *dmabase) {
  if ((dmabase[DMA_REG_STATUS] & 0x1) == 0x1) {
    // clear the start bit
    dmabase[DMA_REG_CONTROL] &= ~(DMA_CONTROL_START);
    // clear the done bit
    dmabase[DMA_REG_STATUS]  = 0;
    return 1;
  }
  return 0;
}

void dmareset(volatile unsigned *dmabase) {
  dmabase[DMA_REG_CONTROL] = DMA_CONTROL_RESET;
}

void dmacopy(volatile unsigned *dmabase,
	     unsigned source,
	     unsigned destination,
	     unsigned nbytes) {
  // clear done bit
  dmabase[DMA_REG_STATUS]  = 0;
  // program the transaction
  dmabase[DMA_REG_LENGTH]  = nbytes;
  dmabase[DMA_REG_READ]    = source;
  dmabase[DMA_REG_WRITE]   = destination;
  // start the transaction
  dmabase[DMA_REG_CONTROL] =
    DMA_CONTROL_BYTE |
    DMA_CONTROL_ENDLEN |
    DMA_CONTROL_START;
}

void dmacopyconst(volatile unsigned *dmabase,
		  unsigned source,
		  unsigned destination,
		  unsigned nbytes) {
  // clear done bit
  dmabase[DMA_REG_STATUS]  = 0;
  // program the transaction
  dmabase[DMA_REG_LENGTH]  = nbytes;
  dmabase[DMA_REG_READ]    = source;
  dmabase[DMA_REG_WRITE]   = destination;
  // start the transaction
  dmabase[DMA_REG_CONTROL] =
    DMA_CONTROL_BYTE |
    DMA_CONTROL_ENDLEN |
    DMA_CONTROL_WCONST |
    DMA_CONTROL_START;
}

//------------------------------------------------------------------
int shiftreadreg(volatile unsigned *shiftbase, int idx) {
  if ((idx == 8) | (idx == 9) | ((idx >= 0) && (idx <= 3))) 
    return shiftbase[idx];
  else
    return 0xdeaddead;
}

void shiftwrite(volatile unsigned *shiftbase, int v) {
  ((volatile unsigned char *) shiftbase)[0] = v;
}

void shiftclear(volatile unsigned *shiftbase) {
  shiftbase[1] = 1;
}

//------------------------------------------------------------------

// bulk storage in FPGA on AXI bus
sample_hptr mem1;
volatile unsigned *dmabase;
volatile unsigned *shiftbase;
volatile unsigned *ledpio;

void *peripherals;
sample_hptr        onchipmem;

int main() {
  long long csw, ccp, ccpdma;
  
  int memfd;  
  if( ( memfd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
    printf( "ERROR: could not open \"/dev/mem\"...\n" );
    return( 1 );
  }

  peripherals = mmap( NULL,
		      HW_REGS_SPAN,
		      ( PROT_READ | PROT_WRITE ),
		      MAP_SHARED,
		      memfd,
		      HW_REGS_BASE );
  if (peripherals == MAP_FAILED) {
    printf("ERROR: mmap() failed ...\n");
    return 1;
  }

  printf("HW_REGS_BASE %x\n", HW_REGS_BASE);
  printf("ALT_LWFPGASLVS_OFST %x\n", ALT_LWFPGASLVS_OFST);
  
  dmabase = peripherals +
    ( ( unsigned )( ALT_LWFPGASLVS_OFST + DMA_0_BASE) &
      ( unsigned )( HW_REGS_MASK ) );
  ledpio = peripherals +
    ( ( unsigned )( ALT_LWFPGASLVS_OFST + LED_PIO_0_BASE) &
      ( unsigned )( HW_REGS_MASK ) );

  // mem1 points to ONCHIP_MEMORY2_0, 128Kbyte source memory
  mem1 = mmap( NULL,
	       ONCHIP_MEMORY2_0_SPAN,
	       ( PROT_READ | PROT_WRITE ),
	       MAP_SHARED,
	       memfd,
	       0xC0000000 + ONCHIP_MEMORY2_0_BASE);
  if (mem1 == MAP_FAILED) {
    printf("ERROR: mmap() failed ...\n");
    return 1;
  }
  
  printf("SHIFTBLOCKCP_0_BASE %x\n", SHIFTBLOCKCP_0_BASE);
  shiftbase = mmap( NULL,
		    SHIFTBLOCKCP_0_SPAN,
		    ( PROT_READ | PROT_WRITE ),
		    MAP_SHARED,
		    memfd,
		    0xC0000000 + SHIFTBLOCKCP_0_BASE);
  if (shiftbase == MAP_FAILED) {
    printf("ERROR: shiftbase mmap() failed ...\n");
    return 1;
  }

  printf("mem1:    %x\n",   (unsigned) mem1   );
  printf("shiftbase: %x\n", (unsigned) shiftbase);
  printf("dma:     %x\n",   (unsigned) dmabase);
  printf("led:     %x\n",   (unsigned) ledpio );

  unsigned i, j;

  // fill memory with random data
  for (i=0; i<1024*128; i++) {
    mem1[i] = (i + 1) % 128; // rand() % 0xff;
  }

  unsigned sumsw[1024];
  unsigned sumcp[1024];
  unsigned sumcpdma[1024];

  //-----------------------------------------------------------
  // compute max-peak in software
  int sum, max, s;
  csw = cpucycles();
  for (j=0; j<1024; j++) {
    
    max = 0;
    for (i=0; i<128; i++) {
      sum = 0;
      for (s = 0; s<16; s++) 
	sum += (unsigned char) mem1[j*128 + ((i + s) % 128)];
      if (sum > max)
	max = sum;
    }
    
    sumsw[j] = max;
  }
  csw = cpucycles() - csw;

  //-----------------------------------------------------------
  // compute max-peak using coprocessor
  ccp = cpucycles();
  for (j=0; j<1024; j++) {
    
    // clear maximum
    shiftclear(shiftbase);
    
    // load data
    for (i=0; i<128; i++) 
      shiftwrite(shiftbase, mem1[j*128 + i]);

    // retrieve max two times
    // small delay, to let coprocessor finish
    sumcp[j] = shiftreadreg(shiftbase, 8);
    sumcp[j] = shiftreadreg(shiftbase, 8);
  }
  ccp = cpucycles() - ccp;

  //-----------------------------------------------------------
  // compute max-peak using coprocessor and dma
  ccpdma = cpucycles();
  for (j=0; j<1024; j++) {
    
    // clear maximum
    shiftclear(shiftbase);

    // load data
    dmareset(dmabase);
    dmacopyconst(dmabase,
		 DMA_0_READ_MASTER_ONCHIP_MEMORY2_0_BASE + (j*128),
		 DMA_0_WRITE_MASTER_SHIFTBLOCKCP_0_BASE,
		 128
		 );
    while (! dmacomplete(dmabase)) ;

    // retrieve maximum
    sumcpdma[j] = shiftreadreg(shiftbase, 8);
  }
  ccpdma = cpucycles() - ccpdma;

  //-----------------------------------------------------------
  // compare result of three cases
  printf("Software Cycles        %lld\n", csw);
  printf("Coprocessor Cycles     %lld\n", ccp);
  printf("Coprocessor+DMA Cycles %lld\n", ccpdma);

  for (j=0; j<1024; j++) {
    if (sumsw[j] != sumcp[j])
      printf("Error index %d sw %x coproc %x\n", j, sumsw[j], sumcp[j]);
    if (sumsw[j] != sumcpdma[j])
      printf("Error index %d sw %x dma %x\n", j, sumsw[j], sumcpdma[j]);    
  }
  
  //-----------------------------------------------------------
  // verbose example of coprocessor operation
  unsigned verbose = 0;
  if (verbose) {
    // clear maximum
    shiftclear(shiftbase);
    
    // load data
    for (i=0; i<128; i++) {
      printf("%8x v %8x max %8x taps %8x %8x %8x %8x\n", i,
	     shiftreadreg(shiftbase, 9),
	     shiftreadreg(shiftbase, 8),
	     shiftreadreg(shiftbase, 0),
	     shiftreadreg(shiftbase, 1),
	     shiftreadreg(shiftbase, 2),
	     shiftreadreg(shiftbase, 3));
      
      shiftwrite(shiftbase, mem1[i]);
    }
    
    // retrieve maximum
    printf("%8x max %8x\n", j, shiftreadreg(shiftbase, 8));
  }
   

  close(memfd);
  
  return 0;
}
