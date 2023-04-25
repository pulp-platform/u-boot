#include <init.h>

#define CHESHIRE_AXI_LLC_ADDR 0x03001000

int board_init(void) {
  //volatile unsigned int *axi_llc = (volatile unsigned int *) CHESHIRE_AXI_LLC_ADDR;
  // Configure the LLC fully as cache
  // CFG_SPM_LOW = 0x0
  //axi_llc[0] = 0x0;

  // CFG_SPM_HIGH = 0x0
  //axi_llc[1] = 0x0;

  // COMMIT_CFG = 0x1
  //axi_llc[4] = 0x1;

  return 0;
}

int ft_board_setup(void *fdt, struct bd_info *bd) {
  return 0;
}
