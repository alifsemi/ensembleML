/* Copyright (c) 2021 ALIF SEMICONDUCTOR

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ALIF SEMICONDUCTOR nor the names of its contributors
     may be used to endorse or promote products derived from this software
     without specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/

/**************************************************************************//**
 * @file     i3c_camera_testApp.c
 * @author   Tanay Rami
 * @email    tanay@alifsemi.com
 * @version  V1.0.0
 * @date     30-April-2021
 * @brief    TestApp to verify i2c over i3c using Threadx as an operating system.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

/* Includes ----------------------------------------------------------------- */
/* System Includes */
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Project Includes */
#include "delay.h"
#include "Driver_PINMUX_AND_PINPAD.h"

#include "image_processing.h"
extern uint8_t lcd_image[DIMAGE_Y][DIMAGE_X][RGB_BYTES];

//camera irq
int IRQn = 336;

//#define MIPI_PRINTF					printf
void MIPI_PRINTF(const char *fmt,...)	{ sleep_or_wait_usec(200); }

#define CAMERA_BASE_ADDR			0x49030000
#define expslv1_base_addr  			0x4903F000
#define cdc_base_addr      			0x49031000
#define mipi_dsi_base_addr 			0x49032000
#define mipi_csi_base_addr 			0x49033000


//define VERSION ADDRESS REGISTER
#define     A_VERSION_CFG                       0x00
//define CLOCK MANAGER ADDRESS REGISTERS
#define     A_PWR_UP                            0x04
#define     A_CLKMGR_CFG                        0x08
//define DPI ADDRESS REGISTERS
#define     A_DPI_VCID                          0x0C
#define     A_DPI_COLOR_CODING                  0x10
#define     A_DPI_CFG_POL                       0x14
#define     A_DPI_LP_CMD_TIM                    0x18
#define     A_DPI_DUAL_PIXEL_MODE               0x14
//define DBI ADDRESS REGISTERS
#define     A_DBI_VCID                          0x1C
#define     A_DBI_CFG                           0x20
#define     A_DBI_PARTITIONING_EN               0x24
#define     A_DBI_CMDSIZE                       0x28
//define PACKET HANDLER ADDRESS REGISTERS
#define     A_PCKHDL_CFG                        0x2C
#define     A_GEN_VCID                          0x30
#define     A_MODE_CFG                          0x34
//define VIDEO MODE ADDRESS REGISTERS
#define     A_VID_MODE_CFG                      0x38
#define     A_VID_PKT_SIZE                      0x3C
#define     A_VID_NUM_CHUNKS                    0x40
#define     A_VID_NULL_SIZE                     0x44
#define     A_VID_HSA_TIME                      0x48
#define     A_VID_HBP_TIME                      0x4C
#define     A_VID_HLINE_TIME                    0x50
#define     A_VID_VSA_LINE                      0x54
#define     A_VID_VBP_LINES                     0x58
#define     A_VID_VFP_LINES                     0x5C
#define     A_VID_VACTIVE_LINES                 0x60
#define     A_EDPI_CMD_SIZE                     0x64
#define     A_CMD_MODE_CFG                      0x68
//define SFR2GENERIC ADDRESS REGISTERS
#define     A_GEN_HDR                           0x6C
#define     A_GEN_PLD_DATA                      0x70
#define     A_CMD_PKT_STATUS                    0x74
#define     A_TO_CNT_CFG                        0x78
#define     A_HS_RD_TO_CNT                      0x7C
#define     A_LP_RD_TO_CNT                      0x80
#define     A_HS_WR_TO_CNT                      0x84
#define     A_LP_WR_TO_CNT                      0x88
#define     A_BTA_TO_CNT                        0x8C
#define     A_SDF_3D                            0x90
#define     A_LPCLK_CTRL                        0x94
//define PHYIFCTRLDIG ADDRESS REGISTERS
#define     A_PHY_TMR_LPCLK_CFG                 0x98
#define     A_PHY_TMR_CFG                       0x9C
#define     A_PHY_TMR_RD_CFG                    0xF4
#define     A_PHYRSTZ                           0xA0
#define     A_PHY_IF_CFG                        0xA4
#define     A_PHY_ULPS_CTRL                     0xA8
#define     A_PHY_TX_TRIGGERS                   0xAC
#define     A_PHY_STATUS                        0xB0
#define     A_PHY_TST_CTRL0                     0xB4
#define     A_PHY_TST_CTRL1                     0xB8
//define ERROR CONTROL ADDRESS REGISTERS
#define     A_INT_ST0                           0xBC
#define     A_INT_ST1                           0xC0
#define     A_INT_MASK0_CFG                     0xC4
#define     A_INT_MASK1_CFG                     0xC8
#define     A_PHY_CAL                           0xCC
#define     A_VID_SHADOW_CTRL                   0x100

#define     A_DPI_VCID_ACT                      0x10C
#define     A_DPI_COLOR_CODING_ACT              0x110
#define     A_DPI_LP_CMD_TIM_ACT                0x118
#define     A_EDPI_TE_HW_CFG                    0x11C
#define     A_VID_MODE_CFG_ACT                  0x138
#define     A_VID_PKT_SIZE_ACT                  0x13C
#define     A_VID_NUM_CHUNKS_ACT                0x140
#define     A_VID_NULL_SIZE_ACT                 0x144
#define     A_VID_HSA_TIME_ACT                  0x148
#define     A_VID_HBP_TIME_ACT                  0x14C
#define     A_VID_HLINE_TIME_ACT                0x150
#define     A_VID_VSA_LINES_ACT                 0x154
#define     A_VID_VBP_LINES_ACT                 0x158
#define     A_VID_VFP_LINES_ACT                 0x15C
#define     A_VID_VACTIVE_LINES_ACT             0x160
#define     A_VID_PKT_STATUS                    0x168
#define     A_SDF_3D_CFG_ACT                    0x190

#define     A_INT_FORCE0                        0xD8
#define     A_INT_FORCE1                        0xDC

#define     A_AUTO_ULPS_MODE                    0xE0
#define     A_AUTO_ULPS_ENTRY_DELAY             0xE4
#define     A_AUTO_ULPS_WAKEUP_TIME             0xE8
#define     A_EDPI_ADV_FEATURES                 0xEC
#define     A_AUTO_ULPS_MIN_TIME                0xF8

#define     A_DSC_PARAMETER                     0xF0

//////////////////////////////////////


#define HW_REG_WORD(base,offset) *((volatile unsigned int *) (base + offset))
#define HW_REG_BYTE(base,offset) *((volatile unsigned char *) (base + offset))


/* Define main entry point.  */
static void CSI_DPHY_REG_WRITE (int,uint8_t);
static void DSI_DPHY_REG_WRITE (int,uint8_t);
static uint8_t CSI_DPHY_REG_READ (int);
static uint8_t DSI_DPHY_REG_READ (int);
#define expslv1_base_addr  0x4903F000
#define mipi_csi_base_addr 0x49033000
#define mipi_dsi_base_addr 0x49032000

//Display
static void DCSW1_S (uint8_t, uint8_t); //cmd, data
static void DCSWN_S (uint8_t); //cmd
static void DCSW_L (uint8_t, uint8_t, uint8_t, uint8_t, uint8_t); //cmd, data1, data2, data3, data4
static void SMRPS_S (uint8_t);
static void DCSRR1_S (uint8_t);

//#define FREQ_50M
//#define FREQ_125M
#define FREQ_250M

void hw_disp_init()
{
	  MIPI_PRINTF("Hello HP-YMN\n");

	  MIPI_PRINTF("CSI DPHY setup\n");

	  int rd_data =0,exp_data =0,err=0;
	  uint8_t r_data_8;

#ifdef FREQ_50M
  int hsfreqrange = 0x30;
  int pll_soc_m_7_0 = 0x4d;
  int pll_soc_m_9_8 = 0x1;
  int pll_soc_n = 0x2;
  uint8_t vco_cntrl = 0x28; //101000
  int below_450Mbps = 1;
#elif defined(FREQ_125M) //actually it is 185M in measurement
  int hsfreqrange = 0x33;
  int pll_soc_m_7_0 = 0x71;
  int pll_soc_m_9_8 = 0x2;
  int pll_soc_n = 0x3;
  uint8_t vco_cntrl = 0x1F; //101000
  int below_450Mbps = 1;
#elif defined(FREQ_150M)
  int hsfreqrange = 0x14;
  int pll_soc_m_7_0 = 0x77;
  int pll_soc_m_9_8 = 0x1;
  int pll_soc_n = 0x2;
  uint8_t vco_cntrl = 0x1B; //011011
  int below_450Mbps = 1;
#elif defined(FREQ_250M)
  int hsfreqrange = 0x33;
  int pll_soc_m_7_0 = 0x71;
  int pll_soc_m_9_8 = 0x2;
  int pll_soc_n = 0x2;
  uint8_t vco_cntrl = 0x18; //101000
  int below_450Mbps = 1;
#elif defined(FREQ_300M) //actually it is 185M in measurement
  int hsfreqrange = 0x14;
  int pll_soc_m_7_0 = 0x77;
  int pll_soc_m_9_8 = 0x1;
  int pll_soc_n = 0x2;
  uint8_t vco_cntrl = 0x13; //101000
  int below_450Mbps = 1;
#elif defined(FREQ_400M)
  int hsfreqrange = 0x9;
  int pll_soc_m_7_0 = 0xF4;
  int pll_soc_m_9_8 = 0x1;
  int pll_soc_n = 0x2;
  uint8_t vco_cntrl = 0x10; //010000
  int below_450Mbps = 1;
#else //1G
  int hsfreqrange = 0xA;
  int pll_soc_m_7_0 = 0xA1;
  int pll_soc_m_9_8 = 0x1;
  int pll_soc_n = 0x3;
  uint8_t vco_cntrl = 0xF; //001111
  int below_450Mbps = 0;
#endif

  // video format
  uint16_t HSYNC = 0x40; //64
  uint16_t VSYNC = 0xA;  //10
  uint16_t HBP   = 0x5E; //94
  uint16_t VBP   = 0x14; //20
  uint16_t HACT  = 0x1E0; //480
  //uint16_t VACT  = 0x320; //800
  uint16_t VACT  = 0x340; //800
  //uint16_t VACT  = 0x50; //80
  uint16_t HFP   = 0x48B; //adjust this to sync up DSI host controller
  uint16_t VFP   = 0x14; //20

  uint32_t *p, i;
  for (i = 0, p = (uint32_t *) &lcd_image; i < (HACT*VACT*3/4); i++, p++)
	  *p = 0;

      MIPI_PRINTF("MIPIDSI Display TEST \n");

	  //turn on Dig 38.4MHz clock -jira SE-694
	  //HW_REG_WORD(0x7100a100, 0x00) = 0x00010A29;
      //HW_REG_WORD(0x7100a100, 0x00) = 0x00108229;

      //use default 32Khz clock to program the following registers
      //HW_REG_WORD(0x7100a098, 0x00)= 0x0;


      rd_data = HW_REG_WORD(0x7100a100,0x00);
	  MIPI_PRINTF("0x7100a100 0x%0x \n", rd_data);

	  HW_REG_WORD(0x7100a100, 0x00)=rd_data | 0x1<<5;
	  rd_data = HW_REG_WORD(0x7100a100,0x00);

	  MIPI_PRINTF("0x7100a100 0x%0x \n", rd_data);

	  //configure source to crystal
	  HW_REG_WORD(0x71007410,0x00) = 0;
	  HW_REG_WORD(0x71007410,0x00) = 0x00000002;

	  //enable phy power
	  HW_REG_WORD(0x7004001C,0x00) |= (1U << 10);

	  ////////////////////////////////////////////////////////////////
	  //DSI DPHY setup as master
	  ////////////////////////////////////////////////////////////////

	  //cdc200_pixclk_ctrl.clk_ena = 0x0
	  //cdc200_pixclk_ctrl.clk_divisor = 0x1ff
	  HW_REG_WORD(expslv1_base_addr,0x04) =  0x00060001;//set cdc200 pix clock//sub system 400/6=66.7 Mhz

	  //number of lines
	  HW_REG_WORD((mipi_dsi_base_addr),0xa4)=0x00000001;

	  MIPI_PRINTF("MIPI DSI DPHY-Write task started....\n");

	  //Startup to Master mode
	  //[1] rstz; [0]shutdown
	  //1. Set rstz = 1'b0;
	  //2. Set shutdownz= 1'b0;
	  HW_REG_WORD(mipi_dsi_base_addr,0x00a0) = 0x00000000;//shutdownz=0,rstz=0

	  //testport_sel [4]	RW	0x0	testport select; 0: select tx_testport; 1: select rx_testport	( rx_dphy_ctrl0.testport_sel = 0x0 )
	  //txrxz [8]	RW	0x0	control tx_rxz pin of DPHY	( rx_dphy_ctrl0.txrxz = 0x0 )
	  //Master/Slave Configuration Signal.
	  //This signal controls whether the PHY functionalities are configured
	  //for a Master-side D-PHY implementation (tx_rxz=1) or for a Slaveside
	  //D- PHY implementation (tx_rxz=0).
	  //Note: This signal configures all the PHY lanes (data and clock).
	  //Synchronous To: Asynchronous
	  //Active State: High
	  //3. set tx_rxz=1'b1;
	  HW_REG_WORD(expslv1_base_addr,0x30) =  0x00000100;

	  //4. Set rx_testclr and tx_testclr = 1'b1;
	  //4.1 set tx_testclr
	  //Just toggle tx_testclr
	  HW_REG_WORD(mipi_dsi_base_addr,0xb4) = 0x00000000;//test_clr=0
	  HW_REG_WORD(mipi_dsi_base_addr,0xb4) = 0x00000001;//test_clr=1
	  HW_REG_WORD(mipi_dsi_base_addr,0xb4) = 0x00000000;//test_clr=0

	  //testport_sel [4]	RW	0x0	testport select; 0: select tx_testport; 1: select rx_testport	( rx_dphy_ctrl0.testport_sel = 0x0 )
	  HW_REG_WORD(expslv1_base_addr,0x30) =  0x00000110;//set csi pix clock
	  //4.2 set rx_test_clr
	  //Just toggle rx_testclr
	  HW_REG_WORD(mipi_dsi_base_addr,0xb4) = 0x00000000;//test_clr=0
	  HW_REG_WORD(mipi_dsi_base_addr,0xb4) = 0x00000001;//test_clr=1
	  HW_REG_WORD(mipi_dsi_base_addr,0xb4) = 0x00000000;//test_clr=0

	  //5. Wait for 15 ns;
	  //for(int i=0;i<5;i++);

	  //switch back to tx_testport
	  HW_REG_WORD(expslv1_base_addr,0x30) =  0x00000100;

	  //check DSI DPHY state
	  //dphy4txtester_DIG_RD_TX_SYS_0
	  r_data_8 = DSI_DPHY_REG_READ(0x1E);
	  MIPI_PRINTF("MIPI DSI DPHY Read 1E 0x%0x \n", r_data_8);

	  //7. Set hsfreqrange[6:0] = 7'b0001010;
	  //[22:16] - 1G = hsfreqrange[6:0] = 0x0A
	  //[31:24] - cfgclkfreqrange[7:0] = round[(Fcfg_clk(MHz)-17)*4] = (25-17)*4 = 8'b001000000
	  //cfgclk = tb_top.DUT.u_sse700.u_sse700_f0_expslv1.mipi_tx_dphy_cfg_clk = 25MHz

	  //7.1 TX
	  rd_data = HW_REG_WORD((expslv1_base_addr),0x30);
	  MIPI_PRINTF("EXPSLV1 0x30 0x%0x \n", rd_data);
	  HW_REG_WORD((expslv1_base_addr),0x30)=0x20000100|(hsfreqrange<<16);
	  rd_data = HW_REG_WORD((expslv1_base_addr),0x30);
	  MIPI_PRINTF("EXPSLV1 0x30 0x%0x \n", rd_data);

	  //check hsfreqrange[6:0] in DPHY
	  //dphy4rxtester_DIG_RD_RX_SYS_1
	  r_data_8 = DSI_DPHY_REG_READ(0x1F);
	  MIPI_PRINTF("MIPI DSI DPHY Read 1F 0x%0x \n", r_data_8);

	  //8. Configure TX register 0x16A to set pll_mpll_prog_rw (bits1:0) to 2'b11.
	  //Before D-PHY power, set TX register 0x16A bits1:0 to 2'b11 (pll_mpll_prog_rw).
	  r_data_8 = DSI_DPHY_REG_READ(0x16A);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x16A 0x%0x \n", r_data_8);

	  DSI_DPHY_REG_WRITE(0x16A,0x03);

	  r_data_8 = DSI_DPHY_REG_READ(0x16A);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x16A 0x%0x \n", r_data_8);

	  //9. Configure TX register 0x1AB to set cb_sel_vref_lprx_rw (bits 1:0) to 2'b10.
	  r_data_8 = DSI_DPHY_REG_READ(0x1AB);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x1AB 0x%0x \n", r_data_8);

	  DSI_DPHY_REG_WRITE(0x1AB,0x06);

	  r_data_8 = DSI_DPHY_REG_READ(0x1AB);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x1AB 0x%0x \n", r_data_8);

	  //10. Configure TX register 0x1AA to set cb_sel_vrefcd_lprx_rw (bits 6:5) to 2'b10.
	  r_data_8 = DSI_DPHY_REG_READ(0x1AA);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x1AA 0x%0x \n", r_data_8);

	  DSI_DPHY_REG_WRITE(0x1AA,0x53);

	  r_data_8 = DSI_DPHY_REG_READ(0x1AA);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x1AA 0x%0x \n", r_data_8);

	  //When operating as master or when in High-Speed BIST modes,
	  //for datarates below 450 Mbps, clkdiv_clk_en must be enabled.
	  //To do this, write 1'b1 in TX configuration register with address 0x1AC bit [4].
	  r_data_8 = DSI_DPHY_REG_READ(0x1AC);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x1AC 0x%0x \n", r_data_8);

	  DSI_DPHY_REG_WRITE(0x1AC,(r_data_8|below_450Mbps<<4));

	  r_data_8 = DSI_DPHY_REG_READ(0x1AC);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x1AC 0x%0x \n", r_data_8);

	  //11. Configure TX register 0x402 to set txclk_term_lowcap_lp00_en_ovr_en_rw and
	  //txclk_term_lowcap_lp00_en_ovr_rw(bits 1:0) to 2'b10;
	  r_data_8 = DSI_DPHY_REG_READ(0x402);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x402 0x%0x \n", r_data_8);

	  DSI_DPHY_REG_WRITE(0x402,0x2);

	  r_data_8 = DSI_DPHY_REG_READ(0x402);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x402 0x%0x \n", r_data_8);

	  //12. Refer to table ??Supported rise/fall time limits?? on page 114 and configure TX test control registers
	  //with appropriate values for the specified rise/fall time.
	  //500 Mbps and ?? 1 Gbps 1 ns 150 ps - 300 ps 1 12??d657 225 - 291
	  //<< 500 Mbps 2 ns 150 ps - 300 ps 1 12??d657 225
	  //dphy4txtester_DIG_RDWR_TX_SLEW_5
	  r_data_8 = DSI_DPHY_REG_READ(0x270);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x270 0x%0x \n", r_data_8);

	  DSI_DPHY_REG_WRITE(0x270,0x91);

	  r_data_8 = DSI_DPHY_REG_READ(0x270);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x270 0x%0x \n", r_data_8);

	  //dphy4txtester_DIG_RDWR_TX_SLEW_6
	  r_data_8 = DSI_DPHY_REG_READ(0x271);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x271 0x%0x \n", r_data_8);

	  DSI_DPHY_REG_WRITE(0x271,0x2);

	  r_data_8 = DSI_DPHY_REG_READ(0x271);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x271 0x%0x \n", r_data_8);

	  //dphy4txtester_DIG_RDWR_TX_SLEW_7
	  //13. Set bits [5:4] of TX control register with address 0x272 to 2'b01 to enable slew rate calibration. (Lanes
	  //that are disabled should not run slew-rate calibration algorithm. For the disabled lanes, set
	  //sr_finished_ovr_en = 1??b1, sr_finished_ovr = 1??b1, srcal_en_ovr_en = 1??b1 through test control
	  //registers 0x310, 0x50b, 0x70b);
	  r_data_8 = DSI_DPHY_REG_READ(0x272);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x272 0x%0x \n", r_data_8);

	  DSI_DPHY_REG_WRITE(0x272,0x11);

	  r_data_8 = DSI_DPHY_REG_READ(0x272);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x272 0x%0x \n", r_data_8);

	  //For the disabled lanes, set
	  //sr_finished_ovr_en = 1??b1, sr_finished_ovr = 1??b1, srcal_en_ovr_en = 1??b1 through test control registers 0x310, 0x50b, 0x70b, 0x90b, 0xbob);
	  //bit2, bit1, bit0 are given as "reserved" in the Databook; but program as per the requirement.
	  //Since you are not using lane2, lane3 - you can disable those bits in the registers for lane2 and lane3.

	  //dphy4txtester_DIG_RDWR_TX_LANE2_SLEWRATE_0
	  //lane 2
	  r_data_8 = DSI_DPHY_REG_READ(0x90b);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x90b 0x%0x \n", r_data_8);

	  DSI_DPHY_REG_WRITE(0x90b,0x0e);

	  r_data_8 = DSI_DPHY_REG_READ(0x90b);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x90b 0x%0x \n", r_data_8);

	  //lane 3
	  r_data_8 = DSI_DPHY_REG_READ(0xb0b);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0xb0b 0x%0x \n", r_data_8);

	  DSI_DPHY_REG_WRITE(0xb0b,0x0e);

	  r_data_8 = DSI_DPHY_REG_READ(0xb0b);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0xb0b 0x%0x \n", r_data_8);

	  //14. Set cfgclkfreqrange[7:0] = round[(Fcfg_clk(MHz)-17)*4] = 8'b00101000;
	  //already set
	  //15. Apply cfg_clk signal with the appropriate frequency with 25 Mhz frequency;
	  //16. Configure PLL operating frequency through D-PHY test control registers or through PLL SoC
	  MIPI_PRINTF("MIPI PLL task started....\n");

	  //pll_soc_clksel [21:20]	RW	0x0	clkext div selection - clksel
	  //should be set to 2'b01
	  rd_data = HW_REG_WORD(expslv1_base_addr,0x10);
	  rd_data |= 1UL << 20;
	  HW_REG_WORD(expslv1_base_addr,0x10) = rd_data;

	  //When the PHY is configured as a master (tx_rxz=1'b1) the PLL needs to always be properly configured for
	  //the desired operating frequency before D-PHY Start-up.
	  //Use pll shadow control
	  //pll_soc_shadow_control [4]	RW	0x0	Selection of PLL configuration mechanism	( dphy_pll_ctr
	  //rd_data = HW_REG_WORD(expslv1_base_addr,0x10);
	  //rd_data = rd_data | 1 << 4;
	  //HW_REG_WORD(expslv1_base_addr,0x10) = rd_data;

	  //pll soc shadow control set
	  rd_data = HW_REG_WORD(expslv1_base_addr,0x10);
	  rd_data |= 1UL << 4;
	  HW_REG_WORD(expslv1_base_addr,0x10) = rd_data;

	  //m[11:0] - pll_m_ovr_rw[7:0], pll_m_ovr_rw[11:8], pll_m_ovr_en_rw ?? 0x179, 0x17a, 0x17b
	  //n[3:0] - pll_n_ovr_rw[3:0], pll_n_ovr_en_rw ?? 0x178
	  //Foutmax [MHz] Foutmin [MHz] F VCO max [MHz] F VCO min [MHz] Output division factor P vco_cntrl[5:3]
	  //500           250           4000            2000            8                        010
	  //1000          500           4000            2000            4                        001
	  //62.5          40            4000            2000            64                       101
	  //24Mhz > fclkin/N > 8 Mhz
	  //24Mhz > 38.4Mhz/N > 8 Mhz
	  //N = 4
	  //n = 4 - 1

	  //fout = 40Mhz for 80M bit rate
	  //M/N*1/2*fclkin*1/P = M/3*1/2*38.4Mhz*1/64 = 40Mhz = 400 = 0x190
	  //M = 400 = 0x190

	  //MIPI-DPHY PLL Control Register 1
	  //pll_soc_m [9:0]	RW	0x0	Control of the feedback multiplication ratio M (40 to 625) for SoC direct PLL control	( dphy_pll_ctrl1.pll_soc_m = 0x0 )
	  //pll_soc_n [15:12]	RW	0x0	Control of the input frequency division ratio N (1 to 16) for SoC direct PLL control	( dphy_pll_ctrl1.pll_soc_n = 0x0 )
	  //HW_REG_WORD(expslv1_base_addr,0x14) = 0x031A1;

	  //dphy4txtester_DIG_RDWR_TX_PLL_28
	  //7:0 pll_m_ovr_rw__7__0__ R/W Description: PLL feedback divider override
	  r_data_8 = DSI_DPHY_REG_READ(0x179);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x179 0x%0x \n", r_data_8);

	  DSI_DPHY_REG_WRITE(0x179,pll_soc_m_7_0);

	  r_data_8 = DSI_DPHY_REG_READ(0x179);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x179 0x%0x \n", r_data_8);

	  //dphy4txtester_DIG_RDWR_TX_PLL_29
	  //1:0 pll_m_ovr_rw__9__8__ R/W Description: PLL feedback divider override
	  r_data_8 = DSI_DPHY_REG_READ(0x17a);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x17a 0x%0x \n", r_data_8);

	  DSI_DPHY_REG_WRITE(0x17a,pll_soc_m_9_8);

	  r_data_8 = DSI_DPHY_REG_READ(0x17a);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x17a 0x%0x \n", r_data_8);

	  //dphy4txtester_DIG_RDWR_TX_PLL_30
	  //0 pll_m_ovr_en_rw R/W Description: PLL feedback divider override enable
	  r_data_8 = DSI_DPHY_REG_READ(0x17b);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x17b 0x%0x \n", r_data_8);

	  DSI_DPHY_REG_WRITE(0x17b,0x1);

	  r_data_8 = DSI_DPHY_REG_READ(0x17b);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x17b 0x%0x \n", r_data_8);

	  //dphy4txtester_DIG_RDWR_TX_PLL_27
	  //7 pll_n_ovr_en_rw R/W Description: PLL input divider override enable
	  //6:3 pll_n_ovr_rw__3__0__ R/W Description: PLL input divider override
	  r_data_8 = DSI_DPHY_REG_READ(0x178);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x178 0x%0x \n", r_data_8);

	  DSI_DPHY_REG_WRITE(0x178,(0x80|(pll_soc_n<<3)));

	  r_data_8 = DSI_DPHY_REG_READ(0x178);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x178 0x%0x \n", r_data_8);


	  // vco_cntrl[5:0] - pll_vco_cntrl_ovr_rw[5:0], pll_vco_cntrl_ovr_en_rw ?? 0x17b
	  //dphy4txtester_DIG_RDWR_TX_PLL_30
	  //7 pll_vco_cntrl_ovr_en_rw R/W Description: PLL VCO control override enable
	  //6:1 pll_vco_cntrl_ovr_rw__5__0__ R/W Description: PLL VCO control override
	  //0 pll_m_ovr_en_rw R/W Description: PLL feedback divider override enable
	  //Table 3-6
	  r_data_8 = DSI_DPHY_REG_READ(0x17b);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x17b 0x%0x \n", r_data_8);

	  DSI_DPHY_REG_WRITE(0x17b,(0x81|vco_cntrl<<1));

	  r_data_8 = DSI_DPHY_REG_READ(0x17b);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x17b 0x%0x \n", r_data_8);

	  // cpbias_cntrl[6:0] - pll_cpbias_cntrl_rw[6:0] ?? 0x15e
	  // dphy4txtester_DIG_RDWR_TX_PLL_1
	  // 6:0 pll_cpbias_cntrl_rw__6__0__ R/W Description: PLL Charge Pump Bias Control
	  r_data_8 = DSI_DPHY_REG_READ(0x15e);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x15e 0x%0x \n", r_data_8);

	  DSI_DPHY_REG_WRITE(0x15e,0x0);

	  r_data_8 = DSI_DPHY_REG_READ(0x15e);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x15e 0x%0x \n", r_data_8);

	  // gmp_cntrl[1:0] - pll_gmp_cntrl_rw[1:0] ?? 0x162
	  // int_cntrl[5:0] - pll_int_cntrl_rw[5:0] ?? 0x162
	  // dphy4txtester_DIG_RDWR_TX_PLL_5
	  // 7:2 pll_int_cntrl_rw__5__0__ R/W Description: PLL Integral Charge Pump control
	  // 1:0 pll_gmp_cntrl_rw__1__0__ R/W Description: PLL GMP Control
	  r_data_8 = DSI_DPHY_REG_READ(0x162);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x162 0x%0x \n", r_data_8);

	  DSI_DPHY_REG_WRITE(0x162,0x11);

	  r_data_8 = DSI_DPHY_REG_READ(0x162);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x162 0x%0x \n", r_data_8);

	  // prop_cntrl[5:0] - pll_prop_cntrl_rw[5:0] ?? 0x16e
	  // dphy4txtester_DIG_RDWR_TX_PLL_17
	  // 5:0 pll_prop_cntrl_rw__5__0__ R/W Description: PLL Proportional Charge Pump control
	  r_data_8 = DSI_DPHY_REG_READ(0x16e);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x16e 0x%0x \n", r_data_8);

	  DSI_DPHY_REG_WRITE(0x16e,0x10);

	  r_data_8 = DSI_DPHY_REG_READ(0x16e);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x16e 0x%0x \n", r_data_8);

	  //Output frequency [MHz] vco_cntrl [5:0] cpbias_cntrl [6:0] gmp_cntrl [1:0] int_cntrl[5:0] prop_cntrl[5:0] tx or rx_cb_vref_mpll_re g_rel_rw[2:0]
	  //487.5-615              001111          0000000            01              000100         010000          010
	  //40-46.44               101011          0000000            01              000100         010000          010
	  //pll_soc_cpbias_cntrl [6:0]	RW	0x0	Charge Pump bias control, for SoC direct PLL control	( dphy_pll_ctrl2.pll_soc_cpbias_cntrl = 0x0 )
	  //pll_soc_int_cntrl [13:8]	RW	0x0	Integral Charge Pump control for SoC direct PLL control	( dphy_pll_ctrl2.pll_soc_int_cntrl = 0x0 )
	  //pll_soc_prop_cntrl [21:16]	RW	0x0	Proportional Charge Pump control for SoC direct PLL control	( dphy_pll_ctrl2.pll_soc_prop_cntrl = 0x0 )
	  //pll_soc_vco_cntrl [29:24]	RW	0x0	VCO operating range for SoC direct PLL control	( dphy_pll_ctrl2.pll_soc_vco_cntrl = 0x0 )
	  //HW_REG_WORD(expslv1_base_addr,0x18) = 0x0f100400;

	  //pll_soc_gmp_cntrl [17:16]	RW	0x0	Controls the effective loop-filter resistance (=1/gmp) to increase/decrease MPLL bandwidth
	  //rd_data = HW_REG_WORD(expslv1_base_addr,0x10);
	  //rd_data |= 1UL << 16;
	  //HW_REG_WORD(expslv1_base_addr,0x10) = rd_data;

	  //SNPS: dphy4txtester_DIG_RDWR_TX_CB_3
	  //2:0 cb_vref_mpll_reg_sel_rw__2__0__R/W Description: PLL reference voltage control
	  r_data_8 = DSI_DPHY_REG_READ(0x1AD);
	  MIPI_PRINTF("MIPI DSI DPHY Read 1AD 0x%0x \n", r_data_8);

	  DSI_DPHY_REG_WRITE(0x1AD,0x02);

	  r_data_8 = DSI_DPHY_REG_READ(0x1AD);
	  MIPI_PRINTF("MIPI DSI DPHY Read 1AD 0x%0x \n", r_data_8);

	  ////update PLL pulse
	  ////pll_soc_updatepll [8]	RW	0x0	Control for PLL operation frequency updated
	  //rd_data = HW_REG_WORD(expslv1_base_addr,0x10);
	  //rd_data |= 1UL << 8;
	  //HW_REG_WORD(expslv1_base_addr,0x10) = rd_data;

	  ////Wait 10 ns;
	  //for(int i=0;i<10;i++);
	  //
	  ////pll_soc_updatepll [8]	RW	0x0	Control for PLL operation frequency updated
	  //rd_data = HW_REG_WORD(expslv1_base_addr,0x10);
	  //rd_data &= ~(1UL << 8);
	  //HW_REG_WORD(expslv1_base_addr,0x10) = rd_data;


	  //Wait for PLL lock
	  //do {
	  //  rd_data = HW_REG_WORD(expslv1_base_addr,0x20);
	  //  MIPI_PRINTF("PLL Lock wait 0x%0x \n", rd_data);
	  //} while( (rd_data & 0x00000001) != 0x00000001);

	  //check DSI DPHY state
	  //dphy4txtester_DIG_RD_TX_SYS_0
	  r_data_8 = DSI_DPHY_REG_READ(0x1E);
	  MIPI_PRINTF("MIPI DSI DPHY Read 1E 0x%0x \n", r_data_8);

	  //shadow registers interface (see ??Initialization?? on page 33 for additional details);

	  //17. Set basedir_n = 1'b0;
	  //18. Set forcerxmode_n = 1'b0;
	  HW_REG_WORD((expslv1_base_addr),0x34)=0x00000000;

	  //19. Set all requests inputs to zero;
	  //20. Wait for 15 ns;
	  for(int i=0;i<5;i++);

	  //21. Set enable_n and enableclk=1'b1;
	  HW_REG_WORD((mipi_dsi_base_addr),0xa0)=0x00000004;

	  //22. Wait 5 ns;
	  for(int i=0;i<5;i++);

	  //23. Set shutdownz=1'b1;
	  HW_REG_WORD((mipi_dsi_base_addr),0xa0)=0x00000005;

	  //24. Wait 5 ns;
	  for(int i=0;i<5;i++);

	  //25. Set rstz=1'b1;
	  HW_REG_WORD((mipi_dsi_base_addr),0xa0)=0x00000007;

	  //dphy4txtester_DIG_RDWR_TX_PLL_17
	  // 7 pll_pwron_ovr_rw R/W Description: PLL poweron override
	  // 6 pll_pwron_ovr_en_rw R/W Description: PLL poweron override enable control
	  r_data_8 = DSI_DPHY_REG_READ(0x16e);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x16e 0x%0x \n", r_data_8);

	  DSI_DPHY_REG_WRITE(0x16e,0xD0);

	  r_data_8 = DSI_DPHY_REG_READ(0x16e);
	  MIPI_PRINTF("MIPI DSI DPHY Read 0x16e 0x%0x \n", r_data_8);

	  ////force PLL lock
	  //r_data_8 = DSI_DPHY_REG_READ(0x15F);
	  //MIPI_PRINTF("MIPI DSI DPHY Read 0x15F 0x%0x \n", r_data_8);

	  //DSI_DPHY_REG_WRITE(0x15F,0x01);

	  //r_data_8 = DSI_DPHY_REG_READ(0x15F);
	  //MIPI_PRINTF("MIPI DSI DPHY Read 0x15F 0x%0x \n", r_data_8);

	  ////pll_soc_force_lock [0]
	  //MIPI_PRINTF("MIPI DSI dphy_pll_ctrl0 Read 0x10 0x%0x \n", rd_data);
	  //rd_data = HW_REG_WORD(expslv1_base_addr,0x10);
	  //rd_data |= 1UL << 0;
	  //HW_REG_WORD(expslv1_base_addr,0x10) = rd_data;
	  //MIPI_PRINTF("MIPI DSI dphy_pll_ctrl0 Read 0x10 0x%0x \n", rd_data);


	  //26. Wait until stopstatedata_n and stopstateclk outputs are asserted indicating PHY is driving LP11 in
	  //enabled datalanes and clocklane.
	  do {
	    //dphy4rxtester_DIG_RD_RX_SYS_0 0x1e
	    //suggested by synopsys for PLL checking
	    r_data_8 = DSI_DPHY_REG_READ(0x1e);
	    MIPI_PRINTF("MIPI DSI DPHY Read 0x1e 0x%0x \n", r_data_8);
	    //dphy4txtester_DIG_RD_TX_PLL_0
	    //7 pll_vpl_det R Description: Supply presence detector (volatile)
	    //6 pll_clksel_en R Description: PHY internal clksel override (for lane startup sequence) (volatile)
	    //5 pll_lock R Description: PLL lock observability (volatile)
	    //4 pll_lock_det_on R Description: PLL lock detector power-on (volatile)
	    //3 pll_gear_shift R Description: PLL gear shift (volatile)
	    //2:1 pll_clksel_in__1__0__ R Description: PLL clksel[1:0] output clock selection (volatile)
	    //0 onpll R Description: PLL power-on (volatile)
	    r_data_8 = DSI_DPHY_REG_READ(0x191);
	    MIPI_PRINTF("MIPI DSI DPHY Read 0x191 0x%0x \n", r_data_8);
	    for(int i=0;i<5000;i++);
	    rd_data = HW_REG_WORD((mipi_dsi_base_addr),0xB0);
	    MIPI_PRINTF("MIPI DSI DPHY Host Ctrl Read 0xB0 0x%0x \n", rd_data);

	  } while( (rd_data & 0x00000094) != 0x00000094);

/*
	  //////////////////////////////////////////////////////////////////////////////
	  //// Display LCD back light power
	  //////////////////////////////////////////////////////////////////////////////

	  rd_data = HW_REG_WORD((0x70070170),0x0);
	  MIPI_PRINTF("0x70070170 0x%0x \n", rd_data);

	  //10'h170: prdata[7:0] = gpio_p4v_4_pc_reg;
	  //HW_REG_WORD((0x70070170),0x0) = 0xA3;
	  HW_REG_WORD((0x70070170),0x0) = 0x6F;
	  //HW_REG_WORD((0x70070178),0x0) = 0xC7;

	  rd_data = HW_REG_WORD((0x70070170),0x0);
	  MIPI_PRINTF("0x70070170 0x%0x \n", rd_data);

	  //change direction
	  HW_REG_WORD((0x70000000),0x4) = 0x00;

	  //high
	  rd_data = HW_REG_WORD((0x70000000),0x0);
	  MIPI_PRINTF("0x70000000 0x%0x \n", rd_data);

	  HW_REG_WORD((0x70000000),0x0) = (rd_data|0x10);

	  rd_data = HW_REG_WORD((0x70000000),0x0);
	  MIPI_PRINTF("0x70000000 0x%0x \n", rd_data);
*/
	  ////////////////////////////////////////////////////////////////////////////
	  // CDC200 Generator
	  ////////////////////////////////////////////////////////////////////////////
	  MIPI_PRINTF("CDC200 Generator \n");

//	  //pin mux to bring out parallel signals for debug
//	  HW_REG_WORD(0x71006010,0x00) =  0x55555555;
//	  HW_REG_WORD(0x71006010,0x04) =  0x55665555;
//	  HW_REG_WORD(0x71006010,0x08) =  0x56655555;
//	  HW_REG_WORD(0x71006010,0x0C) =  0x11115555;
	  //////////////////////////////////////////////

	  //number of layers = 1
	  HW_REG_WORD(cdc_base_addr,0x01*4) = 0x00000001;

	  //sync size
	  //[31:16] HSYNC = pixel width - 1 = 0
	  //[15:0] VSYNC = line - 1 = 0
	  //HW_REG_WORD(cdc_base_addr,0x02*4) = 0x00080003;
	  //HW_REG_WORD(cdc_base_addr,0x02*4) = 0x00400009; //hsync = 65-1; vsync = 10-1
	  HW_REG_WORD(cdc_base_addr,0x02*4) = (HSYNC-1)<<16|(VSYNC-1); //hsync = 64-1; vsync = 10-1
	  rd_data = HW_REG_WORD(cdc_base_addr,0x02*4);
	  MIPI_PRINTF("sync size %04x\n", rd_data);

	  //back porch
	  //[31:16] hsync and back porch -1 = 65 + 94 - 1 = 158 - 1 = 0x9D
	  //[15:0] vsync and back porch -1 = 10 + 20 - 1 = 29 = 0x1D
	  //HW_REG_WORD(cdc_base_addr,0x03*4) = 0x009D001D;
	  HW_REG_WORD(cdc_base_addr,0x03*4) = (HSYNC+HBP-1)<<16|(VSYNC+VBP-1);
	  rd_data = HW_REG_WORD(cdc_base_addr,0x03*4);
	  MIPI_PRINTF("back porch size %04x\n", rd_data);

	  //active width
	  //[31:16] Accumulated width including sync, back porch and active width -1 (pixels)
	  // 65 + 94 + 480 - 1 =  = 0x21B
	  //[15:0] Accumulated height including sync and back porch -1
	  // 30 + 800 - 1 = 839 = 0x33D
	  //HW_REG_WORD(cdc_base_addr,0x04*4) = 0x027D033D;
	  HW_REG_WORD(cdc_base_addr,0x04*4) = (HSYNC+HBP+HACT-1)<<16|(VSYNC+VBP+VACT-1);
	  rd_data = HW_REG_WORD(cdc_base_addr,0x04*4);
	  MIPI_PRINTF("active width size %04x\n", rd_data);

	  //Total width
	  //HTOTAL = 65 + 94 + 480 + big delay - 1 = 0x22F
	  //VTOTAL = 30 + 800 + 10 - 1 = 0x347
	  //HW_REG_WORD(cdc_base_addr,0x05*4) = 0x08000347;
	  HW_REG_WORD(cdc_base_addr,0x05*4) = (HSYNC+HBP+HACT+HFP-1)<<16|(VSYNC+VBP+VACT+VFP-1);
	  rd_data = HW_REG_WORD(cdc_base_addr,0x05*4);
	  MIPI_PRINTF("total width size %04x\n", rd_data);

	  //Layer 1
	  //window horizontal position
	  //horizontal stop position = active width + back porch +sync = 540 = 0x21C
	  //horizontal start position = back porch = 60 = 0x3C
	  //HW_REG_WORD(cdc_base_addr,(0x40+0x4)*4) = 0x021C003C;
	  HW_REG_WORD(cdc_base_addr,(0x40+0x4)*4) = (HSYNC+HBP+HACT-1)<<16|(HSYNC+HBP);
	  rd_data = HW_REG_WORD(cdc_base_addr,(0x40+0x4)*4);
	  MIPI_PRINTF("Layer 1 %04x\n", rd_data);

	  //window vertical position
	  //vertical stop position = 830 = 0x33E
	  //vertical start position = 30 = 0x1E
	  //HW_REG_WORD(cdc_base_addr,(0x40+0x5)*4) = 0x033E001E;
	  HW_REG_WORD(cdc_base_addr,(0x40+0x5)*4) = (VSYNC+VBP+VACT-1)<<16|(VSYNC+VBP);
	  rd_data = HW_REG_WORD(cdc_base_addr,(0x40+0x5)*4);
	  MIPI_PRINTF("window vertical position %04x\n", rd_data);

	  //Pixel Format - ID RGB888 - 24-bit RGB
	  HW_REG_WORD(cdc_base_addr,(0x40+0x7)*4) = 0x00000001;
	  rd_data = HW_REG_WORD(cdc_base_addr,(0x40+0x7)*4);
	  MIPI_PRINTF("pixel format %04x\n", rd_data);

	  //color FB start address
	  HW_REG_WORD(cdc_base_addr,(0x40+0xd)*4) = (uint32_t) &lcd_image;
	  rd_data = HW_REG_WORD(cdc_base_addr,(0x40+0xd)*4);
	  MIPI_PRINTF("color FB start address %04x\n", rd_data);

	  //color FB length
	  //Pitch in bytes (one line = 480 pixels = 480 x 3 bytes = 0x630
	  //bytes + 7
	  //HW_REG_WORD(cdc_base_addr,(0x40+0x0e)*4) = 0x05A005A7;
	  HW_REG_WORD(cdc_base_addr,(0x40+0x0e)*4) = (HACT*3)<<16|(HACT*3+7);
	  rd_data = HW_REG_WORD(cdc_base_addr,(0x40+0xe)*4);
	  MIPI_PRINTF("color FB length %04x\n", rd_data);

	  //color FB lines = 272
	  //HW_REG_WORD(cdc_base_addr,(0x40+0xf)*4) = 0x00000110;
	  HW_REG_WORD(cdc_base_addr,(0x40+0xf)*4) = VACT;
	  rd_data = HW_REG_WORD(cdc_base_addr,(0x40+0xf)*4);
	  MIPI_PRINTF("color FB lines %04x\n", rd_data);

	  //layer control - enable layer on
	  //HW_REG_WORD(cdc_base_addr,0x43*4) = 0x00000001;

	  //end of layer1

	  //enable single frame mode
	  //rd_data = HW_REG_WORD((cdc_base_addr+0x06*4),0);
	  //HW_REG_WORD(cdc_base_addr,0x06*4) = (rd_data | 1 << 24);
	  //rd_data = HW_REG_WORD(cdc_base_addr,0x06*4);
	  //MIPI_PRINTF("enable single frame mode %04x\n", rd_data);

	  //single frame enable
	  //rd_data = HW_REG_WORD((cdc_base_addr+0x08*4),0);
	  //HW_REG_WORD(cdc_base_addr,0x08*4) = (rd_data | 1 << 12);

	  //layer control - enable layer on
	  HW_REG_WORD(cdc_base_addr,(0x40+0x03)*4) = 0x00000001;
	  rd_data = HW_REG_WORD(cdc_base_addr,(0x40+0x3)*4);
	  MIPI_PRINTF("layer on %04x\n", rd_data);

	  //layer2 on
	  HW_REG_WORD(cdc_base_addr,(0x80+0x03)*4) = 0x00000001;
	  rd_data = HW_REG_WORD(cdc_base_addr,(0x80+0x3)*4);
	  MIPI_PRINTF("layer 2 on %04x\n", rd_data);

	  //default color - transparent black
	  HW_REG_WORD(cdc_base_addr,(0x80+0x09)*4) = 0x00000000;
	  rd_data = HW_REG_WORD(cdc_base_addr,(0x80+0x9)*4);
	  MIPI_PRINTF("layer 2 transparent black %04x\n", rd_data);

	  //immediate reload
	  HW_REG_WORD(cdc_base_addr,0x09*4) = 0x00000001;
	  rd_data = HW_REG_WORD(cdc_base_addr,0x09*4);
	  MIPI_PRINTF("immediate reload %04x\n", rd_data);

	  //line irq on
	  HW_REG_WORD(cdc_base_addr,0x0d*4) = 0x00000001;
	  rd_data = HW_REG_WORD(cdc_base_addr,0x0d*4);
	  MIPI_PRINTF("line irq on %04x\n", rd_data);

	  //clear line irq
	  HW_REG_WORD(cdc_base_addr,0x0f*4) = 0x00000001;
	  rd_data = HW_REG_WORD(cdc_base_addr,0x0f*4);
	  MIPI_PRINTF("clear line irq %04x\n", rd_data);

	  //global enable, CRC enable
	  HW_REG_WORD(cdc_base_addr,0x06*4) = 0x00002221;
	  rd_data = HW_REG_WORD(cdc_base_addr,0x06*4);
	  MIPI_PRINTF("global enable %04x\n", rd_data);

	  MIPI_PRINTF("cdc200 Done!\n");

	  for(int i=0;i<500000;i++)
	  {
		  if (i%5000 == 0) {MIPI_PRINTF("i %d\n",i);}
	  }

	  //////////////////////////////////////////////////////////////////////////////
	  //// Display_resetn
	  //////////////////////////////////////////////////////////////////////////////

	  //P4_6
	  rd_data = HW_REG_WORD((0x70070178),0x0);
	  MIPI_PRINTF("0x70070178 0x%0x \n", rd_data);

	  HW_REG_WORD((0x70070178),0x0) = 0xA3;
	  //HW_REG_WORD((0x70070178),0x0) = 0xC7;

	  rd_data = HW_REG_WORD((0x70070178),0x0);
	  MIPI_PRINTF("0x70070178 0x%0x \n", rd_data);

	  //change direction
	  HW_REG_WORD((0x70000000),0x4) = 0x00;

	  //high
	  HW_REG_WORD((0x70000000),0x0) = 0x40;

	  //for(int i=0;i<50000;i++); //it is around 1.4 ms measured by LA
	  for(int i=0;i<50000;i++); //it is around 1.4 ms measured by LA

	  //low
	  HW_REG_WORD((0x70000000),0x0) = 0x00;

	  //for(int i=0;i<50000;i++); //it is around 1.4 ms measured by LA
	  for(int i=0;i<500000;i++); //it is around 1.4 ms measured by LA

	  //high
	  HW_REG_WORD((0x70000000),0x0) = 0x40;

	  ////////////////////////////////////////////////////////////////////////////
	  // Video Mode Pattern Generator
	  ////////////////////////////////////////////////////////////////////////////
	  MIPI_PRINTF("MIPIDSI Video Mode Pattern Generator \n");

	  //LPCLK_CTR
	  //This register configures the possibility for using non continuous clock in the clock lane.
	  //1 auto_clklane_ctrl R/W This bit enables the automatic mechanism to stop providing clock in the clock lane when time allows.
	  //0 phy_txrequestclkhs R/W This bit controls the D-PHY PPI txrequestclkhs signal.
	  //from synopsys recommandation
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x94);
	  MIPI_PRINTF("EXPSLV1 0x94 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),0x94)=0x00000002;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x94);
	  MIPI_PRINTF("EXPSLV1 0x94 0x%0x \n", rd_data);

	  //PHY_TMR_LPCLK_CFG
	  //This register sets the time that DWC_mipi_dsi_host assumes
	  //in calculations for the clock lane to switch between high-speed and low-power.
	  //9:0 phy_clklp2hs_time R/W This field configures the maximum time that the D-PHY
	  //    clock lane takes to go from low-power to high-speed transmission measured in lane byte clock cycles.
	  //25:16 phy_clkhs2lp_time R/W This field configures the maximum time that the D-PHY
	  //    clock lane takes to go from high-speed to low-power transmission measured in lane byte clock cycles.
	  //	video->max_clk_hs_to_lp_cycles = 66;  /* value for max freq */
	  //	video->max_clk_lp_to_hs_cycles = 181; /* value for max freq */
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x98);
	  MIPI_PRINTF("EXPSLV1 0x98 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),0x98)=0x42<<16|0xB5;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x98);
	  MIPI_PRINTF("EXPSLV1 0x98 0x%0x \n", rd_data);

	  //PHY_TMR_CFG
	  //This register sets the time that DWC_mipi_dsi_host assumes
	  //in calculations for the data lanes to switch between high-speed and low-power.
	  //9:0 phy_lp2hs_time R/W This field configures the maximum time that the D-PHY
	  //    data lanes take to go from low-power to high-speed transmission measured in lane byte clock cycles.
	  //25:16 phy_hs2lp_time R/W This field configures the maximum time that the D-PHY
	  //    data lanes take to go from high-speed to low-power transmission measured in lane byte clock cycles.
	  //	video->max_hs_to_lp_cycles = 50;	  /* value for max freq */
	  //	video->max_lp_to_hs_cycles = 153;	 /* value for max freq */
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x9C);
	  MIPI_PRINTF("EXPSLV1 0x9C 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),0x9C)=0x32<<16|0x99;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x9C);
	  MIPI_PRINTF("EXPSLV1 0x9C 0x%0x \n", rd_data);

	  //0. This register controls the power up of the controller.
	  //0 shutdownz R/W This bit configures the controller either to power up or to reset
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x04);
	  MIPI_PRINTF("EXPSLV1 0x04 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),0x04)=0x00000001|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x04);
	  MIPI_PRINTF("EXPSLV1 0x04 0x%0x \n", rd_data);

	  //This register configures the polarity of DPI signals.
	  //0 dataen_active_low
	  //1 vsync_active_low
	  //2 hsync_active_low
	  //rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x14);
	  //MIPI_PRINTF("EXPSLV1 0x14 0x%0x \n", rd_data);

	  //HW_REG_WORD((mipi_dsi_base_addr),0x14)=0x00000001;

	  //rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x14);
	  //MIPI_PRINTF("EXPSLV1 0x14 0x%0x \n", rd_data);

	  //1. configure MODE_CFG register to enable command mode
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x34);
	  MIPI_PRINTF("EXPSLV1 0x34 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),0x34)=0x00000001|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x34);
	  MIPI_PRINTF("EXPSLV1 0x34 0x%0x \n", rd_data);

	  //1.1 configure Vdieo Mode type VID_MODE_CFG[1:0]
	  //1:0 vid_mode_type R/W This field indicates the video mode transmission type
	  //15 lp_cmd_en R/W When set to 1, this bit enables the command transmission only in low-power mode.
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x38);
	  MIPI_PRINTF("EXPSLV1 0x38 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),0x38)=0x00008002|rd_data; //burst mode

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x38);
	  MIPI_PRINTF("EXPSLV1 0x38 0x%0x \n", rd_data);

	  //The following are added for ESC mode LPDT
	  //CLKMGR_CFG
	  //7:0 tx_esc_clk_division R/W
	  //  This field indicates the division factor for the TX Escape clock source (lanebyteclk).
	  //  The values 0 and 1 stop the TX_ESC clock generation.
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_CLKMGR_CFG);
	  MIPI_PRINTF("EXPSLV1 A_CLKMGR_CFG 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),A_CLKMGR_CFG)=0x00000002|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_CLKMGR_CFG);
	  MIPI_PRINTF("EXPSLV1 A_CLKMGR_CFG 0x%0x \n", rd_data);

	  //CMD_MODE_CFG
	  //24 max_rd_pkt_size
	  //19 dcs_lw_tx
	  //18 dcs_sr_0p_tx
	  //17 dcs_sw_1p_tx
	  //16 dcs_sw_0p_tx
	  //14 gen_lw_tx
	  //13 gen_sr_2p_tx
	  //12 gen_sr_1p_tx R/W This bit configures the Generic short read packet with one parameter command transmission type:
	  //11 gen_sr_0p_tx R/W This bit configures the Generic short read packet with zero parameter command transmission type:
	  //10 gen_sw_2p_tx R/W This bit configures the Generic short write packet with two parameters command transmission type:
	  //9 gen_sw_1p_tx R/W This bit configures the Generic short write packet with one parameter command transmission type:
	  //8 gen_sw_0p_tx R/W This bit configures the Generic short write packet with zero parameter command transmission type:

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_CMD_MODE_CFG);
	  MIPI_PRINTF("EXPSLV1 A_CMD_MODE_CFG 0x%0x \n", rd_data);
	  //low power = 1; high speed = 0;
	  HW_REG_WORD((mipi_dsi_base_addr),A_CMD_MODE_CFG)=0x010F7F00|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_CMD_MODE_CFG);
	  MIPI_PRINTF("EXPSLV1 A_CMD_MODE_CFG 0x%0x \n", rd_data);

	  ////////////////////////////////////////////////////////////////////////////
	  // Read Display Power Mode (0Ah) - ILI9806E DISPLAY REGISTERS
	  ////////////////////////////////////////////////////////////////////////////
	  /*
	  //1stParameter Read BSTON 0 0 SLPOUT NORON DISON 0 0 XXh
	  //default - 0x80
	  //Step 1: ?? The MPU sends ??Set Maximum Return Packet Size?? (Short Packet (SPa)) (SMRPS-S) to the display module
	  //when it wants to return one byte from the display module
	  //
	  //Virtual Channel (VC, DI[7??6]): 00b
	  //Data Type (DT, DI[5??0]): 11 0111b
	  // Maximum Return Packet Size (MRPS)
	  //Data 0: 01hex
	  //Data 1: 00hex

	  //37 Set Maximum Return Packet Size SPa (Short Packet) SMRPS-S

	  //GEN_HDR
	  //5:0 gen_dt R/W This field configures the packet Data Type of the header packet. 0x37
	  //7:6 gen_vc R/W This field configures the Virtual Channel ID of the header packet.
	  //15:8 gen_wc_lsbyte R/W This field configures the least significant byte of the header packet's Word count for long packets or data 0 for short packets.
	  //23:16 gen_wc_msbyte R/W This field configures the most significant byte of the header packet's word count for long packets or data 1 for short packets.

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR);
	  MIPI_PRINTF("EXPSLV1 A_GEN_HDR 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR)=0x00000137|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR);
	  MIPI_PRINTF("EXPSLV1 A_GEN_HDR 0x%0x \n", rd_data);

	  //rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_PHY_STATUS);
	  //MIPI_PRINTF("MIPI DSI DPHY Host Ctrl Read A_PHY_STATUS 0x%0x \n", rd_data);

	  //apb_read( `A_GEN_PLD_DATA           ,   A_GEN_PLD_DATA[31:0]      ,   "VERBOSE_MODE");

	  //wait for first LPDT to complete before next
	  //4 phy_stopstate0lane R This bit indicates the status of phystopstate0lane D-PHY signal.
	  do {
	    for(int i=0;i<5000;i++);
	    rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_PHY_STATUS);
	    MIPI_PRINTF("MIPI DSI DPHY Host Ctrl Read A_PHY_STATUS 0x%0x \n", rd_data);
	  } while( (rd_data & 0x00000010) != 0x00000010);

	  //06 DCS Read, No Parameter SPa (Short Packet) DCSRN-S
	  //Read Display Power Mode (0Ah)
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR);
	  MIPI_PRINTF("EXPSLV1 A_GEN_HDR 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR)=0x00000A06;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR);
	  MIPI_PRINTF("EXPSLV1 A_GEN_HDR 0x%0x \n", rd_data);

	  //This register sets the payload for packets sent using the Generic interface and, when read returns the contents of READ responses from the peripheral
	  //A_GEN_PLD_DATA
	  //7:0 gen_pld_b1 R/W This field indicates byte 1 of the packet payload.
	  do {
	    for(int i=0;i<500;i++);
	    rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_PLD_DATA);
	    MIPI_PRINTF("EXPSLV1 A_GEN_PLD_DATA 0x%0x \n", rd_data);
	  } while (rd_data == 0x0);

	  //05 DCS Write, No Parameter SPa (Short Packet) DCSWN-S
	  //Normal Display mode on
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR);
	  MIPI_PRINTF("EXPSLV1 A_GEN_HDR 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR)=0x00001305;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR);
	  MIPI_PRINTF("EXPSLV1 A_GEN_HDR 0x%0x \n", rd_data);

	  //05 DCS Write, No Parameter SPa (Short Packet) DCSWN-S
	  //All Pixel On
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR);
	  MIPI_PRINTF("EXPSLV1 A_GEN_HDR 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR)=0x00002305;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR);
	  MIPI_PRINTF("EXPSLV1 A_GEN_HDR 0x%0x \n", rd_data);

	  //05 DCS Write, No Parameter SPa (Short Packet) DCSWN-S
	  //Display On
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR);
	  MIPI_PRINTF("EXPSLV1 A_GEN_HDR 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR)=0x00002905;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR);
	  MIPI_PRINTF("EXPSLV1 A_GEN_HDR 0x%0x \n", rd_data);

	  */

	  //***************************************************************//LCD SETING
	  //write_command(0xFF);        // Change to Page 1 CMD
	  //write_data(0xFF);
	  //write_data(0x98);
	  //write_data(0x06);
	  //write_data(0x04);
	  //write_data(0x01);

	  DCSW_L(0xFF, 0x98, 0x06, 0x04, 0x01);
	  DCSW1_S(0x08,0x10);

	  DCSW1_S(0x20,0x00);
	  DCSW1_S(0x21,0x01);

	  DCSW1_S(0x30,0x01);
	  DCSW1_S(0x31,0x00);

	  DCSW1_S(0x40,0x16); //
	  DCSW1_S(0x41,0x33);//22
	  DCSW1_S(0x42,0x03); //VGL=DDVDH+VCIP -DDVDL,VGH=2DDVDL-VCIP
	  DCSW1_S(0x43,0x09); //SET VGH clamp level
	  DCSW1_S(0x44,0x06); //SET VGL clamp level

	  DCSW1_S(0x50,0x88);
	  DCSW1_S(0x51,0x88);
	  DCSW1_S(0x52,0x00);
	  DCSW1_S(0x53,0x49); //VCOM
	  DCSW1_S(0x55,0x49);

	  DCSW1_S(0x60,0x07);
	  DCSW1_S(0x61,0x00);
	  DCSW1_S(0x62,0x07);
	  DCSW1_S(0x63,0x00);

	  //++++++++++++++++++ Gamma Setting ++++++++++++++++++//
	  DCSW1_S(0xA0,0x00);
	  DCSW1_S(0xA1,0x09);
	  DCSW1_S(0xA2,0x11);
	  DCSW1_S(0xA3,0x0B);
	  DCSW1_S(0xA4,0x05);
	  DCSW1_S(0xA5,0x08);
	  DCSW1_S(0xA6,0x06);
	  DCSW1_S(0xA7,0x04);
	  DCSW1_S(0xA8,0x09);
	  DCSW1_S(0xA9,0x0C);
	  DCSW1_S(0xAA,0x15);
	  DCSW1_S(0xAB,0x08);
	  DCSW1_S(0xAC,0x0F);
	  DCSW1_S(0xAD,0x12);
	  DCSW1_S(0xAE,0x09);
	  DCSW1_S(0xAF,0x00);
	   ///==============Nagitive
	  DCSW1_S(0xC0,0x00);
	  DCSW1_S(0xC1,0x09);
	  DCSW1_S(0xC2,0x10);
	  DCSW1_S(0xC3,0x0C);
	  DCSW1_S(0xC4,0x05);
	  DCSW1_S(0xC5,0x08);
	  DCSW1_S(0xC6,0x06);
	  DCSW1_S(0xC7,0x04);
	  DCSW1_S(0xC8,0x08);
	  DCSW1_S(0xC9,0x0C);
	  DCSW1_S(0xCA,0x14);
	  DCSW1_S(0xCB,0x08);
	  DCSW1_S(0xCC,0x0F);
	  DCSW1_S(0xCD,0x11);
	  DCSW1_S(0xCE,0x09);
	  DCSW1_S(0xCF,0x00);

	  //DCSW1_S(0xFF); // Change to Page 6 CMD for GIP timing
	  //write_data(0xFF);
	  //write_data(0x98);
	  //write_data(0x06);
	  //write_data(0x04);
	  //write_data(0x06);

	  DCSW_L(0xFF, 0x98, 0x06, 0x04, 0x06);

	  DCSW1_S(0x00,0x20);
	  DCSW1_S(0x01,0x0A);
	  DCSW1_S(0x02,0x00);
	  DCSW1_S(0x03,0x00);
	  DCSW1_S(0x04,0x01);
	  DCSW1_S(0x05,0x01);
	  DCSW1_S(0x06,0x98);
	  DCSW1_S(0x07,0x06);
	  DCSW1_S(0x08,0x01);
	  DCSW1_S(0x09,0x80);
	  DCSW1_S(0x0A,0x00);
	  DCSW1_S(0x0B,0x00);
	  DCSW1_S(0x0C,0x01);
	  DCSW1_S(0x0D,0x01);
	  DCSW1_S(0x0E,0x05);
	  DCSW1_S(0x0F,0x00);

	  DCSW1_S(0x10,0xF0);
	  DCSW1_S(0x11,0xF4);
	  DCSW1_S(0x12,0x01);
	  DCSW1_S(0x13,0x00);
	  DCSW1_S(0x14,0x00);
	  DCSW1_S(0x15,0xC0);
	  DCSW1_S(0x16,0x08);
	  DCSW1_S(0x17,0x00);
	  DCSW1_S(0x18,0x00);
	  DCSW1_S(0x19,0x00);
	  DCSW1_S(0x1A,0x00);
	  DCSW1_S(0x1B,0x00);
	  DCSW1_S(0x1C,0x00);
	  DCSW1_S(0x1D,0x00);

	  DCSW1_S(0x20,0x01);
	  DCSW1_S(0x21,0x23);
	  DCSW1_S(0x22,0x45);
	  DCSW1_S(0x23,0x67);
	  DCSW1_S(0x24,0x01);
	  DCSW1_S(0x25,0x23);
	  DCSW1_S(0x26,0x45);
	  DCSW1_S(0x27,0x67);

	  DCSW1_S(0x30,0x11);
	  DCSW1_S(0x31,0x11);
	  DCSW1_S(0x32,0x00);
	  DCSW1_S(0x33,0xEE);
	  DCSW1_S(0x34,0xFF);
	  DCSW1_S(0x35,0xBB);
	  DCSW1_S(0x36,0xAA);
	  DCSW1_S(0x37,0xDD);
	  DCSW1_S(0x38,0xCC);
	  DCSW1_S(0x39,0x66);
	  DCSW1_S(0x3A,0x77);
	  DCSW1_S(0x3B,0x22);
	  DCSW1_S(0x3C,0x22);
	  DCSW1_S(0x3D,0x22);
	  DCSW1_S(0x3E,0x22);
	  DCSW1_S(0x3F,0x22);
	  DCSW1_S(0x40,0x22);

	  //DCSW1_S(0xFF);// Change to Page 7 CMD for GIP timing
	  //write_data(0xFF);
	  //write_data(0x98);
	  //write_data(0x06);
	  //write_data(0x04);
	  //write_data(0x07);

	  DCSW_L(0xFF, 0x98, 0x06, 0x04, 0x07);

	  DCSW1_S(0x17,0x22);

	  DCSW1_S(0x02,0x77);

	  DCSW1_S(0x26,0xB2);

	  //DCSW1_S(0xFF); // Change to Page 0 CMD for Normal command
	  //write_data(0xFF);
	  //write_data(0x98);
	  //write_data(0x06);
	  //write_data(0x04);
	  //write_data(0x00);

	  DCSW_L(0xFF, 0x98, 0x06, 0x04, 0x00);

	  DCSWN_S(0x35); //TE ON

	  DCSW1_S(0x3A,0x70); //24BIT

	  //DCSWN_S(0x11);
	  //delay(120);
	  //for(int i=0;i<120;i++);
	  //DCSWN_S(0x29);
	  //delay(25);
	  //for(int i=0;i<125;i++);

/*
	  //37 Set Maximum Return Packet Size SPa (Short Packet) SMRPS-S

	  //GEN_HDR
	  //5:0 gen_dt R/W This field configures the packet Data Type of the header packet. 0x37
	  //7:6 gen_vc R/W This field configures the Virtual Channel ID of the header packet.
	  //15:8 gen_wc_lsbyte R/W This field configures the least significant byte of the header packet's Word count for long packets or data 0 for short packets.
	  //23:16 gen_wc_msbyte R/W This field configures the most significant byte of the header packet's word count for long packets or data 1 for short packets.
	  SMRPS_S(0x1);

	  //06 DCS Read, No Parameter SPa (Short Packet) DCSRN-S
	  //Read Display Power Mode (0Ah)
	  DCSRR1_S(0x0A);

	  //This register sets the payload for packets sent using the Generic interface and, when read returns the contents of READ responses from the peripheral
	  //A_GEN_PLD_DATA
	  //7:0 gen_pld_b1 R/W This field indicates byte 1 of the packet payload.
	  do {
	    for(int i=0;i<500;i++);
	    rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_PLD_DATA);
	    MIPI_PRINTF("EXPSLV1 A_GEN_PLD_DATA 0x%0x \n", rd_data);
	  } while (rd_data == 0x0);
*/



	  //sleep out
	  //DCSWN_S(0x11);

	  //05 DCS Write, No Parameter SPa (Short Packet) DCSWN-S
	  //Normal Display mode on
	  //DCSW1_S(0x05, 0x13);

	  //05 DCS Write, No Parameter SPa (Short Packet) DCSWN-S
	  //All Pixel On
	  //DCSW1_S(0x05, 0x23);

	  //05 DCS Write, No Parameter SPa (Short Packet) DCSWN-S
	  //Display On
	  //DCSW1_S(0x05, 0x29);

	  //sleep out
	  DCSWN_S(0x11);
	  //delay(120);
	  for(int i=0;i<120;i++);
	  //Display on
	  DCSWN_S(0x29);
	  //delay(25);
	  //for(int i=0;i<125;i++);

	  ////////////////////////////////////////////////////////////////////////////
	  // video mode
	  ////////////////////////////////////////////////////////////////////////////

	  //This register configures the possibility for using non continuous clock in the clock lane.
	  //0 phy_txrequestclkhs R/W This bit controls the D-PHY PPI txrequestclkhs signal.
	  //from synopsys recommandation
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x94);
	  MIPI_PRINTF("EXPSLV1 0x94 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),0x94)=0x00000001|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x94);
	  MIPI_PRINTF("EXPSLV1 0x94 0x%0x \n", rd_data);

	  //1. configure MODE_CFG register to enable command mode
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x34);
	  MIPI_PRINTF("EXPSLV1 0x34 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),0x34)=0x00000000&rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x34);
	  MIPI_PRINTF("EXPSLV1 0x34 0x%0x \n", rd_data);

	  //2. configure DPI_COLOR_CODING register
	  // 0x5 (CC05): 24-bit
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x10);
	  MIPI_PRINTF("EXPSLV1 0x10 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),0x10)=0x00000005|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x10);
	  MIPI_PRINTF("EXPSLV1 0x10 0x%0x \n", rd_data);

	  //2. DPI_CFG_POL - This register configures the polarity of DPI signals.
	  // 0x1 vsync_active_low
	  // 0x2 hsync_active_low
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x14);
	  MIPI_PRINTF("EXPSLV1 0x14 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),0x14)=0x00000006|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x14);
	  MIPI_PRINTF("EXPSLV1 0x14 0x%0x \n", rd_data);

	  //3. vid_pkt_size
	  // pixel perline
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x3C);
	  MIPI_PRINTF("EXPSLV1 0x3C 0x%0x \n", rd_data);

	  //HW_REG_WORD((mipi_dsi_base_addr),0x3c)=0x000001EC|rd_data; //480+12
	  //HW_REG_WORD((mipi_dsi_base_addr),0x3c)=0x000001E0|rd_data;
	  HW_REG_WORD((mipi_dsi_base_addr),0x3c)=HACT|rd_data; //active video 480

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x3C);
	  MIPI_PRINTF("EXPSLV1 0x3C 0x%0x \n", rd_data);

	  //4. vid_num_chunks
	  // chunks per line
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x40);
	  MIPI_PRINTF("EXPSLV1 0x40 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),0x40)=0x00000000|rd_data; //0 - no chunks

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x40);
	  MIPI_PRINTF("EXPSLV1 0x40 0x%0x \n", rd_data);

	  //5. null packet
	  //12:0 vid_null_size R/W This register configures the number of bytes inside a null packet. Setting to 0 disables null packets.
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x44);
	  MIPI_PRINTF("EXPSLV1 0x44 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),0x44)=0x00000000|rd_data; //disable

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x44);
	  MIPI_PRINTF("EXPSLV1 0x44 0x%0x \n", rd_data);

	  //6. This register configures the video HSA time.
	  //11:0 vid_hsa_time R/W This field configures the Horizontal Synchronism Active period in lane byte clock cycles.
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x48);
	  MIPI_PRINTF("EXPSLV1 0x48 0x%0x \n", rd_data);

	  //HSA 64 bits
	  //HW_REG_WORD((mipi_dsi_base_addr),0x48)=0x00000040|rd_data;
	  HW_REG_WORD((mipi_dsi_base_addr),0x48)=HSYNC|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x48);
	  MIPI_PRINTF("EXPSLV1 0x48 0x%0x \n", rd_data);

	  //7. This register configures the video HBP time.
	  //11:0 vid_hbp_time R/W This field configures the Horizontal Back Porch period in lane byte clock cycles
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x4C);
	  MIPI_PRINTF("EXPSLV1 0x4C 0x%0x \n", rd_data);

	  //HBP 94
	  //HW_REG_WORD((mipi_dsi_base_addr),0x4C)=0x0000005e|rd_data;
	  HW_REG_WORD((mipi_dsi_base_addr),0x4C)=HBP|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x4C);
	  MIPI_PRINTF("EXPSLV1 0x4C 0x%0x \n", rd_data);

	  //8. This register configures the overall time for each video line.
	  //14:0 vid_hline_time R/W This field configures the size of the total line time (HSA+HBP+HACT+HFP) counted in lane byte clock cycles.
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x50);
	  MIPI_PRINTF("EXPSLV1 0x50 0x%0x \n", rd_data);

	  //HSD = 1688 (HSA+HBP+Hactive)
	  HW_REG_WORD((mipi_dsi_base_addr),0x50)=0x00000698|rd_data; //added big delay since clock is fast and we are in burst mode
	  //HW_REG_WORD((mipi_dsi_base_addr),0x50)=0x000001E0|rd_data; //480+2+5+5

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x50);
	  MIPI_PRINTF("EXPSLV1 0x50 0x%0x \n", rd_data);

	  //9. This register configures the VSA period.
	  //9:0 vsa_lines R/W This field configures the Vertical Synchronism Active period measured in number of horizontal lines.
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x54);
	  MIPI_PRINTF("EXPSLV1 0x54 0x%0x \n", rd_data);

	  //HW_REG_WORD((mipi_dsi_base_addr),0x54)=0x0000000a|rd_data;
	  HW_REG_WORD((mipi_dsi_base_addr),0x54)=VSYNC|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x54);
	  MIPI_PRINTF("EXPSLV1 0x54 0x%0x \n", rd_data);

	  //10. This register configures the VBP period.
	  //9:0 vbp_lines R/W This field configures the Vertical Back Porch period measured in number of horizontal lines.
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x58);
	  MIPI_PRINTF("EXPSLV1 0x58 0x%0x \n", rd_data);

	  //HW_REG_WORD((mipi_dsi_base_addr),0x58)=0x00000014|rd_data;
	  HW_REG_WORD((mipi_dsi_base_addr),0x58)=VBP|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x58);
	  MIPI_PRINTF("EXPSLV1 0x58 0x%0x \n", rd_data);

	  //11. This register configures the VFP period.
	  //9:0 vfp_lines R/W This field configures the Vertical Front Porch period measured in number of horizontal lines.
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x5C);
	  MIPI_PRINTF("EXPSLV1 0x5C 0x%0x \n", rd_data);

	  //HW_REG_WORD((mipi_dsi_base_addr),0x5C)=0x000000a|rd_data;
	  HW_REG_WORD((mipi_dsi_base_addr),0x5C)=VFP|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x5C);
	  MIPI_PRINTF("EXPSLV1 0x5C 0x%0x \n", rd_data);

	  //12. This register configures the vertical resolution of video.
	  //13:0 v_active_lines R/W This field configures the Vertical Active period measured in number of horizontal lines.
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x60);
	  MIPI_PRINTF("EXPSLV1 0x60 0x%0x \n", rd_data);

	  //HW_REG_WORD((mipi_dsi_base_addr),0x60)=0x00000320|rd_data; //854
	  HW_REG_WORD((mipi_dsi_base_addr),0x60)=VACT|rd_data; //800

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x60);
	  MIPI_PRINTF("EXPSLV1 0x60 0x%0x \n", rd_data);

	  //05 DCS Write, No Parameter SPa (Short Packet) DCSWN-S
	  //software reset
	  //DCSW1_S(0x05, 0x01);

	  //05 DCS Write, No Parameter SPa (Short Packet) DCSWN-S
	  //Normal Display mode on
	  //DCSW1_S(0x05, 0x13);

	  //05 DCS Write, No Parameter SPa (Short Packet) DCSWN-S
	  //All Pixel On
	  //DCSW1_S(0x05, 0x23);

	  //05 DCS Write, No Parameter SPa (Short Packet) DCSWN-S
	  //Display On
	  //DCSW1_S(0x05, 0x29);

	  //1. configure MODE_CFG register to enable video mode
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x34);
	  MIPI_PRINTF("EXPSLV1 0x34 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),0x34)=0x00000000&rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x34);
	  MIPI_PRINTF("EXPSLV1 0x34 0x%0x \n", rd_data);

	  //13. configure Vdieo Mode type VID_MODE_CFG[1:0]
	  //24 vpg_orientation R/W This field indicates the color bar orientation as follows: 0x0 (VPGORENT0): Vertical mode
	  //20 vpg_mode R/W This field is to select the pattern: 0x0 (COLORBAR): horizontal or vertical
	  //16 vpg_en R/W When set to 1, this bit enables the video mode pattern generator.
	  //15 lp_cmd_en R/W When set to 1, this bit enables the command transmission only in low-power mode.
	  //1:0 vid_mode_type R/W This field indicates the video mode transmission type
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x38);
	  MIPI_PRINTF("EXPSLV1 0x38 0x%0x \n", rd_data);

	  rd_data = rd_data & ~0x8000; //disable bit 15
	  //HW_REG_WORD((mipi_dsi_base_addr),0x38)=rd_data; //burst mode, vpg_en = 0
	  HW_REG_WORD((mipi_dsi_base_addr),0x38)=0x00000000|rd_data; //burst mode

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x38);
	  MIPI_PRINTF("EXPSLV1 0x38 0x%0x \n", rd_data);

	  //////////////////////////////////////////////////////////////////////////////
	  //// Display LCD back light power
	  //////////////////////////////////////////////////////////////////////////////

	  rd_data = HW_REG_WORD((0x70070170),0x0);
	  MIPI_PRINTF("0x70070170 0x%0x \n", rd_data);

	  //10'h170: prdata[7:0] = gpio_p4v_4_pc_reg;
	  //HW_REG_WORD((0x70070170),0x0) = 0xA3;
	  HW_REG_WORD((0x70070170),0x0) = 0x6F;
	  //HW_REG_WORD((0x70070178),0x0) = 0xC7;

	  rd_data = HW_REG_WORD((0x70070170),0x0);
	  MIPI_PRINTF("0x70070170 0x%0x \n", rd_data);

	  //change direction
	  HW_REG_WORD((0x70000000),0x4) = 0x00;

	  //high
	  rd_data = HW_REG_WORD((0x70000000),0x0);
	  MIPI_PRINTF("0x70000000 0x%0x \n", rd_data);

	  HW_REG_WORD((0x70000000),0x0) = (rd_data|0x10);

	  rd_data = HW_REG_WORD((0x70000000),0x0);
	  MIPI_PRINTF("0x70000000 0x%0x \n", rd_data);
}

uint8_t CSI_DPHY_REG_READ(int start_add)
{

  //50
  //[0] phy_testclk R/W Clock to capture testdin bus contents into the macro, with
  //    testen signal controlling the operation selection.
  //[1] phy_testclr R/W When active, performs vendor specific interface initialization.
  //    Active High.
  //
  //54
  //[16] phy_testen R/W When asserted high, it configures an address write operation
  //     on the falling edge of testclk. When asserted low, it
  //     configures a data write operation on the rising edge of
  //     testclk
  //[15:8] phy_testdout R Vendor-specific 8-bit data output for reading data and other
  //      probing functionalities
  //[7:0] phy_testdin R/W Test interface 8-bit data input for programming internal
  //      registers and accessing test functionalities.

  int lsb_addr,read_data;
  uint8_t r_data_8;

  //1.
  HW_REG_WORD(mipi_csi_base_addr,0x0050) = 0x00000000;//a. testclk
  HW_REG_WORD(mipi_csi_base_addr,0x0054) = 0x00000000;//a. testen
  HW_REG_WORD(mipi_csi_base_addr,0x0054) = 0x00010000;//b. testen_high
  HW_REG_WORD(mipi_csi_base_addr,0x0050) = 0x00000002;//c. testclk high
  HW_REG_WORD(mipi_csi_base_addr,0x0054) = 0x00010000;//d. place testdin 0x00
  HW_REG_WORD(mipi_csi_base_addr,0x0050) = 0x00000000;//e. testclk low
  HW_REG_WORD(mipi_csi_base_addr,0x0054) = 0x00000000;//f. place testen low
  HW_REG_WORD(mipi_csi_base_addr,0x0054) = ((start_add >> 8));//g. MSB testdin
  HW_REG_WORD(mipi_csi_base_addr,0x0050) = 0x00000002;//h. set testclk high

  //2.
  HW_REG_WORD(mipi_csi_base_addr,0x0050) = 0x00000000;//a. testclk low
  HW_REG_WORD(mipi_csi_base_addr,0x0054) = 0x00010000;//b. testen_high
  HW_REG_WORD(mipi_csi_base_addr,0x0050) = 0x00000002;//c. set testclk high
  lsb_addr = ((start_add & 0x00ff) | 0x00010000);
  HW_REG_WORD(mipi_csi_base_addr,0x0054) = lsb_addr; //d. LSB testdin
  HW_REG_WORD(mipi_csi_base_addr,0x0050) = 0x00000000;//e. set testclk low

  read_data = HW_REG_WORD(mipi_csi_base_addr,0x0054);

  r_data_8 = read_data >> 8;

  HW_REG_WORD(mipi_csi_base_addr,0x0054) = 0x00000000;//f. testen_low

  return r_data_8;
}

void CSI_DPHY_REG_WRITE(int start_add,uint8_t write_data)
{

  //50
  //[1] phy_testclk R/W Clock to capture testdin bus contents into the macro, with
  //    testen signal controlling the operation selection.
  //[0] phy_testclr R/W When active, performs vendor specific interface initialization.
  //    Active High.
  //
  //54
  //[16] phy_testen R/W When asserted high, it configures an address write operation
  //     on the falling edge of testclk. When asserted low, it
  //     configures a data write operation on the rising edge of
  //     testclk
  //[15:8] phy_testdout R Vendor-specific 8-bit data output for reading data and other
  //      probing functionalities
  //[7:0] phy_testdin R/W Test interface 8-bit data input for programming internal
  //      registers and accessing test functionalities.

  int lsb_addr;

  //1.
  HW_REG_WORD(mipi_csi_base_addr,0x0050) = 0x00000000;//a. testclk
  HW_REG_WORD(mipi_csi_base_addr,0x0054) = 0x00000000;//a. testen
  HW_REG_WORD(mipi_csi_base_addr,0x0054) = 0x00010000;//b. testen_high
  HW_REG_WORD(mipi_csi_base_addr,0x0050) = 0x00000002;//c. testclk high
  HW_REG_WORD(mipi_csi_base_addr,0x0054) = 0x00010000;//d. place testdin 0x00
  HW_REG_WORD(mipi_csi_base_addr,0x0050) = 0x00000000;//e. testclk low
  HW_REG_WORD(mipi_csi_base_addr,0x0054) = 0x00000000;//f. place testen low
  HW_REG_WORD(mipi_csi_base_addr,0x0054) = ((start_add >> 8));//g. MSB testdin
  HW_REG_WORD(mipi_csi_base_addr,0x0050) = 0x00000002;//h. set testclk high

  //2.
  HW_REG_WORD(mipi_csi_base_addr,0x0050) = 0x00000000;//a. testclk low
  HW_REG_WORD(mipi_csi_base_addr,0x0054) = 0x00010000;//b. testen_high
  HW_REG_WORD(mipi_csi_base_addr,0x0050) = 0x00000002;//c. set testclk high
  lsb_addr = ((start_add & 0x00ff) | 0x00010000);
  HW_REG_WORD(mipi_csi_base_addr,0x0054) = lsb_addr; //d. LSB testdin
  HW_REG_WORD(mipi_csi_base_addr,0x0050) = 0x00000000;//e. set testclk low
  HW_REG_WORD(mipi_csi_base_addr,0x0054) = 0x00000000;//b. testen_low

  //3.
  HW_REG_WORD(mipi_csi_base_addr,0x0054) = write_data; //a.
  HW_REG_WORD(mipi_csi_base_addr,0x0050) = 0x00000002; //b. testclk high

  //turn off for clean exit
  HW_REG_WORD(mipi_csi_base_addr,0x0050) = 0x00000000;//a. testclk
  HW_REG_WORD(mipi_csi_base_addr,0x0054) = 0x00000000;//a. testen
}

uint8_t DSI_DPHY_REG_READ(int start_add)
{

  //50
  //[0] phy_testclk R/W Clock to capture testdin bus contents into the macro, with
  //    testen signal controlling the operation selection.
  //[1] phy_testclr R/W When active, performs vendor specific interface initialization.
  //    Active High.
  //
  //54
  //[16] phy_testen R/W When asserted high, it configures an address write operation
  //     on the falling edge of testclk. When asserted low, it
  //     configures a data write operation on the rising edge of
  //     testclk
  //[15:8] phy_testdout R Vendor-specific 8-bit data output for reading data and other
  //      probing functionalities
  //[7:0] phy_testdin R/W Test interface 8-bit data input for programming internal
  //      registers and accessing test functionalities.

  int lsb_addr, read_data;
  uint8_t r_data_8;

  //1.
  HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000000;//a. testclk
  HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00000000;//a. testen
  HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00010000;//b. testen_high
  HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000002;//c. testclk high
  HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00010000;//d. place testdin 0x00
  HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000000;//e. testclk low
  HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00000000;//f. place testen low
  HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = ((start_add >> 8));//g. MSB testdin
  HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000002;//h. set testclk high

  //2.
  HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000000;//a. testclk low
  HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00010000;//b. testen_high
  HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000002;//c. set testclk high
  lsb_addr = ((start_add & 0x00ff) | 0x00010000);
  HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = lsb_addr; //d. LSB testdin
  HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000000;//e. set testclk low

  read_data = HW_REG_WORD(mipi_dsi_base_addr,0x00b8);

  r_data_8 = read_data >> 8;

  HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00000000;//f. testen_low

  return r_data_8;
}

void DSI_DPHY_REG_WRITE(int start_add,uint8_t write_data)
{

  //50
  //[1] phy_testclk R/W Clock to capture testdin bus contents into the macro, with
  //    testen signal controlling the operation selection.
  //[0] phy_testclr R/W When active, performs vendor specific interface initialization.
  //    Active High.
  //
  //54
  //[16] phy_testen R/W When asserted high, it configures an address write operation
  //     on the falling edge of testclk. When asserted low, it
  //     configures a data write operation on the rising edge of
  //     testclk
  //[15:8] phy_testdout R Vendor-specific 8-bit data output for reading data and other
  //      probing functionalities
  //[7:0] phy_testdin R/W Test interface 8-bit data input for programming internal
  //      registers and accessing test functionalities.

  int lsb_addr;

  //1.
  HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000000;//a. testclk
  HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00000000;//a. testen
  HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00010000;//b. testen_high
  HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000002;//c. testclk high
  HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00010000;//d. place testdin 0x00
  HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000000;//e. testclk low
  HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00000000;//f. place testen low
  HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = ((start_add >> 8));//g. MSB testdin
  HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000002;//h. set testclk high

  //2.
  HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000000;//a. testclk low
  HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00010000;//b. testen_high
  HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000002;//c. set testclk high
  lsb_addr = ((start_add & 0x00ff) | 0x00010000);
  HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = lsb_addr; //d. LSB testdin
  HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000000;//e. set testclk low
  HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00000000;//b. testen_low

  //3.
  HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = write_data; //a.
  HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000002; //b. testclk high

  //turn off for clean exit
  HW_REG_WORD(mipi_dsi_base_addr,0x00b4) = 0x00000000;//a. testclk
  HW_REG_WORD(mipi_dsi_base_addr,0x00b8) = 0x00000000;//a. testen
}

void DCSW1_S (uint8_t cmd, uint8_t data)
{

  uint32_t rd_data;

  HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR)=0x00000015|cmd<<8|data<<16;

  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR);
  MIPI_PRINTF("EXPSLV1 A_GEN_HDR 0x%0x \n", rd_data);

  //do {
  //  for(int i=0;i<5000;i++);
  //  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_PHY_STATUS);
  //  MIPI_PRINTF("MIPI DSI DPHY Host Ctrl Read A_PHY_STATUS 0x%0x \n", rd_data);
  //} while( (rd_data & 0x00000010) != 0x00000010);

}

void DCSWN_S (uint8_t cmd)
{

  uint32_t rd_data;

  HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR)=0x00000015|cmd<<8;

  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR);
  MIPI_PRINTF("EXPSLV1 A_GEN_HDR 0x%0x \n", rd_data);

  //do {
  //  for(int i=0;i<5000;i++);
  //  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_PHY_STATUS);
  //  MIPI_PRINTF("MIPI DSI DPHY Host Ctrl Read A_PHY_STATUS 0x%0x \n", rd_data);
  //} while( (rd_data & 0x00000010) != 0x00000010);

}

void DCSW_L (uint8_t cmd, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4)
{

  uint32_t rd_data;

  HW_REG_WORD((mipi_dsi_base_addr),A_GEN_PLD_DATA)=data2<<24|data1<<16|cmd<<8|cmd;

  HW_REG_WORD((mipi_dsi_base_addr),A_GEN_PLD_DATA)=0x0|data4<<8|data3;

  HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR)=0x00000039|0x6<<8;

  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR);
  MIPI_PRINTF("EXPSLV1 A_GEN_HDR 0x%0x \n", rd_data);

  //do {
  //  for(int i=0;i<5000;i++);
  //  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_PHY_STATUS);
  //  MIPI_PRINTF("MIPI DSI DPHY Host Ctrl Read A_PHY_STATUS 0x%0x \n", rd_data);
  //} while( (rd_data & 0x00000010) != 0x00000010);

}

void SMRPS_S(uint8_t byte)
{
  uint32_t rd_data;

  HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR)=0x00000037|byte<<8;

  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR);
  MIPI_PRINTF("EXPSLV1 A_GEN_HDR 0x%0x \n", rd_data);

  //do {
  //  for(int i=0;i<5000;i++);
  //  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_PHY_STATUS);
  //  MIPI_PRINTF("MIPI DSI DPHY Host Ctrl Read A_PHY_STATUS 0x%0x \n", rd_data);
  //} while( (rd_data & 0x00000010) != 0x00000010);

}

void DCSRR1_S (uint8_t cmd)
{
  uint32_t rd_data;

  //06 DCS Read, No Parameter SPa (Short Packet) DCSRN-S
  //Read Display Power Mode (0Ah)
  HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR)=0x00000006|cmd<<8;

  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR);
  MIPI_PRINTF("EXPSLV1 A_GEN_HDR 0x%0x \n", rd_data);

  //do {
  //  for(int i=0;i<5000;i++);
  //  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_PHY_STATUS);
  //  MIPI_PRINTF("MIPI DSI DPHY Host Ctrl Read A_PHY_STATUS 0x%0x \n", rd_data);
  //} while( (rd_data & 0x00000010) != 0x00000010);

}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
