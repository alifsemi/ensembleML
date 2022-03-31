


#include <stdio.h>
#include <stdint.h>
#include "global_map.h"
#include "base_def.h"
#include "mipi_dsi_host.h"
#include "delay.h"

#define mipi_dsi_base_addr 0x49032000

// video format
 uint16_t HSYNC = 0x40; //64
 uint16_t VSYNC = 0xA;  //10
 uint16_t HBP   = 0x5E; //94
 uint16_t VBP   = 0x14; //20
 uint16_t HACT  = 0x1E0; //480
 uint16_t VACT  = 0x320; //800
 uint16_t HFP   = 0x48B; //adjust this to sync up DSI host controller
 uint16_t VFP   = 0x14; //20

 static void DSI_DPHY_REG_WRITE (int,uint8_t);
 static uint8_t DSI_DPHY_REG_READ (int);

void dsi_command_mode_initialization(void)
{
	int rd_data;
	DEBUG_PRINTF("MIPIDSI Video Mode Pattern Generator \n");

	//LPCLK_CTR
	//This register configures the possibility for using non continuous clock in the clock lane.
	//1 auto_clklane_ctrl R/W This bit enables the automatic mechanism to stop providing clock in the clock lane when time allows.
	//0 phy_txrequestclkhs R/W This bit controls the D-PHY PPI txrequestclkhs signal.
	//from synopsys recommandation
	rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x94);
	DEBUG_PRINTF("EXPSLV1 0x94 0x%0x \n", rd_data);

	HW_REG_WORD((mipi_dsi_base_addr),0x94)=0x00000002;

	rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x94);
	DEBUG_PRINTF("EXPSLV1 0x94 0x%0x \n", rd_data);

	HW_REG_WORD((mipi_dsi_base_addr),0x18)=0x000a000a;

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
	DEBUG_PRINTF("EXPSLV1 0x98 0x%0x \n", rd_data);

	HW_REG_WORD((mipi_dsi_base_addr),0x98)=0x42<<16|0xB5;

	rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x98);
	DEBUG_PRINTF("EXPSLV1 0x98 0x%0x \n", rd_data);

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
	DEBUG_PRINTF("EXPSLV1 0x9C 0x%0x \n", rd_data);

	HW_REG_WORD((mipi_dsi_base_addr),0x9C)=0x32<<16|0x99;

	rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x9C);
	DEBUG_PRINTF("EXPSLV1 0x9C 0x%0x \n", rd_data);

	//This register configures the polarity of DPI signals.
	//0 dataen_active_low
	//1 vsync_active_low
	//2 hsync_active_low
	//rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x14);
	//DEBUG_PRINTF("EXPSLV1 0x14 0x%0x \n", rd_data);

	//HW_REG_WORD((mipi_dsi_base_addr),0x14)=0x00000001;

	//rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x14);
	//DEBUG_PRINTF("EXPSLV1 0x14 0x%0x \n", rd_data);

	//1. configure MODE_CFG register to enable command mode
	rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x34);
	DEBUG_PRINTF("EXPSLV1 0x34 0x%0x \n", rd_data);

	HW_REG_WORD((mipi_dsi_base_addr),0x34)=0x00000001|rd_data;

	rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x34);
	DEBUG_PRINTF("EXPSLV1 0x34 0x%0x \n", rd_data);

	//1.1 configure Vdieo Mode type VID_MODE_CFG[1:0]
	//1:0 vid_mode_type R/W This field indicates the video mode transmission type
	//15 lp_cmd_en R/W When set to 1, this bit enables the command transmission only in low-power mode.
	rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x38);
	DEBUG_PRINTF("EXPSLV1 0x38 0x%0x \n", rd_data);

	HW_REG_WORD((mipi_dsi_base_addr),0x38)=0x00008002|rd_data; //burst mode

	rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x38);
	DEBUG_PRINTF("EXPSLV1 0x38 0x%0x \n", rd_data);

	//The following are added for ESC mode LPDT
	//CLKMGR_CFG
	//7:0 tx_esc_clk_division R/W
	//  This field indicates the division factor for the TX Escape clock source (lanebyteclk).
	//  The values 0 and 1 stop the TX_ESC clock generation.
	rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_CLKMGR_CFG);
	DEBUG_PRINTF("EXPSLV1 A_CLKMGR_CFG 0x%0x \n", rd_data);

	HW_REG_WORD((mipi_dsi_base_addr),A_CLKMGR_CFG)=0x00000002|rd_data;

	rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_CLKMGR_CFG);
	DEBUG_PRINTF("EXPSLV1 A_CLKMGR_CFG 0x%0x \n", rd_data);

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
	DEBUG_PRINTF("EXPSLV1 A_CMD_MODE_CFG 0x%0x \n", rd_data);
	//low power = 1; high speed = 0;
	HW_REG_WORD((mipi_dsi_base_addr),A_CMD_MODE_CFG)=0x010F7F00|rd_data;

	rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_CMD_MODE_CFG);
	DEBUG_PRINTF("EXPSLV1 A_CMD_MODE_CFG 0x%0x \n", rd_data);

}

void dsi_video_mode_initialization(void)
{
	int rd_data;

	  //This register configures the possibility for using non continuous clock in the clock lane.
	  //0 phy_txrequestclkhs R/W This bit controls the D-PHY PPI txrequestclkhs signal.
	  //from synopsys recommandation
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x94);
	  DEBUG_PRINTF("EXPSLV1 0x94 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),0x94)=0x00000001|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x94);
	  DEBUG_PRINTF("EXPSLV1 0x94 0x%0x \n", rd_data);

	  //1. configure MODE_CFG register to enable command mode
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x34);
	  DEBUG_PRINTF("EXPSLV1 0x34 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),0x34)=0x00000000&rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x34);
	  DEBUG_PRINTF("EXPSLV1 0x34 0x%0x \n", rd_data);

	  //2. configure DPI_COLOR_CODING register
	  // 0x5 (CC05): 24-bit
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x10);
	  DEBUG_PRINTF("EXPSLV1 0x10 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),0x10)=0x00000005|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x10);
	  DEBUG_PRINTF("EXPSLV1 0x10 0x%0x \n", rd_data);

	  //2. DPI_CFG_POL - This register configures the polarity of DPI signals.
	  // 0x1 vsync_active_low
	  // 0x2 hsync_active_low
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x14);
	  DEBUG_PRINTF("EXPSLV1 0x14 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),0x14)=0x00000006|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x14);
	  DEBUG_PRINTF("EXPSLV1 0x14 0x%0x \n", rd_data);

	  //3. vid_pkt_size
	  // pixel perline
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x3C);
	  DEBUG_PRINTF("EXPSLV1 0x3C 0x%0x \n", rd_data);

	  //HW_REG_WORD((mipi_dsi_base_addr),0x3c)=0x000001EC|rd_data; //480+12
	  //HW_REG_WORD((mipi_dsi_base_addr),0x3c)=0x000001E0|rd_data;
	  HW_REG_WORD((mipi_dsi_base_addr),0x3c)=HACT|rd_data; //active video 480

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x3C);
	  DEBUG_PRINTF("EXPSLV1 0x3C 0x%0x \n", rd_data);

	  //4. vid_num_chunks
	  // chunks per line
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x40);
	  DEBUG_PRINTF("EXPSLV1 0x40 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),0x40)=0x00000000|rd_data; //0 - no chunks

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x40);
	  DEBUG_PRINTF("EXPSLV1 0x40 0x%0x \n", rd_data);

	  //5. null packet
	  //12:0 vid_null_size R/W This register configures the number of bytes inside a null packet. Setting to 0 disables null packets.
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x44);
	  DEBUG_PRINTF("EXPSLV1 0x44 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),0x44)=0x00000000|rd_data; //disable

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x44);
	  DEBUG_PRINTF("EXPSLV1 0x44 0x%0x \n", rd_data);

	  //6. This register configures the video HSA time.
	  //11:0 vid_hsa_time R/W This field configures the Horizontal Synchronism Active period in lane byte clock cycles.
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x48);
	  DEBUG_PRINTF("EXPSLV1 0x48 0x%0x \n", rd_data);

	  //HSA 64 bits
	  //HW_REG_WORD((mipi_dsi_base_addr),0x48)=0x00000040|rd_data;
	  HW_REG_WORD((mipi_dsi_base_addr),0x48)=HSYNC|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x48);
	  DEBUG_PRINTF("EXPSLV1 0x48 0x%0x \n", rd_data);

	  //7. This register configures the video HBP time.
	  //11:0 vid_hbp_time R/W This field configures the Horizontal Back Porch period in lane byte clock cycles
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x4C);
	  DEBUG_PRINTF("EXPSLV1 0x4C 0x%0x \n", rd_data);

	  //HBP 94
	  //HW_REG_WORD((mipi_dsi_base_addr),0x4C)=0x0000005e|rd_data;
	  HW_REG_WORD((mipi_dsi_base_addr),0x4C)=HBP|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x4C);
	  DEBUG_PRINTF("EXPSLV1 0x4C 0x%0x \n", rd_data);

	  //8. This register configures the overall time for each video line.
	  //14:0 vid_hline_time R/W This field configures the size of the total line time (HSA+HBP+HACT+HFP) counted in lane byte clock cycles.
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x50);
	  DEBUG_PRINTF("EXPSLV1 0x50 0x%0x \n", rd_data);

	  //HSD = 1688 (HSA+HBP+Hactive)
	  HW_REG_WORD((mipi_dsi_base_addr),0x50)=0x00000698|rd_data; //added big delay since clock is fast and we are in burst mode
	  //HW_REG_WORD((mipi_dsi_base_addr),0x50)=0x000001E0|rd_data; //480+2+5+5

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x50);
	  DEBUG_PRINTF("EXPSLV1 0x50 0x%0x \n", rd_data);

	  //9. This register configures the VSA period.
	  //9:0 vsa_lines R/W This field configures the Vertical Synchronism Active period measured in number of horizontal lines.
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x54);
	  DEBUG_PRINTF("EXPSLV1 0x54 0x%0x \n", rd_data);

	  //HW_REG_WORD((mipi_dsi_base_addr),0x54)=0x0000000a|rd_data;
	  HW_REG_WORD((mipi_dsi_base_addr),0x54)=VSYNC|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x54);
	  DEBUG_PRINTF("EXPSLV1 0x54 0x%0x \n", rd_data);

	  //10. This register configures the VBP period.
	  //9:0 vbp_lines R/W This field configures the Vertical Back Porch period measured in number of horizontal lines.
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x58);
	  DEBUG_PRINTF("EXPSLV1 0x58 0x%0x \n", rd_data);

	  //HW_REG_WORD((mipi_dsi_base_addr),0x58)=0x00000014|rd_data;
	  HW_REG_WORD((mipi_dsi_base_addr),0x58)=VBP|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x58);
	  DEBUG_PRINTF("EXPSLV1 0x58 0x%0x \n", rd_data);

	  //11. This register configures the VFP period.
	  //9:0 vfp_lines R/W This field configures the Vertical Front Porch period measured in number of horizontal lines.
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x5C);
	  DEBUG_PRINTF("EXPSLV1 0x5C 0x%0x \n", rd_data);

	  //HW_REG_WORD((mipi_dsi_base_addr),0x5C)=0x000000a|rd_data;
	  HW_REG_WORD((mipi_dsi_base_addr),0x5C)=VFP|rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x5C);
	  DEBUG_PRINTF("EXPSLV1 0x5C 0x%0x \n", rd_data);

	  //12. This register configures the vertical resolution of video.
	  //13:0 v_active_lines R/W This field configures the Vertical Active period measured in number of horizontal lines.
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x60);
	  DEBUG_PRINTF("EXPSLV1 0x60 0x%0x \n", rd_data);

	  //HW_REG_WORD((mipi_dsi_base_addr),0x60)=0x00000320|rd_data; //854
	  HW_REG_WORD((mipi_dsi_base_addr),0x60)=VACT|rd_data; //800

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x60);
	  DEBUG_PRINTF("EXPSLV1 0x60 0x%0x \n", rd_data);

	  //1. configure MODE_CFG register to enable video mode
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x34);
	  DEBUG_PRINTF("EXPSLV1 0x34 0x%0x \n", rd_data);

	  HW_REG_WORD((mipi_dsi_base_addr),0x34)=0x00000000&rd_data;

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x34);
	  DEBUG_PRINTF("EXPSLV1 0x34 0x%0x \n", rd_data);

	  //13. configure Vdieo Mode type VID_MODE_CFG[1:0]
	  //24 vpg_orientation R/W This field indicates the color bar orientation as follows: 0x0 (VPGORENT0): Vertical mode
	  //20 vpg_mode R/W This field is to select the pattern: 0x0 (COLORBAR): horizontal or vertical
	  //16 vpg_en R/W When set to 1, this bit enables the video mode pattern generator.
	  //15 lp_cmd_en R/W When set to 1, this bit enables the command transmission only in low-power mode.
	  //1:0 vid_mode_type R/W This field indicates the video mode transmission type
	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x38);
	  DEBUG_PRINTF("EXPSLV1 0x38 0x%0x \n", rd_data);

	  rd_data = rd_data & ~0x8000; //disable bit 15
	  //HW_REG_WORD((mipi_dsi_base_addr),0x38)=rd_data; //burst mode, vpg_en = 0
	  HW_REG_WORD((mipi_dsi_base_addr),0x38)=0x00000000|rd_data; //burst mode

	  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x38);
	  DEBUG_PRINTF("EXPSLV1 0x38 0x%0x \n", rd_data);

}


void dsi_reset(void)
{
	int rd_data;

	//0. This register controls the power up of the controller.
	//0 shutdownz R/W This bit configures the controller either to power up or to reset
	rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x04);
	DEBUG_PRINTF("EXPSLV1 0x04 0x%0x \n", rd_data);

	HW_REG_WORD((mipi_dsi_base_addr),0x04) = rd_data & ~(1U << 0);

	rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x04);
	DEBUG_PRINTF("EXPSLV1 0x04 0x%0x \n", rd_data);
}

void dsi_powerup(void)
{
	int rd_data;

	//0. This register controls the power up of the controller.
	//0 shutdownz R/W This bit configures the controller either to power up or to reset
	rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x04);
	DEBUG_PRINTF("EXPSLV1 0x04 0x%0x \n", rd_data);

	HW_REG_WORD((mipi_dsi_base_addr),0x04) = rd_data | 1U;

	rd_data = HW_REG_WORD((mipi_dsi_base_addr),0x04);
	DEBUG_PRINTF("EXPSLV1 0x04 0x%0x \n", rd_data);
}

void DCSW1_S (uint8_t cmd, uint8_t data)
{

  uint32_t rd_data;

  HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR)=0x00000015|cmd<<8|data<<16;

  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR);
  DEBUG_PRINTF("EXPSLV1 A_GEN_HDR 0x%0x \n", rd_data);
  sleep_or_wait_msec(1);

  //do {
  //  for(int i=0;i<5000;i++);
  //  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_PHY_STATUS);
  //  DEBUG_PRINTF("MIPI DSI DPHY Host Ctrl Read A_PHY_STATUS 0x%0x \n", rd_data);
  //} while( (rd_data & 0x00000010) != 0x00000010);

}

void DCSWN_S (uint8_t cmd)
{

  uint32_t rd_data;

  HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR)=0x00000015|cmd<<8;

  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR);
  DEBUG_PRINTF("EXPSLV1 A_GEN_HDR 0x%0x \n", rd_data);
  sleep_or_wait_msec(1);

  //do {
  //  for(int i=0;i<5000;i++);
  //  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_PHY_STATUS);
  //  DEBUG_PRINTF("MIPI DSI DPHY Host Ctrl Read A_PHY_STATUS 0x%0x \n", rd_data);
  //} while( (rd_data & 0x00000010) != 0x00000010);

}

void DCSW_L (uint8_t cmd, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4)
{

  uint32_t rd_data;

  HW_REG_WORD((mipi_dsi_base_addr),A_GEN_PLD_DATA)=data2<<24|data1<<16|cmd<<8|cmd;

  HW_REG_WORD((mipi_dsi_base_addr),A_GEN_PLD_DATA)=0x0|data4<<8|data3;

  HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR)=0x00000039|0x6<<8;

  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR);
  DEBUG_PRINTF("EXPSLV1 A_GEN_HDR 0x%0x \n", rd_data);
  sleep_or_wait_msec(1);

  //do {
  //  for(int i=0;i<5000;i++);
  //  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_PHY_STATUS);
  //  DEBUG_PRINTF("MIPI DSI DPHY Host Ctrl Read A_PHY_STATUS 0x%0x \n", rd_data);
  //} while( (rd_data & 0x00000010) != 0x00000010);

}

void SMRPS_S(uint8_t byte)
{
  uint32_t rd_data;

  HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR)=0x00000037|byte<<8;

  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR);
  DEBUG_PRINTF("EXPSLV1 A_GEN_HDR 0x%0x \n", rd_data);
  sleep_or_wait_msec(1);

  //do {
  //  for(int i=0;i<5000;i++);
  //  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_PHY_STATUS);
  //  DEBUG_PRINTF("MIPI DSI DPHY Host Ctrl Read A_PHY_STATUS 0x%0x \n", rd_data);
  //} while( (rd_data & 0x00000010) != 0x00000010);

}

void DCSRR1_S (uint8_t cmd)
{
  uint32_t rd_data;

  //06 DCS Read, No Parameter SPa (Short Packet) DCSRN-S
  //Read Display Power Mode (0Ah)
  HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR)=0x00000006|cmd<<8;

  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_GEN_HDR);
  DEBUG_PRINTF("EXPSLV1 A_GEN_HDR 0x%0x \n", rd_data);
  sleep_or_wait_msec(1);
  //do {
  //  for(int i=0;i<5000;i++);
  //  rd_data = HW_REG_WORD((mipi_dsi_base_addr),A_PHY_STATUS);
  //  DEBUG_PRINTF("MIPI DSI DPHY Host Ctrl Read A_PHY_STATUS 0x%0x \n", rd_data);
  //} while( (rd_data & 0x00000010) != 0x00000010);

}

int tx_phyconfig(void)
{
	int rd_data =0,exp_data =0,err=0;
	uint8_t r_data_8;

#ifdef TX_FREQ_50M
  int hsfreqrange = 0x30;
  int pll_soc_m_7_0 = 0x4d;
  int pll_soc_m_9_8 = 0x1;
  int pll_soc_n = 0x2;
  uint8_t vco_cntrl = 0x28; //101000
  int below_450Mbps = 1;
  int pixel_clk_div = 0x4;
#elif defined(TX_FREQ_100M)
  //int hsfreqrange = 0x14;
  //int pll_soc_m_7_0 = 0x77;
  //int pll_soc_m_9_8 = 0x1;
  //int pll_soc_n = 0x3;
  ////uint8_t vco_cntrl = 0x1B; //011011
  //uint8_t vco_cntrl = 0x1F; //011111
  //int below_450Mbps = 1;
  int hsfreqrange = 0x14;
  int pll_soc_m_7_0 = 0xFA;
  int pll_soc_m_9_8 = 0x0;
  int pll_soc_n = 0x2;
  uint8_t vco_cntrl = 0x1B; //011011
  int below_450Mbps = 1;
  int pixel_clk_div = 0x4;
#elif defined(TX_FREQ_125M)
  int hsfreqrange = 0x33;
  int pll_soc_m_7_0 = 0x71;
  int pll_soc_m_9_8 = 0x2;
  int pll_soc_n = 0x3;
  uint8_t vco_cntrl = 0x1F; //101000
  int below_450Mbps = 1;
  int pixel_clk_div = 0x8;
#elif defined(TX_FREQ_150M)
  //int hsfreqrange = 0x14;
  //int pll_soc_m_7_0 = 0x77;
  //int pll_soc_m_9_8 = 0x1;
  //int pll_soc_n = 0x3;
  ////uint8_t vco_cntrl = 0x1B; //011011
  //uint8_t vco_cntrl = 0x1F; //011111
  //int below_450Mbps = 1;
  int hsfreqrange = 0x14;
  int pll_soc_m_7_0 = 0x77;
  int pll_soc_m_9_8 = 0x1;
  int pll_soc_n = 0x2;
  uint8_t vco_cntrl = 0x1B; //011011
  int below_450Mbps = 1;
  int pixel_clk_div = 0x8;
#elif defined(TX_FREQ_200M)
  int hsfreqrange = 0x5;
  int pll_soc_m_7_0 = 0xF4;
  int pll_soc_m_9_8 = 0x1;
  int pll_soc_n = 0x2;
  uint8_t vco_cntrl = 0x18; //0110000
  int below_450Mbps = 0;
  int pixel_clk_div = 0x4;
#elif defined(TX_FREQ_250M)
  int hsfreqrange = 0x26;
  int pll_soc_m_7_0 = 0x38;
  int pll_soc_m_9_8 = 0x1;
  int pll_soc_n = 0x2;
  uint8_t vco_cntrl = 0x17; //010111
  int below_450Mbps = 1;
  int pixel_clk_div = 0x4;
#elif defined(TX_FREQ_371M)
  int hsfreqrange = 0x39;
  int pll_soc_m_7_0 = 0xCF;
  int pll_soc_m_9_8 = 0x1;
  int pll_soc_n = 0x2;
  uint8_t vco_cntrl = 0x10; //010000
  int below_450Mbps = 1;
  int pixel_clk_div = 0x4;
#elif defined(TX_FREQ_400M)
  int hsfreqrange = 0x9;
  int pll_soc_m_7_0 = 0xF4;
  int pll_soc_m_9_8 = 0x1;
  int pll_soc_n = 0x2;
  uint8_t vco_cntrl = 0x10; //010000
  int below_450Mbps = 0;
  int pixel_clk_div = 0x4;
#else //1G
  //int hsfreqrange = 0xA;
  //int pll_soc_m_7_0 = 0xA1;
  //int pll_soc_m_9_8 = 0x1;
  //int pll_soc_n = 0x2;
  //uint8_t vco_cntrl = 0xF; //001111
  //int below_450Mbps = 0;
  int hsfreqrange = 0xA;
  int pll_soc_m_7_0 = 0x38;
  int pll_soc_m_9_8 = 0x1;
  int pll_soc_n = 0x2;
  uint8_t vco_cntrl = 0xF; //001111
  int below_450Mbps = 0;
  int pixel_clk_div = 0x4;

#endif

////////////////////////////////////////////////////////////////
//DSI DPHY setup as master
////////////////////////////////////////////////////////////////

//cdc200_pixclk_ctrl.clk_ena = 0x0
//cdc200_pixclk_ctrl.clk_divisor = 0x1ff
//HW_REG_WORD(CFGSLV1_BASE,0x04) =  0x00040001;//set cdc200 pix clock//sub system 4
//HW_REG_WORD(CFGSLV1_BASE,0x04) =  0x00000001 | (pixel_clk_div<<16);//set cdc200 pix clock//sub system 8 = 50Mhz

//number of lines
HW_REG_WORD((mipi_dsi_base_addr),0xa4)=0x00000001;

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
HW_REG_WORD(CFGSLV1_BASE,0x30) =  0x00000100;

//4. Set rx_testclr and tx_testclr = 1'b1;
//4.1 set tx_testclr
//Just toggle tx_testclr
HW_REG_WORD(mipi_dsi_base_addr,0xb4) = 0x00000000;//test_clr=0
HW_REG_WORD(mipi_dsi_base_addr,0xb4) = 0x00000001;//test_clr=1
HW_REG_WORD(mipi_dsi_base_addr,0xb4) = 0x00000000;//test_clr=0

//testport_sel [4]	RW	0x0	testport select; 0: select tx_testport; 1: select rx_testport	( rx_dphy_ctrl0.testport_sel = 0x0 )
HW_REG_WORD(CFGSLV1_BASE,0x30) =  0x00000110;//set csi pix clock
//4.2 set rx_test_clr
//Just toggle rx_testclr
HW_REG_WORD(mipi_dsi_base_addr,0xb4) = 0x00000000;//test_clr=0
HW_REG_WORD(mipi_dsi_base_addr,0xb4) = 0x00000001;//test_clr=1
HW_REG_WORD(mipi_dsi_base_addr,0xb4) = 0x00000000;//test_clr=0

//5. Wait for 15 ns;
//for(int i=0;i<5;i++);

//switch back to tx_testport
HW_REG_WORD(CFGSLV1_BASE,0x30) =  0x00000100;

//check DSI DPHY state
//dphy4txtester_DIG_RD_TX_SYS_0
r_data_8 = DSI_DPHY_REG_READ(0x1E);
DEBUG_PRINTF("MIPI DSI DPHY Read 1E 0x%0x \n", r_data_8);

//7. Set hsfreqrange[6:0] = 7'b0001010;
//[22:16] - 1G = hsfreqrange[6:0] = 0x0A
//[31:24] - cfgclkfreqrange[7:0] = round[(Fcfg_clk(MHz)-17)*4] = (25-17)*4 = 8'b001000000
//cfgclk = tb_top.DUT.u_sse700.u_sse700_f0_expslv1.mipi_tx_dphy_cfg_clk = 25MHz

//7.1 TX
rd_data = HW_REG_WORD((CFGSLV1_BASE),0x30);
DEBUG_PRINTF("EXPSLV1 0x30 0x%0x \n", rd_data);
HW_REG_WORD((CFGSLV1_BASE),0x30)=0x20000100|(hsfreqrange<<16);
rd_data = HW_REG_WORD((CFGSLV1_BASE),0x30);
DEBUG_PRINTF("EXPSLV1 0x30 0x%0x \n", rd_data);


//check hsfreqrange[6:0] in DPHY
//dphy4rxtester_DIG_RD_RX_SYS_1
r_data_8 = DSI_DPHY_REG_READ(0x1F);
DEBUG_PRINTF("MIPI DSI DPHY Read 1F 0x%0x \n", r_data_8);

//8. Configure TX register 0x16A to set pll_mpll_prog_rw (bits1:0) to 2'b11.
r_data_8 = DSI_DPHY_REG_READ(0x16A);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x16A 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x16A,0x03);

r_data_8 = DSI_DPHY_REG_READ(0x16A);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x16A 0x%0x \n", r_data_8);

//9. Configure TX register 0x1AB to set cb_sel_vref_lprx_rw (bits 1:0) to 2'b10.
r_data_8 = DSI_DPHY_REG_READ(0x1AB);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x1AB 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x1AB,0x06);

r_data_8 = DSI_DPHY_REG_READ(0x1AB);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x1AB 0x%0x \n", r_data_8);

//10. Configure TX register 0x1AA to set cb_sel_vrefcd_lprx_rw (bits 6:5) to 2'b10.
r_data_8 = DSI_DPHY_REG_READ(0x1AA);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x1AA 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x1AA,0x53);

r_data_8 = DSI_DPHY_REG_READ(0x1AA);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x1AA 0x%0x \n", r_data_8);

//When operating as master or when in High-Speed BIST modes,
//for datarates below 450 Mbps, clkdiv_clk_en must be enabled.
//To do this, write 1'b1 in TX configuration register with address 0x1AC bit [4].
r_data_8 = DSI_DPHY_REG_READ(0x1AC);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x1AC 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x1AC,(r_data_8|below_450Mbps<<4));

r_data_8 = DSI_DPHY_REG_READ(0x1AC);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x1AC 0x%0x \n", r_data_8);

//11. Configure TX register 0x402 to set txclk_term_lowcap_lp00_en_ovr_en_rw and
//txclk_term_lowcap_lp00_en_ovr_rw(bits 1:0) to 2'b10;
r_data_8 = DSI_DPHY_REG_READ(0x402);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x402 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x402,0x2);

r_data_8 = DSI_DPHY_REG_READ(0x402);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x402 0x%0x \n", r_data_8);

//12. Refer to table ??Supported rise/fall time limits?? on page 114 and configure TX test control registers
//with appropriate values for the specified rise/fall time.
//500 Mbps and ?? 1 Gbps 1 ns 150 ps - 300 ps 1 12??d657 225 - 291
//dphy4txtester_DIG_RDWR_TX_SLEW_5
r_data_8 = DSI_DPHY_REG_READ(0x270);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x270 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x270,0x91);

r_data_8 = DSI_DPHY_REG_READ(0x270);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x270 0x%0x \n", r_data_8);

//dphy4txtester_DIG_RDWR_TX_SLEW_6
r_data_8 = DSI_DPHY_REG_READ(0x271);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x271 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x271,0x2);

r_data_8 = DSI_DPHY_REG_READ(0x271);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x271 0x%0x \n", r_data_8);

//dphy4txtester_DIG_RDWR_TX_SLEW_7
//13. Set bits [5:4] of TX control register with address 0x272 to 2'b01 to enable slew rate calibration. (Lanes
//that are disabled should not run slew-rate calibration algorithm. For the disabled lanes, set
//sr_finished_ovr_en = 1??b1, sr_finished_ovr = 1??b1, srcal_en_ovr_en = 1??b1 through test control
//registers 0x310, 0x50b, 0x70b);
r_data_8 = DSI_DPHY_REG_READ(0x272);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x272 0x%0x \n", r_data_8);

//DSI_DPHY_REG_WRITE(0x272,0x10);
DSI_DPHY_REG_WRITE(0x272,0x11); //sr_range = 1 from Saritha

r_data_8 = DSI_DPHY_REG_READ(0x272);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x272 0x%0x \n", r_data_8);

//For the disabled lanes, set
//sr_finished_ovr_en = 1??b1, sr_finished_ovr = 1??b1, srcal_en_ovr_en = 1??b1 through test control registers 0x310, 0x50b, 0x70b, 0x90b, 0xbob);
//bit2, bit1, bit0 are given as "reserved" in the Databook; but program as per the requirement.
//Since you are not using lane2, lane3 - you can disable those bits in the registers for lane2 and lane3.

//dphy4txtester_DIG_RDWR_TX_LANE2_SLEWRATE_0
//lane 2
r_data_8 = DSI_DPHY_REG_READ(0x90b);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x90b 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x90b,0x0e); //Saritha

r_data_8 = DSI_DPHY_REG_READ(0x90b);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x90b 0x%0x \n", r_data_8);

//lane 3
r_data_8 = DSI_DPHY_REG_READ(0xb0b);
DEBUG_PRINTF("MIPI DSI DPHY Read 0xb0b 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0xb0b,0x0e); //Saritha

r_data_8 = DSI_DPHY_REG_READ(0xb0b);
DEBUG_PRINTF("MIPI DSI DPHY Read 0xb0b 0x%0x \n", r_data_8);

//14. Set cfgclkfreqrange[7:0] = round[(Fcfg_clk(MHz)-17)*4] = 8'b00101000;
//already set
//15. Apply cfg_clk signal with the appropriate frequency with 25 Mhz frequency;
//16. Configure PLL operating frequency through D-PHY test control registers or through PLL SoC
DEBUG_PRINTF("MIPI PLL task started....\n");

//pll_soc_clksel [21:20]	RW	0x0	clkext div selection
//should be set to 2'b01
rd_data = HW_REG_WORD(CFGSLV1_BASE,0x10);
rd_data |= 1UL << 20;
HW_REG_WORD(CFGSLV1_BASE,0x10) = rd_data;

//When the PHY is configured as a master (tx_rxz=1'b1) the PLL needs to always be properly configured for
//the desired operating frequency before D-PHY Start-up.
//Use pll shadow control
//pll_soc_shadow_control [4]	RW	0x0	Selection of PLL configuration mechanism	( dphy_pll_ctr
//rd_data = HW_REG_WORD(CFGSLV1_BASE,0x10);
//rd_data = rd_data | 1 << 4;
//HW_REG_WORD(CFGSLV1_BASE,0x10) = rd_data;

//pll soc shadow control set
rd_data = HW_REG_WORD(CFGSLV1_BASE,0x10);
rd_data |= 1UL << 4;
HW_REG_WORD(CFGSLV1_BASE,0x10) = rd_data;

//m[11:0] - pll_m_ovr_rw[7:0], pll_m_ovr_rw[11:8], pll_m_ovr_en_rw ?? 0x179, 0x17a, 0x17b
//n[3:0] - pll_n_ovr_rw[3:0], pll_n_ovr_en_rw ?? 0x178
//Foutmax [MHz] Foutmin [MHz] F VCO max [MHz] F VCO min [MHz] Output division factor P vco_cntrl[5:3]
//500           250           4000            2000            8                        010
//1000          500           4000            2000            4                        001
//24Mhz > fclkin/N > 8 Mhz
//24Mhz > 38.4Mhz/N > 8 Mhz
//N = 4
//n = 4 - 1

//fout = 500Mhz for 1 G bit rate
//M/N*1/2*fclkin*1/P = M/2*1/2*38.4Mhz*1/8 = 500Mhz = 432 = 0x1B0
//M = 417 = 0x1A1

//MIPI-DPHY PLL Control Register 1
//pll_soc_m [9:0]	RW	0x0	Control of the feedback multiplication ratio M (40 to 625) for SoC direct PLL control	( dphy_pll_ctrl1.pll_soc_m = 0x0 )
//pll_soc_n [15:12]	RW	0x0	Control of the input frequency division ratio N (1 to 16) for SoC direct PLL control	( dphy_pll_ctrl1.pll_soc_n = 0x0 )
//HW_REG_WORD(CFGSLV1_BASE,0x14) = 0x031A1;

//dphy4txtester_DIG_RDWR_TX_PLL_28
//7:0 pll_m_ovr_rw__7__0__ R/W Description: PLL feedback divider override
r_data_8 = DSI_DPHY_REG_READ(0x179);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x179 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x179,pll_soc_m_7_0);

r_data_8 = DSI_DPHY_REG_READ(0x179);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x179 0x%0x \n", r_data_8);

//dphy4txtester_DIG_RDWR_TX_PLL_29
//1:0 pll_m_ovr_rw__9__8__ R/W Description: PLL feedback divider override
r_data_8 = DSI_DPHY_REG_READ(0x17a);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x17a 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x17a,pll_soc_m_9_8);

r_data_8 = DSI_DPHY_REG_READ(0x17a);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x17a 0x%0x \n", r_data_8);

//dphy4txtester_DIG_RDWR_TX_PLL_30
//0 pll_m_ovr_en_rw R/W Description: PLL feedback divider override enable
r_data_8 = DSI_DPHY_REG_READ(0x17b);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x17b 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x17b,0x1);

r_data_8 = DSI_DPHY_REG_READ(0x17b);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x17b 0x%0x \n", r_data_8);

//dphy4txtester_DIG_RDWR_TX_PLL_27
//7 pll_n_ovr_en_rw R/W Description: PLL input divider override enable
//6:3 pll_n_ovr_rw__3__0__ R/W Description: PLL input divider override
r_data_8 = DSI_DPHY_REG_READ(0x178);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x178 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x178,(0x80|(pll_soc_n<<3)));

r_data_8 = DSI_DPHY_REG_READ(0x178);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x178 0x%0x \n", r_data_8);


// vco_cntrl[5:0] - pll_vco_cntrl_ovr_rw[5:0], pll_vco_cntrl_ovr_en_rw ?? 0x17b
//dphy4txtester_DIG_RDWR_TX_PLL_30
//7 pll_vco_cntrl_ovr_en_rw R/W Description: PLL VCO control override enable
//6:1 pll_vco_cntrl_ovr_rw__5__0__ R/W Description: PLL VCO control override
r_data_8 = DSI_DPHY_REG_READ(0x17b);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x17b 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x17b,(0x81|vco_cntrl<<1));

r_data_8 = DSI_DPHY_REG_READ(0x17b);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x17b 0x%0x \n", r_data_8);

// cpbias_cntrl[6:0] - pll_cpbias_cntrl_rw[6:0] ?? 0x15e
// dphy4txtester_DIG_RDWR_TX_PLL_1
// 6:0 pll_cpbias_cntrl_rw__6__0__ R/W Description: PLL Charge Pump Bias Control
r_data_8 = DSI_DPHY_REG_READ(0x15e);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x15e 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x15e,0x0);

r_data_8 = DSI_DPHY_REG_READ(0x15e);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x15e 0x%0x \n", r_data_8);

// gmp_cntrl[1:0] - pll_gmp_cntrl_rw[1:0] ?? 0x162
// int_cntrl[5:0] - pll_int_cntrl_rw[5:0] ?? 0x162
// dphy4txtester_DIG_RDWR_TX_PLL_5
// 7:2 pll_int_cntrl_rw__5__0__ R/W Description: PLL Integral Charge Pump control
// 1:0 pll_gmp_cntrl_rw__1__0__ R/W Description: PLL GMP Control
r_data_8 = DSI_DPHY_REG_READ(0x162);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x162 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x162,0x11);

r_data_8 = DSI_DPHY_REG_READ(0x162);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x162 0x%0x \n", r_data_8);

// prop_cntrl[5:0] - pll_prop_cntrl_rw[5:0] ?? 0x16e
// dphy4txtester_DIG_RDWR_TX_PLL_17
// 5:0 pll_prop_cntrl_rw__5__0__ R/W Description: PLL Proportional Charge Pump control
r_data_8 = DSI_DPHY_REG_READ(0x16e);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x16e 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x16e,0x10);

r_data_8 = DSI_DPHY_REG_READ(0x16e);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x16e 0x%0x \n", r_data_8);

//Output frequency [MHz] vco_cntrl [5:0] cpbias_cntrl [6:0] gmp_cntrl [1:0] int_cntrl[5:0] prop_cntrl[5:0] tx or rx_cb_vref_mpll_re g_rel_rw[2:0]
//487.5-615              001111          0000000            01              000100         010000          010
//pll_soc_cpbias_cntrl [6:0]	RW	0x0	Charge Pump bias control, for SoC direct PLL control	( dphy_pll_ctrl2.pll_soc_cpbias_cntrl = 0x0 )
//pll_soc_int_cntrl [13:8]	RW	0x0	Integral Charge Pump control for SoC direct PLL control	( dphy_pll_ctrl2.pll_soc_int_cntrl = 0x0 )
//pll_soc_prop_cntrl [21:16]	RW	0x0	Proportional Charge Pump control for SoC direct PLL control	( dphy_pll_ctrl2.pll_soc_prop_cntrl = 0x0 )
//pll_soc_vco_cntrl [29:24]	RW	0x0	VCO operating range for SoC direct PLL control	( dphy_pll_ctrl2.pll_soc_vco_cntrl = 0x0 )
//HW_REG_WORD(CFGSLV1_BASE,0x18) = 0x0f100400;

//pll_soc_gmp_cntrl [17:16]	RW	0x0	Controls the effective loop-filter resistance (=1/gmp) to increase/decrease MPLL bandwidth
//rd_data = HW_REG_WORD(CFGSLV1_BASE,0x10);
//rd_data |= 1UL << 16;
//HW_REG_WORD(CFGSLV1_BASE,0x10) = rd_data;

//SNPS: dphy4txtester_DIG_RDWR_TX_CB_3
//2:0 cb_vref_mpll_reg_sel_rw__2__0__R/W Description: PLL reference voltage control
r_data_8 = DSI_DPHY_REG_READ(0x1AD);
DEBUG_PRINTF("MIPI DSI DPHY Read 1AD 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x1AD,0x02);

r_data_8 = DSI_DPHY_REG_READ(0x1AD);
DEBUG_PRINTF("MIPI DSI DPHY Read 1AD 0x%0x \n", r_data_8);

////update PLL pulse
////pll_soc_updatepll [8]	RW	0x0	Control for PLL operation frequency updated
//rd_data = HW_REG_WORD(CFGSLV1_BASE,0x10);
//rd_data |= 1UL << 8;
//HW_REG_WORD(CFGSLV1_BASE,0x10) = rd_data;

////Wait 10 ns;
//for(int i=0;i<10;i++);
//
////pll_soc_updatepll [8]	RW	0x0	Control for PLL operation frequency updated
//rd_data = HW_REG_WORD(CFGSLV1_BASE,0x10);
//rd_data &= ~(1UL << 8);
//HW_REG_WORD(CFGSLV1_BASE,0x10) = rd_data;


//Wait for PLL lock
//do {
//  rd_data = HW_REG_WORD(CFGSLV1_BASE,0x20);
//  DEBUG_PRINTF("PLL Lock wait 0x%0x \n", rd_data);
//} while( (rd_data & 0x00000001) != 0x00000001);

//check DSI DPHY state
//dphy4txtester_DIG_RD_TX_SYS_0
r_data_8 = DSI_DPHY_REG_READ(0x1E);
DEBUG_PRINTF("MIPI DSI DPHY Read 1E 0x%0x \n", r_data_8);

//shadow registers interface (see ??Initialization?? on page 33 for additional details);

//17. Set basedir_n = 1'b0;
//18. Set forcerxmode_n = 1'b0;
HW_REG_WORD((CFGSLV1_BASE),0x34)=0x00000000;

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
DEBUG_PRINTF("MIPI DSI DPHY Read 0x16e 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x16e,0xD0);

r_data_8 = DSI_DPHY_REG_READ(0x16e);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x16e 0x%0x \n", r_data_8);

////force PLL lock
//r_data_8 = DSI_DPHY_REG_READ(0x15F);
//DEBUG_PRINTF("MIPI DSI DPHY Read 0x15F 0x%0x \n", r_data_8);

//DSI_DPHY_REG_WRITE(0x15F,0x01);

//r_data_8 = DSI_DPHY_REG_READ(0x15F);
//DEBUG_PRINTF("MIPI DSI DPHY Read 0x15F 0x%0x \n", r_data_8);

////pll_soc_force_lock [0]
//DEBUG_PRINTF("MIPI DSI dphy_pll_ctrl0 Read 0x10 0x%0x \n", rd_data);
//rd_data = HW_REG_WORD(CFGSLV1_BASE,0x10);
//rd_data |= 1UL << 0;
//HW_REG_WORD(CFGSLV1_BASE,0x10) = rd_data;
//DEBUG_PRINTF("MIPI DSI dphy_pll_ctrl0 Read 0x10 0x%0x \n", rd_data);

static volatile int retry = 100;
//26. Wait until stopstatedata_n and stopstateclk outputs are asserted indicating PHY is driving LP11 in
//enabled datalanes and clocklane.
do {
  //dphy4rxtester_DIG_RD_RX_SYS_0 0x1e
  //suggested by synopsys for PLL checking
  r_data_8 = DSI_DPHY_REG_READ(0x1e);
  DEBUG_PRINTF("MIPI DSI DPHY Read 0x1e 0x%0x \n", r_data_8);
  //dphy4txtester_DIG_RD_TX_PLL_0
  //7 pll_vpl_det R Description: Supply presence detector (volatile)
  //6 pll_clksel_en R Description: PHY internal clksel override (for lane startup sequence) (volatile)
  //5 pll_lock R Description: PLL lock observability (volatile)
  //4 pll_lock_det_on R Description: PLL lock detector power-on (volatile)
  //3 pll_gear_shift R Description: PLL gear shift (volatile)
  //2:1 pll_clksel_in__1__0__ R Description: PLL clksel[1:0] output clock selection (volatile)
  //0 onpll R Description: PLL power-on (volatile)
  r_data_8 = DSI_DPHY_REG_READ(0x191);
  DEBUG_PRINTF("MIPI DSI DPHY Read 0x191 0x%0x \n", r_data_8);
  //for(int i=0;i<5000;i++);
  sleep_or_wait_msec(1);
  rd_data = HW_REG_WORD((mipi_dsi_base_addr),0xB0);
  DEBUG_PRINTF("MIPI DSI DPHY Host Ctrl Read 0xB0 0x%0x \n", rd_data);

  if (--retry <= 0)
	  return -1;

} while( (rd_data & 0x00000094) != 0x00000094);

/*
for(int i=0;i<50000;i++);

//dphy4txtester_DIG_RD_TX_PLL_4
//1. For each reading, make sure the PHY is in an active mode;
//2. Set bits [1:0] of TX test control register with address 0x1AC to 2??b00 (cb_atb_sel=2'b00);
//dphy4txtester_DIG_RDWR_TX_CB_2
//2 cb_atb_ext_con_rw R/W Description: allows direct connecting ATB line into ATB pin:
//1:0 cb_atb_sel_rw__1__0__ R/W Description: Analog Test Bus selection:
//0 : DAC
r_data_8 = DSI_DPHY_REG_READ(0x1AC);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x1AC 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x1AC,0x08);

r_data_8 = DSI_DPHY_REG_READ(0x1AC);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x1AC 0x%0x \n", r_data_8);

//dphy4txtester_DIG_RDWR_TX_PLL_0
//2 pll_atb_sense_sel_rw R/W Description: PLL analog test bus selection
r_data_8 = DSI_DPHY_REG_READ(0x15D);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x15D 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x15D,0x04);

r_data_8 = DSI_DPHY_REG_READ(0x15D);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x15D 0x%0x \n", r_data_8);

//meas_iv[17:0] bus selected (TX registers 0x167, 0x168, 0x169)
//dphy4txtester_DIG_RDWR_TX_PLL_10
//7:0 pll_meas_iv_rw__7__0__
//mpll_meas_iv[0] - DCC_p ?? connects the internal positive DCC control line - 0.41V @ meas_iv[11]=0
r_data_8 = DSI_DPHY_REG_READ(0x167);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x167 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x167,0x04);

r_data_8 = DSI_DPHY_REG_READ(0x167);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x167 0x%0x \n", r_data_8);

//3. Select atb_lptx1200_on_lane0 and place it on atb_line setting bit[0] of test control register with
//address 0x501 to 1'b1;
//dphy4txtester_DIG_RDWR_TX_LANE0_LANE_0

//r_data_8 = DSI_DPHY_REG_READ(0x501);
//DEBUG_PRINTF("MIPI DSI DPHY Read 0x501 0x%0x \n", r_data_8);

//DSI_DPHY_REG_WRITE(0x501,0x01);

//r_data_8 = DSI_DPHY_REG_READ(0x501);
//DEBUG_PRINTF("MIPI DSI DPHY Read 0x501 0x%0x \n", r_data_8);

//dphy4txtester_DIG_RDWR_TX_LANE1_LANE_0
//r_data_8 = DSI_DPHY_REG_READ(0x701);
//DEBUG_PRINTF("MIPI DSI DPHY Read 0x701 0x%0x \n", r_data_8);

//DSI_DPHY_REG_WRITE(0x701,0x01);

//r_data_8 = DSI_DPHY_REG_READ(0x701);
//DEBUG_PRINTF("MIPI DSI DPHY Read 0x701 0x%0x \n", r_data_8);

//atb_lptx1200_on_clklane Write in TX register 0x301 bit 0
//1- LP transmitter voltage reference 1.1V � 1.3V
//dphy4txtester_DIG_RDWR_TX_CLKLANE_LANE_0
//r_data_8 = DSI_DPHY_REG_READ(0x301);
//DEBUG_PRINTF("MIPI DSI DPHY Read 0x301 0x%0x \n", r_data_8);

//DSI_DPHY_REG_WRITE(0x301,0x01);

//r_data_8 = DSI_DPHY_REG_READ(0x301);
//DEBUG_PRINTF("MIPI DSI DPHY Read 0x301 0x%0x \n", r_data_8);

//4. Set bit [0] of TX test control register with address 0x1DA to 1??b0;
//dphy4txtester_DIG_RDWR_TX_DAC_0
//1 adc_done_clear_rw R/W Description: Clears last ADC procedure status
//0 adc_en_rw R/W Description: ADC enable, triggers ADC conversion
r_data_8 = DSI_DPHY_REG_READ(0x1DA);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x1DA 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x1DA,0x0);

r_data_8 = DSI_DPHY_REG_READ(0x1DA);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x1DA 0x%0x \n", r_data_8);

//5. Set bit [1] of TX test control register with address 0x1DA to 1??b1 to clear the previous ADC result;

DSI_DPHY_REG_WRITE(0x1DA,0x2);

r_data_8 = DSI_DPHY_REG_READ(0x1DA);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x1DA 0x%0x \n", r_data_8);

for(int i=0;i<50000;i++);

//pulse adc_done_clear bit
DSI_DPHY_REG_WRITE(0x1DA,0x0);

r_data_8 = DSI_DPHY_REG_READ(0x1DA);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x1DA 0x%0x \n", r_data_8);

//6. Set bit [0] of TX test control register with address 0x1DA to 1??b1;

DSI_DPHY_REG_WRITE(0x1DA,0x1);

r_data_8 = DSI_DPHY_REG_READ(0x1DA);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x1DA 0x%0x \n", r_data_8);

//7. Read TX test control register with address 0x1F2 and wait until bit[0] = 1??b1. That indicates the
//previous ADC result reading was cleared.
//dphy4txtester_DIG_RD_TX_DAC_0
//0 adc_done R Description: Done flag for observation in ADC machine
do {
  r_data_8 = DSI_DPHY_REG_READ(0x1F2);
  DEBUG_PRINTF("MIPI CSI DPHY Read 0x1F2 0x%0x \n", r_data_8);
} while( (r_data_8 & 0x01) != 0x01);

//8. Set bit [0] of TX test control register with address 0x1DA to 1??b0.
r_data_8 = DSI_DPHY_REG_READ(0x1DA);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x1DA 0x%0x \n", r_data_8);

DSI_DPHY_REG_WRITE(0x1DA,0x0);

r_data_8 = DSI_DPHY_REG_READ(0x1DA);
DEBUG_PRINTF("MIPI DSI DPHY Read 0x1DA 0x%0x \n", r_data_8);

////9. Set bit [1] of TX test control register with address 0x1DA to 1??b0.
//DSI_DPHY_REG_WRITE(0x1DA,0x2);
//
//r_data_8 = DSI_DPHY_REG_READ(0x1DA);
//DEBUG_PRINTF("MIPI DSI DPHY Read 0x1DA 0x%0x \n", r_data_8);

////for(int i=0;i<50000;i++);

////pulse adc_done_clear bit
//DSI_DPHY_REG_WRITE(0x1DA,0x0);
//
//r_data_8 = DSI_DPHY_REG_READ(0x1DA);
//DEBUG_PRINTF("MIPI DSI DPHY Read 0x1DA 0x%0x \n", r_data_8);

////10. Set bit [0] of TX test control register with address 0x1DA to 1??b1 to initiate the conversion procedure.
////The atb_line is compared with the Vdac value (generated using an ADC that represents the 10-bit
////input digital word).
//
//DSI_DPHY_REG_WRITE(0x1DA,0x1);
//
//r_data_8 = DSI_DPHY_REG_READ(0x1DA);
//DEBUG_PRINTF("MIPI DSI DPHY Read 0x1DA 0x%0x \n", r_data_8);

////11. The digital algorithm (Successive Approximation) iterates the 10 bits until the digital code represents
////the voltage on atb_line. The comparator output (cb_comp_out) can be observed reading bit[4] of test
////TX control register with address 0x221.
////dphy4txtester_DIG_RD_TX_TERM_CAL_1
////4 cb_comp_out R Description: Analog comparator output (volatile)
//
////12. Read TX test control register with address 0x1F2 and wait until bit[0] = 1??b1. That indicates the
////conversion process is complete.
//do {
//  r_data_8 = DSI_DPHY_REG_READ(0x1F2);
//  DEBUG_PRINTF("MIPI CSI DPHY Read 0x%0x \n", r_data_8);
//} while( (r_data_8 & 0x01) != 0x01);

//13. Read ADC result (cb_dac_prog[9:8],cb_dac_prog[7:0]):
//?? Bits [7:0] of TX test control register with address 0x1F3: cb_dac_prog[7:0];
//?? Bits [1:0] of TX test control register with address 0x1F4: cb_dac_prog[9:8];
r_data_8 = DSI_DPHY_REG_READ(0x1F3);
DEBUG_PRINTF("MIPI CSI DPHY Read 1F3 0x%0x \n", r_data_8);

r_data_8 = DSI_DPHY_REG_READ(0x1F4);
DEBUG_PRINTF("MIPI CSI DPHY Read 1F4 0x%0x \n", r_data_8);
*/
	return 0;
}


static uint8_t DSI_DPHY_REG_READ(int start_add)
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

static void DSI_DPHY_REG_WRITE(int start_add,uint8_t write_data)
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




