/*
 * Jz4740 SLCD Controller
 *
 *  Copyright (c) 2009 yajin <yajin@vm-kernel.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*Most of these code is from rockbox.*/


#include <config.h>
#include <common.h>
#include <devices.h>
#include <lcd.h>

#include <asm/io.h>             /* virt_to_phys() */

#if defined(CONFIG_JZ4730)
#include <asm/jz4730.h>
#endif
#if defined(CONFIG_JZ4740)
#include <asm/jz4740.h>
#endif
#if defined(CONFIG_JZ5730)
#include <asm/jz5730.h>
#endif

#include "jz_lcd.h"
#include "jz_slcd.h"



#define SLCD_DMA_CHAN             0

int lcd_line_length;

int lcd_color_fg;
int lcd_color_bg;

 /*
  * Frame buffer memory information
  */
void *lcd_base;                 /* Start of framebuffer memory  */
void *lcd_console_address;      /* Start of console buffer      */

short console_col;
short console_row;

 /************************************************************************/

void lcd_ctrl_init(void *lcdbase);

void lcd_enable(void);
void lcd_disable(void);

vidinfo_t panel_info = {
#if defined(CONFIG_JZSLCD_TFT_G240400RTSW_3WTP_E)
    240, 400, 4,
#endif
};


#define my__gpio_as_lcd_16bit()         \
do {                                    \
    REG_GPIO_PXFUNS(2) = 0x001cffff;    \
    REG_GPIO_PXSELC(2) = 0x001cffff;    \
    REG_GPIO_PXPES(2) = 0x001cffff;     \
} while (0)

static void _display_pin_init(void)
{
    int i;
    my__gpio_as_lcd_16bit();
    __gpio_as_output(PIN_CS_N);
    __gpio_as_output(PIN_RESET_N);
    __gpio_clear_pin(PIN_CS_N);
    __gpio_set_pin(PIN_RESET_N);
    mdelay(5);
    __gpio_clear_pin(PIN_RESET_N);
    mdelay(5);
    __gpio_set_pin(PIN_RESET_N);
    mdelay(5);
    __gpio_as_output(BACKLIGHT_GPIO);
    __gpio_set_pin(BACKLIGHT_GPIO);
}

static void _set_slcd_bus(void)
{
    REG_LCD_CFG &= ~LCD_CFG_LCDPIN_MASK;
    REG_LCD_CFG |= LCD_CFG_LCDPIN_SLCD;

    REG_SLCD_CFG =
        (SLCD_CFG_BURST_8_WORD | SLCD_CFG_DWIDTH_16 | SLCD_CFG_CWIDTH_16BIT |
         SLCD_CFG_CS_ACTIVE_LOW | SLCD_CFG_RS_CMD_LOW |
         SLCD_CFG_CLK_ACTIVE_FALLING | SLCD_CFG_TYPE_PARALLEL);
}

static void _set_slcd_clock(void)
{
    unsigned int val;
    int pll_div;

    __cpm_stop_lcd();
    pll_div = (REG_CPM_CPCCR & CPM_CPCCR_PCS);  /* clock source, 0:pllout/2 1: pllout */
    pll_div = pll_div ? 1 : 2;
    val = (__cpm_get_pllout() / pll_div) / __cpm_get_pclk();
    val--;
    if (val > 0x1ff)
        val = 0x1ff;            /* CPM_LPCDR is too large, set it to 0x1ff */
    __cpm_set_pixdiv(val);
    __cpm_start_lcd();
}

static void _display_init(void)
{
    int i;

    SLCD_SEND_COMMAND(REG_SOFT_RESET, SOFT_RESET(1));
    mdelay(2);
    SLCD_SEND_COMMAND(REG_SOFT_RESET, SOFT_RESET(0));
    mdelay(2);
    SLCD_SEND_COMMAND(REG_ENDIAN_CTRL, 0);

    SLCD_SEND_COMMAND(REG_DRIVER_OUTPUT, 0x100);
    SLCD_SEND_COMMAND(REG_LCD_DR_WAVE_CTRL, 0x100);
#if CONFIG_ORIENTATION == SCREEN_PORTRAIT
    SLCD_SEND_COMMAND(REG_ENTRY_MODE,
                      (ENTRY_MODE_BGR | ENTRY_MODE_VID | ENTRY_MODE_HID |
                       ENTRY_MODE_HWM));
#else
    SLCD_SEND_COMMAND(REG_ENTRY_MODE,
                      (ENTRY_MODE_BGR | ENTRY_MODE_VID | ENTRY_MODE_AM |
                      ENTRY_MODE_HWM));
#endif
    SLCD_SEND_COMMAND(REG_DISP_CTRL2, 0x503);
    SLCD_SEND_COMMAND(REG_DISP_CTRL3, 1);
    SLCD_SEND_COMMAND(REG_LPCTRL, 0x10);
    SLCD_SEND_COMMAND(REG_EXT_DISP_CTRL1, 0);
    SLCD_SEND_COMMAND(REG_EXT_DISP_CTRL2, 0);
    SLCD_SEND_COMMAND(REG_DISP_CTRL1, DISP_CTRL1_D(1));
    SLCD_SEND_COMMAND(REG_PAN_INTF_CTRL1, 0x12);
    SLCD_SEND_COMMAND(REG_PAN_INTF_CTRL2, 0x202);
    SLCD_SEND_COMMAND(REG_PAN_INTF_CTRL3, 0x300);
    SLCD_SEND_COMMAND(REG_PAN_INTF_CTRL4, 0x21e);
    SLCD_SEND_COMMAND(REG_PAN_INTF_CTRL5, 0x202);
    SLCD_SEND_COMMAND(REG_PAN_INTF_CTRL6, 0x100);
    SLCD_SEND_COMMAND(REG_FRM_MRKR_CTRL, 0x8000);
    SLCD_SEND_COMMAND(REG_PWR_CTRL1,
                      (PWR_CTRL1_SAPE | PWR_CTRL1_BT(6) | PWR_CTRL1_APE |
                       PWR_CTRL1_AP(3)));
    SLCD_SEND_COMMAND(REG_PWR_CTRL2, 0x147);
    SLCD_SEND_COMMAND(REG_PWR_CTRL3, 0x1bd);
    SLCD_SEND_COMMAND(REG_PWR_CTRL4, 0x2f00);
    SLCD_SEND_COMMAND(REG_PWR_CTRL5, 0);
    SLCD_SEND_COMMAND(REG_PWR_CTRL6, 1);
    SLCD_SEND_COMMAND(REG_RAM_HADDR_SET, 0);    /* set cursor at x_start */
    SLCD_SEND_COMMAND(REG_RAM_VADDR_SET, 0);    /* set cursor at y_start */
#if CONFIG_ORIENTATION == SCREEN_PORTRAIT
    SLCD_SEND_COMMAND(REG_RAM_HADDR_START, 0);  /* y_start */
    SLCD_SEND_COMMAND(REG_RAM_HADDR_END, 239);  /* y_end */
    SLCD_SEND_COMMAND(REG_RAM_VADDR_START, 0);  /* x_start */
    SLCD_SEND_COMMAND(REG_RAM_VADDR_END, 399);  /* x_end */
#else
    SLCD_SEND_COMMAND(REG_RAM_HADDR_START, 0);  /* y_start */
    SLCD_SEND_COMMAND(REG_RAM_HADDR_END, 399);  /* y_end */
    SLCD_SEND_COMMAND(REG_RAM_VADDR_START, 0);  /* x_start */
    SLCD_SEND_COMMAND(REG_RAM_VADDR_END, 239);  /* x_end */
#endif
    SLCD_SEND_COMMAND(REG_RW_NVM, 0);
    SLCD_SEND_COMMAND(REG_VCOM_HVOLTAGE1, 6);
    SLCD_SEND_COMMAND(REG_VCOM_HVOLTAGE2, 0);
    SLCD_SEND_COMMAND(REG_GAMMA_CTRL1, 0x101);
    SLCD_SEND_COMMAND(REG_GAMMA_CTRL2, 0xb27);
    SLCD_SEND_COMMAND(REG_GAMMA_CTRL3, 0x132a);
    SLCD_SEND_COMMAND(REG_GAMMA_CTRL4, 0x2a13);
    SLCD_SEND_COMMAND(REG_GAMMA_CTRL5, 0x270b);
    SLCD_SEND_COMMAND(REG_GAMMA_CTRL6, 0x101);
    SLCD_SEND_COMMAND(REG_GAMMA_CTRL7, 0x1205);
    SLCD_SEND_COMMAND(REG_GAMMA_CTRL8, 0x512);
    SLCD_SEND_COMMAND(REG_GAMMA_CTRL9, 5);
    SLCD_SEND_COMMAND(REG_GAMMA_CTRL10, 3);
    SLCD_SEND_COMMAND(REG_GAMMA_CTRL11, 0xf04);
    SLCD_SEND_COMMAND(REG_GAMMA_CTRL12, 0xf00);
    SLCD_SEND_COMMAND(REG_GAMMA_CTRL13, 0xf);
    SLCD_SEND_COMMAND(REG_GAMMA_CTRL14, 0x40f);
    SLCD_SEND_COMMAND(0x30e, 0x300);
    SLCD_SEND_COMMAND(0x30f, 0x500);
    SLCD_SEND_COMMAND(REG_BIMG_NR_LINE, 0x3100);
    SLCD_SEND_COMMAND(REG_BIMG_DISP_CTRL, 1);
    SLCD_SEND_COMMAND(REG_BIMG_VSCROLL_CTRL, 0);
    SLCD_SEND_COMMAND(REG_PARTIMG1_POS, 0);
    SLCD_SEND_COMMAND(REG_PARTIMG1_RAM_START, 0);
    SLCD_SEND_COMMAND(REG_PARTIMG1_RAM_END, 0);
    SLCD_SEND_COMMAND(REG_PARTIMG2_POS, 0);
    SLCD_SEND_COMMAND(REG_PARTIMG2_RAM_START, 0);
    SLCD_SEND_COMMAND(REG_PARTIMG2_RAM_END, 0);
    SLCD_SEND_COMMAND(REG_ENDIAN_CTRL, 0);
    SLCD_SEND_COMMAND(REG_NVM_ACCESS_CTRL, 0);
    SLCD_SEND_COMMAND(0x7f0, 0x5420);
    SLCD_SEND_COMMAND(0x7f3, 0x288a);
    SLCD_SEND_COMMAND(0x7f4, 0x22);
    SLCD_SEND_COMMAND(0x7f5, 1);
    SLCD_SEND_COMMAND(0x7f0, 0);

    /* LCD ON: */
    SLCD_SEND_COMMAND(REG_DISP_CTRL1, (DISP_CTRL1_BASEE | DISP_CTRL1_VON
                                       | DISP_CTRL1_GON | DISP_CTRL1_DTE |
                                       DISP_CTRL1_D(3)));
    mdelay(5);
    SLCD_SEND_COMMAND(REG_DISP_CTRL1, (DISP_CTRL1_BASEE | DISP_CTRL1_VON
                                       | DISP_CTRL1_GON | DISP_CTRL1_DTE |
                                       DISP_CTRL1_D(2)));
    mdelay(5);
    SLCD_SEND_COMMAND(REG_DISP_CTRL1, (DISP_CTRL1_BASEE | DISP_CTRL1_VON
                                       | DISP_CTRL1_GON | DISP_CTRL1_DTE |
                                       DISP_CTRL1_D(3)));
    mdelay(5);
}

void lcd_set_target(short x, short y, short width, short height)
{
#if CONFIG_ORIENTATION == SCREEN_PORTRAIT
    SLCD_SEND_COMMAND(REG_RAM_HADDR_START, y);  /* y_start */
    SLCD_SEND_COMMAND(REG_RAM_HADDR_END, y + width - 1);        /* y_end */
    SLCD_SEND_COMMAND(REG_RAM_VADDR_START, x);  /* x_start */
    SLCD_SEND_COMMAND(REG_RAM_VADDR_END, x + height - 1);       /* x_end */
#else
    SLCD_SEND_COMMAND(REG_RAM_HADDR_START, y);  /* y_start */
    SLCD_SEND_COMMAND(REG_RAM_HADDR_END, y + height - 1);       /* y_end */
    SLCD_SEND_COMMAND(REG_RAM_VADDR_START, x);  /* x_start */
    SLCD_SEND_COMMAND(REG_RAM_VADDR_END, x + width - 1);        /* x_end */
#endif
    SLCD_SEND_COMMAND(REG_RAM_HADDR_SET, y);    /* set cursor at x_start */
    SLCD_SEND_COMMAND(REG_RAM_VADDR_SET, x);    /* set cursor at y_start */
    SLCD_SET_COMMAND(REG_RW_GRAM);      /* write data to GRAM */
}

static void __slcd_display_on(void)
{
    int i;
    SLCD_SEND_COMMAND(REG_PWR_CTRL1,
                      (PWR_CTRL1_SAPE | PWR_CTRL1_BT(6) | PWR_CTRL1_APE |
                       PWR_CTRL1_AP(3)));
    mdelay(5);
    SLCD_SEND_COMMAND(REG_DISP_CTRL1, (DISP_CTRL1_VON | DISP_CTRL1_GON
                                       | DISP_CTRL1_D(1)));
    mdelay(5);
    SLCD_SEND_COMMAND(REG_DISP_CTRL1, (DISP_CTRL1_VON | DISP_CTRL1_GON
                                       | DISP_CTRL1_DTE | DISP_CTRL1_D(3)
                                       | DISP_CTRL1_BASEE));
}
static void __slcd_display_off(void)
{
    int i;
    SLCD_SEND_COMMAND(REG_DISP_CTRL1, (DISP_CTRL1_VON | DISP_CTRL1_GON
                                       | DISP_CTRL1_DTE | DISP_CTRL1_D(2)));
    mdelay(5);
    SLCD_SEND_COMMAND(REG_DISP_CTRL1, DISP_CTRL1_D(1));
    mdelay(5);
    SLCD_SEND_COMMAND(REG_DISP_CTRL1, DISP_CTRL1_D(0));
    mdelay(5);
    SLCD_SEND_COMMAND(REG_PWR_CTRL1, PWR_CTRL1_SLP);
}

/************JZ SLCD DMA Stuff***********************/


/*SLCD DMA descriptor*/
struct jz_dma_desc
{
    volatile unsigned int dcmd; /* DCMD value for the current transfer */
    volatile unsigned int dsadr;        /* DSAR value for the current transfer */
    volatile unsigned int dtadr;        /* DTAR value for the current transfer */
    volatile unsigned int ddadr;        /* Points to the next descriptor + transfer count */
};

unsigned int dma_src_addr;
unsigned int dma_dst_addr;
unsigned int dma_src_phys_addr, dma_dst_phys_addr;
struct jz_dma_desc *desc;
unsigned int desc_phys_addr;

void jz_slcd_update()
{
    char buff[256];
    desc->dcmd =
        DMAC_DCMD_SAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 |
        DMAC_DCMD_DWDH_16 | DMAC_DCMD_DS_16BIT | DMAC_DCMD_TM | DMAC_DCMD_DES_V
        | DMAC_DCMD_DES_VM | DMAC_DCMD_DES_VIE | DMAC_DCMD_TIE;
    desc->dsadr = dma_src_phys_addr;    /* DMA source address */
    desc->dtadr = dma_dst_phys_addr;    /* DMA target address */
    desc->ddadr = panel_info.vl_col * panel_info.vl_row;

    flush_cache_all();

    /* Setup DMA descriptor address */
    REG_DMAC_DDA(SLCD_DMA_CHAN) = desc_phys_addr;

    /* Setup request source */
    REG_DMAC_DRSR(SLCD_DMA_CHAN) = DMAC_DRSR_RS_SLCD;

    while (REG_SLCD_STATE & SLCD_STATE_BUSY);
    REG_SLCD_CTRL |= SLCD_CTRL_DMA_EN;  /* Enable SLCD DMA support */

    /* Setup DMA channel control/status register */
    REG_DMAC_DCCSR(SLCD_DMA_CHAN) = DMAC_DCCSR_EN;      /* descriptor transfer, clear status, start channel */

    /* Enable DMA */
    REG_DMAC_DMACR = DMAC_DMACR_DMAE;

    /* DMA doorbell set -- start DMA now ... */
    REG_DMAC_DMADBSR = 1 << SLCD_DMA_CHAN;

    while (1)
    {
        if (REG_DMAC_DCCSR((SLCD_DMA_CHAN)) & DMAC_DCCSR_TT)
        {
            REG_DMAC_DCCSR(SLCD_DMA_CHAN) &= ~DMAC_DCCSR_TT;
            break;
        }
    }

}

static void jz_slcd_desc_init(void *lcdbase, vidinfo_t * vid)
{

    char buff[256];


    dma_src_addr = (unsigned int) lcdbase;
    dma_dst_addr = SLCD_FIFO;
    dma_src_phys_addr = virt_to_phys((void *) dma_src_addr);
    dma_dst_phys_addr = virt_to_phys((void *) dma_dst_addr);

    desc =
        (struct jz_dma_desc *) ((u_long) lcdbase +
                                vid->vl_row * (vid->vl_col *
                                               NBITS(vid->vl_bpix)) / 8);
    desc_phys_addr = virt_to_phys((void *) desc);

}

void lcd_enable(void)
{
    __slcd_display_on();
}

void lcd_disable(void)
{
    __slcd_display_off();
}

void lcd_ctrl_init(void *lcdbase)
{
    _display_pin_init();
    _set_slcd_bus();
    _set_slcd_clock();
    mdelay(1);
    _display_init();
    __cpm_start_lcd();
    lcd_set_target(0, 0, panel_info.vl_col, panel_info.vl_row);

    jz_slcd_desc_init(lcdbase, &panel_info);
    __slcd_display_on();
}

