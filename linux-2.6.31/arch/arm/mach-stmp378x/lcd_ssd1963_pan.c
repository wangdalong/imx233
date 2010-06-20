#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/notifier.h>
#include <linux/regulator/consumer.h>
#include <mach/regs-lcdif.h>
#include <mach/regs-lradc.h>
#include <mach/regs-pinctrl.h>
#include <mach/regs-clkctrl.h>
#include <mach/regs-pwm.h>
#include <mach/regs-apbh.h>
#include <mach/gpio.h>
#include <mach/pins.h>
#include <mach/pinmux.h>
#include <mach/lcdif.h>
#include <mach/stmp3xxx.h>
#include <mach/platform.h>
#include <mach/cputype.h>
#include <linux/ctype.h>
#include <linux/vmalloc.h>

#define CMD 0
#define DATA    1

#define lcdif_read(reg) __raw_readl(REGS_LCDIF_BASE + reg)
#define lcdif_write(reg,val) __raw_writel(val, REGS_LCDIF_BASE + reg)

extern struct pin_group lcd_pins;
static struct clk *lcd_clk;

static char lcd_init_data[] =
{
    0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0xe0, 0x01, 0x01, 0x00, 0xe0, 0x01, 0x03,
    0x00, 0x36, 0x01, 0x08, 0x00, 0xb0, 0x01, 0x0c, 0x01, 0x80, 0x01, 0x01, 0x01, 0xdf,
    0x01, 0x01, 0x01, 0x0f, 0x01, 0x00, 0x00, 0xf0, 0x01, 0x00, 0x00, 0x3a, 0x01, 0x60,
    0x00, 0xe6, 0x01, 0x01, 0x01, 0x45, 0x01, 0x47, 0x00, 0xb4, 0x01, 0x02, 0x01, 0x0d,
    0x01, 0x00, 0x01, 0x2b, 0x01, 0x28, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0xb6,
    0x01, 0x01, 0x01, 0x1d, 0x01, 0x00, 0x01, 0x0c, 0x01, 0x09, 0x01, 0x00, 0x01, 0x00,
    0x00, 0x2a, 0x01, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0xdf, 0x00, 0x2b, 0x01, 0x00,
    0x01, 0x00, 0x01, 0x01, 0x01, 0x0f, 0x00, 0x29, 0x00, 0x2c
};

static void mpulcd_setup_pannel_register(char data, char val)
{

    lcdif_write(HW_LCDIF_CTRL_CLR,
        BM_LCDIF_CTRL_LCDIF_MASTER |
        BM_LCDIF_CTRL_RUN);

    lcdif_write(HW_LCDIF_TRANSFER_COUNT,
        BF_LCDIF_TRANSFER_COUNT_V_COUNT(1) |
                BF_LCDIF_TRANSFER_COUNT_H_COUNT(1));

    if(data)
        lcdif_write(HW_LCDIF_CTRL_SET, BM_LCDIF_CTRL_DATA_SELECT);
    else
        lcdif_write(HW_LCDIF_CTRL_CLR, BM_LCDIF_CTRL_DATA_SELECT);

    lcdif_write(HW_LCDIF_CTRL_SET, BM_LCDIF_CTRL_RUN);

    lcdif_write(HW_LCDIF_DATA, val);

    while(lcdif_read(HW_LCDIF_CTRL) & BM_LCDIF_CTRL_RUN)
    ;

    lcdif_write(HW_LCDIF_CTRL1_CLR, BM_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ);
}

void mpulcd_start_refresh(void)
{

    mpulcd_setup_pannel_register(CMD, 0x2c);

    lcdif_write(HW_LCDIF_CTRL_SET, BM_LCDIF_CTRL_DATA_SELECT);
    lcdif_write(HW_LCDIF_TRANSFER_COUNT,
        BF_LCDIF_TRANSFER_COUNT_V_COUNT(272) |
        BF_LCDIF_TRANSFER_COUNT_H_COUNT(480*3));

    stmp3xxx_lcdif_run();
}

static void mpulcd_init_lcdif(void)
{

    lcdif_write(HW_LCDIF_CTRL_CLR,
        BM_LCDIF_CTRL_CLKGATE |
        BM_LCDIF_CTRL_SFTRST);

    lcdif_write(HW_LCDIF_CTRL,
        BF_LCDIF_CTRL_LCD_DATABUS_WIDTH(BV_LCDIF_CTRL_LCD_DATABUS_WIDTH__8_BIT) |
        BF_LCDIF_CTRL_WORD_LENGTH(BV_LCDIF_CTRL_WORD_LENGTH__8_BIT));

    lcdif_write(HW_LCDIF_TIMING, 0x01010101);
}

static void mpulcd_init_panel_hw(void)
{
    int i;

    stmp3xxx_request_pin_group(&lcd_pins, "mpulcd_pin");

    gpio_request(PINID_LCD_ENABLE, "sdd1963");
    gpio_direction_output(PINID_LCD_ENABLE, 1);

    for(i = 0; i < sizeof(lcd_init_data); i += 2) {
    mpulcd_setup_pannel_register(lcd_init_data[i], lcd_init_data[i + 1]);
    udelay(100);
    }
}

static int mpulcd_init_panel(struct device *dev, dma_addr_t phys, int memsize,
        struct stmp3xxx_platform_fb_entry *pentry)
{
    int ret = 0;

    lcd_clk = clk_get(dev, "lcdif");
    if (IS_ERR(lcd_clk)) {
        ret = PTR_ERR(lcd_clk);
        goto out;
    }

    ret = clk_enable(lcd_clk);
    if (ret)
        goto out1;

    ret = clk_set_rate(lcd_clk, 24000);
    if (ret)
        goto out2;

    mpulcd_init_lcdif();
    stmp3xxx_lcdif_dma_init(dev, phys, memsize, 1);
    mpulcd_init_panel_hw();
    stmp3xxx_lcdif_notify_clients(STMP3XXX_LCDIF_PANEL_INIT, pentry);

    return 0;

out2:
    clk_disable(lcd_clk);
out1:
    clk_put(lcd_clk);
out:
    return ret;
}

static void mpulcd_display_on(void)
{

    mpulcd_setup_pannel_register(CMD, 0x29);
    lcdif_write(HW_LCDIF_CTRL1_SET, BM_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ_EN);
    mpulcd_start_refresh();
}

static void mpulcd_display_off(void)
{

    lcdif_write(HW_LCDIF_CTRL1_CLR, BM_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ_EN);
    mpulcd_setup_pannel_register(CMD, 0x28);
}

static void mpulcd_release_panel(struct device *dev,
        struct stmp3xxx_platform_fb_entry *pentry)
{

    stmp3xxx_lcdif_notify_clients(STMP3XXX_LCDIF_PANEL_RELEASE, pentry);
    mpulcd_display_off();
    stmp3xxx_release_pin_group(&lcd_pins, "mpulcd_pin");
    stmp3xxx_lcdif_dma_release();
    clk_disable(lcd_clk);
    clk_put(lcd_clk);
    gpio_direction_output(PINID_LCD_ENABLE, 0);
}

static int mpulcd_blank_panel(int blank)
{
    int ret = 0;

    switch (blank)
    {
        case FB_BLANK_NORMAL:
        case FB_BLANK_HSYNC_SUSPEND:
        case FB_BLANK_POWERDOWN:
        case FB_BLANK_VSYNC_SUSPEND:
        case FB_BLANK_UNBLANK:
            break;

        default:
            ret = -EINVAL;
    }

    return ret;
}

int mpulcd_pan_display(dma_addr_t addr)
{

    lcdif_write(HW_LCDIF_CUR_BUF, addr);
    return 0;
}


static struct stmp3xxx_platform_fb_entry fb_entry = {
    .name           = "ssd1963",
    .x_res          = 272,
    .y_res          = 480,
    .bpp            = 24,
    .cycle_time_ns      = 150,
    .lcd_type       = STMP3XXX_LCD_PANEL_SYSTEM,
    .init_panel     = mpulcd_init_panel,
    .release_panel      = mpulcd_release_panel,
    .blank_panel        = mpulcd_blank_panel,
    .run_panel      = mpulcd_display_on,
    .stop_panel     = mpulcd_display_off,
    .pan_display        = mpulcd_pan_display,
};



static int __init register_devices(void)
{

    stmp3xxx_lcd_register_entry(&fb_entry, stmp3xxx_framebuffer.dev.platform_data);
    return 0;
}

subsys_initcall(register_devices);
