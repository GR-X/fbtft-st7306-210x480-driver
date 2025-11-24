// Linux fbtft driver for ST7306 (210x480) based on ESP32 implementation

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/atomic.h>

#include <video/mipi_display.h>
#include <linux/fb.h>

#include "fbtft.h"

#define DRV_NAME "fb_st7306"

#define ST7306_WIDTH 210
#define ST7306_HEIGHT 480

/* Panel uses two dummy bytes at the start of each line in the vendor code */
#define LINE_DUMMY_BYTES 2

/* Display recommends 40MHz, SPI mode 0 */

/* 与用户提供的一致的私有结构 */
struct st7306_par {
    struct fbtft_par *par;
    struct gpio_desc *te_gpio;
    int te_irq;
    wait_queue_head_t te_wait_queue;
    atomic_t te_received;
};

/* 使用 fbtft 提供的 write_reg 宏来写命令与参数；数据写入时手动拉高 DC 并调用 par->fbtftops.write */

/* 移除未使用的本地复位函数，统一使用 par->fbtftops.reset(par) */

/* TE中断 */
static irqreturn_t st7306_te_interrupt(int irq, void *data)
{
    struct st7306_par *stpar = data;
    atomic_set(&stpar->te_received, 1);
    wake_up_interruptible(&stpar->te_wait_queue);
    return IRQ_HANDLED;
}

static int st7306_request_te_irq(struct st7306_par *stpar)
{
    struct device *dev = &stpar->par->spi->dev;
    int ret;

    stpar->te_gpio = devm_gpiod_get(dev, "te", GPIOD_IN);
    if (IS_ERR(stpar->te_gpio)) {
        dev_err(dev, "Failed to get TE GPIO\n");
        return PTR_ERR(stpar->te_gpio);
    }

    stpar->te_irq = gpiod_to_irq(stpar->te_gpio);
    if (stpar->te_irq < 0) {
        dev_err(dev, "Failed to get IRQ number for TE GPIO: %d\n", stpar->te_irq);
        return stpar->te_irq;
    }

    ret = devm_request_irq(dev, stpar->te_irq, st7306_te_interrupt,
                           IRQF_TRIGGER_RISING, "st7306-te", stpar);
    if (ret) {
        dev_err(dev, "Failed to request TE IRQ: %d\n", ret);
        return ret;
    }

    init_waitqueue_head(&stpar->te_wait_queue);
    atomic_set(&stpar->te_received, 0);
    dev_info(dev, "TE interrupt registered successfully (IRQ: %d)\n", stpar->te_irq);
    return 0;
}

static int wait_for_te_sync(struct st7306_par *stpar)
{
    int ret;
    if (!stpar)
        return -EINVAL;

    if (stpar->te_gpio && !IS_ERR(stpar->te_gpio)) {
        ret = wait_event_interruptible_timeout(stpar->te_wait_queue,
                                               atomic_read(&stpar->te_received),
                                               HZ / 20);
        if (ret == 0){
            printk(KERN_INFO "ST7306: TE timeout\n");
            return -ETIMEDOUT;
        }
        if (ret > 0) {
            atomic_set(&stpar->te_received, 0);
            return 0;
        }
        return ret;
    }
    return 0;
}

static int init_st7306(struct fbtft_par *par)
{
    struct st7306_par *stpar;

    par->extra = kzalloc(sizeof(*stpar), GFP_KERNEL);
    if (!par->extra)
        return -ENOMEM;
    stpar = par->extra;
    stpar->par = par;

    if (st7306_request_te_irq(stpar))
        dev_warn(par->info->device, "TE synchronization disabled\n");

    /* 复位 */
    par->fbtftops.reset(par);
    mdelay(50);

    /* NVM Load Control */
    write_reg(par, 0xD6, 0x17, 0x02);

    /* Booster Enable */
    write_reg(par, 0xD1, 0x01);

    /* Gate Voltage Control */
    write_reg(par, 0xC0, 0x0E, 0x0A);

    /* VSHP Setting */
    write_reg(par, 0xC1, 0x41, 0x41, 0x41, 0x41);

    /* VSLP Setting */
    write_reg(par, 0xC2, 0x32, 0x32, 0x32, 0x32);

    /* VSHN Setting */
    write_reg(par, 0xC4, 0x46, 0x46, 0x46, 0x46);

    /* VSLN Setting */
    write_reg(par, 0xC5, 0x46, 0x46, 0x46, 0x46);

    /* Frame Rate Control */
    write_reg(par, 0xB2, 0x12);

    /* Update Period Gate EQ Control in HPM */
    write_reg(par, 0xB3, 0xE5, 0xF6, 0x05, 0x46, 0x77, 0x77, 0x77, 0x77, 0x76, 0x45);

    /* Update Period Gate EQ Control in LPM */
    write_reg(par, 0xB4, 0x05, 0x46, 0x77, 0x77, 0x77, 0x77, 0x76, 0x45);

    /* Source EQ Enable */
    write_reg(par, 0xB7, 0x13);

    /* Gate Line Setting = 480 lines */
    write_reg(par, 0xB0, 0x78);

	/* Sleep-out */
    write_reg(par, 0x11);
	mdelay(120);

	/* OSC Setting (51Hz) */
    write_reg(par, 0xD8, 0x80, 0xE9);

	/* Source Voltage Select */
    write_reg(par, 0xC9, 0x00);

	/* Memory Data Access Control */
    write_reg(par, 0x36, 0x48);

	/* Data Format Select (0x32) */
    write_reg(par, 0x3a, 0x32);

	/* Gamma Mode Setting: 4GS */
    write_reg(par, 0xB9, 0x00);

	/* Panel Setting */
    write_reg(par, 0xB8, 0x0A);

	/* TE Setting */
    write_reg(par, 0x35, 0x00);

	/* Enable Auto Power down */
    write_reg(par, 0xD0, 0xFF);

	/* HPM ON */
    write_reg(par, 0x38);
    write_reg(par, 0x29);
    printk(KERN_INFO "ST7306: init successfully\n");
    
    return 0;
}


static void st7306_pack_line(const u16 *rgb565_line_even,
				const u16 *rgb565_line_odd,
				u8 *dst /* size >= LINE_DUMMY_BYTES + ST7306_WIDTH */)
{
	int x;
	/* two dummy bytes set to 0x00 produces white in panel code */
	dst[0] = 0x00;
	dst[1] = 0x00;
	for (x = 0; x < ST7306_WIDTH; x++) {
		u16 c_even = rgb565_line_even[x];
		u16 c_odd  = rgb565_line_odd[x];
		u8 r5_e = (c_even >> 11) & 0x1F;
		u8 g6_e = (c_even >> 5)  & 0x3F;
		u8 b5_e = (c_even >> 0)  & 0x1F;
		u8 r5_o = (c_odd  >> 11) & 0x1F;
		u8 g6_o = (c_odd  >> 5)  & 0x3F;
		u8 b5_o = (c_odd  >> 0)  & 0x1F;

		/* vendor mapping uses top bits of each channel:
		 * maskBit = B[4]>>2 | G[5]>>4 | R[4]>>4
		 * even rows occupy bits 2..4, odd rows occupy bits 5..7
		 * and 0 means colored (inverted bits)
		 */
		u8 m_even = ((b5_e & 0x10) >> 2) | ((g6_e & 0x20) >> 4) | ((r5_e & 0x10) >> 4);
		u8 m_odd  = ((b5_o & 0x10) >> 2) | ((g6_o & 0x20) >> 4) | ((r5_o & 0x10) >> 4);
		u8 packed = 0;
		/* start with ones in used bits, then clear according to masks */
		packed |= 0x1C; /* bits 2..4 set */
		packed |= 0xE0; /* bits 5..7 set */
		packed &= ~(m_even << 2);
		packed &= ~(m_odd  << 5);
		dst[LINE_DUMMY_BYTES + x] = packed;
	}
}


static void st7306_set_addr_win(struct fbtft_par *par, int xs, int ys, int xe, int ye)
{
	/* Column address set (0x2A): X range encoded as (XS..XE) like vendor */
    write_reg(par, 0x2a, 0x04, 0x38);
    write_reg(par, 0x2b, 0x00, (ST7306_HEIGHT/2 - 1));
    write_reg(par, 0x2c);
}

static int st7306_write_vmem(struct fbtft_par *par, size_t offset, size_t len)
{
	/* Push full frame every time; controller prefers split into two halves */
	struct fb_info *info = par->info;
    const u16 *vmem = (const u16 *)info->screen_base;
	/* each transfer half covers 120 logical lines; pack pair of lines at a time */
	const int logical_lines = ST7306_HEIGHT; /* 480 */
	const int half = logical_lines / 2;      /* 240 */
	const int group = 2; /* even+odd */
    int y, ret;
    struct st7306_par *stpar = par->extra;

    /* 设置固定地址窗口 */
    st7306_set_addr_win(par, 0, 0, ST7306_WIDTH-1, ST7306_HEIGHT-1);

	/* allocate one packed line buffer */
	{
		size_t packed_line_bytes = LINE_DUMMY_BYTES + ST7306_WIDTH;
		u8 *linebuf = kmalloc(packed_line_bytes, GFP_KERNEL);
		if (!linebuf)
			return -ENOMEM;

        /* TE 同步并发送前半屏（0..239，共 120 对行） */
        ret = wait_for_te_sync(stpar);
        /* 即使超时也继续以保持刷新 */
      //  write_reg(par, MIPI_DCS_WRITE_MEMORY_START);
        gpiod_set_value(par->gpio.dc, 1);
		for (y = 0; y < half; y += group) {
			const u16 *even = vmem + (y + 0) * info->var.xres;
			const u16 *odd  = vmem + (y + 1) * info->var.xres;
			st7306_pack_line(even, odd, linebuf);
            if (par->gpio.dc)
                gpiod_set_value_cansleep(par->gpio.dc, 1);
            par->fbtftops.write(par, linebuf, packed_line_bytes);
		}

		for (y = half; y < logical_lines; y += group) {
			const u16 *even = vmem + (y + 0) * info->var.xres;
			const u16 *odd  = vmem + (y + 1) * info->var.xres;
			st7306_pack_line(even, odd, linebuf);
            if (par->gpio.dc)
                gpiod_set_value_cansleep(par->gpio.dc, 1);
            par->fbtftops.write(par, linebuf, packed_line_bytes);
		}
		kfree(linebuf);
	}
    printk(KERN_INFO "ST7306: send successfully\n");
	return 0;
}


static int st7306_set_var(struct fbtft_par *par)
{
	/* Fixed geometry */
	par->info->var.xres = ST7306_WIDTH;
	par->info->var.yres = ST7306_HEIGHT;
	par->info->var.bits_per_pixel = 16; /* RGB565 input */
	return 0;
}

static struct fbtft_display display = {
	.regwidth = 8,
	.width = ST7306_WIDTH,
	.height = ST7306_HEIGHT,
	.gamma_num = 0,
	.bpp = 16,
	.fbtftops = {
        .init_display = init_st7306,
		.write_vmem = st7306_write_vmem,
		.set_var = st7306_set_var,
	},
};

FBTFT_REGISTER_DRIVER(DRV_NAME, "sitronix,st7306", &display);

MODULE_DESCRIPTION("ST7306 fbtft driver (210x480) using packed 6bpp vertical pairs");
MODULE_AUTHOR("@GR");
MODULE_LICENSE("GPL");
