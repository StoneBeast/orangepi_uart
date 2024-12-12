/*** 
 * @Author       : stoneBeast
 * @Date         : 2024-12-12 15:10:31
 * @Encoding     : UTF-8
 * @LastEditors  : stoneBeast
 * @LastEditTime : 2024-12-12 18:13:57
 * @Description  : 通过orangepi3-lts上的串口进行驱动联系
 */

#include "asm/io.h"
#include "linux/cdev.h"
#include "linux/device.h"
#include "linux/device/class.h"
#include "linux/export.h"
#include "linux/fs.h"
#include "linux/init.h"
#include "linux/kern_levels.h"
#include "linux/module.h"
#include "linux/printk.h"
#include "linux/types.h"

#define CCU_BASE_ADDR           0x03001000
#define UART_BGR_REG_OFFSET     0x090C
#define PIO_BASE_ADDR           0x0300B000
#define PIO_CFG2_OFFSET         3*0x0024+0x08
#define PIO_CFG3_OFFSET         3*0x0024+0x0C
#define UART3_BASS_ADDR         0x05000C00
#define UART_LCR_OFFSET         0x000C
#define APB2_CFG_ADDR           CCU_BASE_ADDR + 0x524

// TODO: cdev
// TODO: miscdev & platform
// TODO: 设备树

struct orangepi_uart_dev_t {
    struct cdev cdev;
    dev_t devno;
};

static struct orangepi_uart_dev_t orangepi_uart_dev;
static struct class *cls;
static struct device *dev;
static unsigned int *uart_clk_cfg;
static unsigned int *gpio_cfg2;
static unsigned int *gpio_cfg3;
static unsigned int *uart_lcr;
static unsigned int *apb2_clk;
static unsigned int *uart3_reg0;
static unsigned int *uart3_reg1;

static int orangepi_uart_open(struct inode *inode, struct file *filp)
{
    return 0;
}

static long orangepi_uart_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return 0;
}

static ssize_t orangepi_uart_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
    return 0;
}

static ssize_t orangepi_uart_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
    return 0;
}

static int orangepi_uart_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static struct file_operations f_ops = {
    .owner = THIS_MODULE,
    .read = orangepi_uart_read,
    .write = orangepi_uart_write,
    .open = orangepi_uart_open,
    .release = orangepi_uart_release,
    .unlocked_ioctl =orangepi_uart_ioctl,
};

static int __init orangepi_uart_init(void)
{
    int ret;

    printk("hhh\n");

    /* 申请设备号 */
    ret = alloc_chrdev_region(&orangepi_uart_dev.devno, 0, 1, "orangepi_uart");
    if (ret < 0)
    {
        printk(KERN_ERR "alloc_chrdev_region error, %d\n", ret);
        return -1;
    }

    /* 初始化cdev */
    cdev_init(&orangepi_uart_dev.cdev, &f_ops);
    cdev_add(&orangepi_uart_dev.cdev, orangepi_uart_dev.devno, 1);

    /* 配置uart外设时钟 */
    uart_clk_cfg = ioremap(CCU_BASE_ADDR + UART_BGR_REG_OFFSET, 4);
    *uart_clk_cfg |= 0x01<<3;

    /* 配置gpio */
    gpio_cfg2 =ioremap(PIO_BASE_ADDR + PIO_CFG2_OFFSET, 4);
    // 0x04<<28
    *gpio_cfg2 &= (~(0x0000000F << 28));
    *gpio_cfg2 |= 0x00000004 << 28;

    gpio_cfg3 = ioremap(PIO_BASE_ADDR + PIO_CFG3_OFFSET, 4);
    *gpio_cfg3 &= (~(0x0000000F));
    *gpio_cfg3 |= 0x00000004;

    /* 配置uart */
    uart_lcr = ioremap(UART3_BASS_ADDR + UART_LCR_OFFSET, 4);
    // 波特率配置为115200   baud rate = (serial clock freq) / (16 * dividor)
    // 115200 = 24000000 / (16 * [13])
    /* 选中DLLR和DLHR */
    *uart_lcr |= (0x01<<7);
    apb2_clk = ioremap(APB2_CFG_ADDR, 4);
    printk(KERN_INFO "APB2 CONFIG REG: 0x%x\n", *apb2_clk);
    uart3_reg0 = ioremap(UART3_BASS_ADDR, 4);
    uart3_reg1 = ioremap(UART3_BASS_ADDR + 0x04, 4);
    *uart3_reg0 &= 0x00; 
    *uart3_reg1 &= 0x00; 
    *uart3_reg0 |= 0x0D;

    //  选中RBR THR 和IER
    *uart_lcr &= ~(0x00000001 << 7);

    // 停止位 1bit
    *uart_lcr &= ~(0x00000001 << 2);

    // 数据位 8bit
    *uart_lcr |= 0x03;

    // 校验位 不启用
    *uart_lcr &= ~(0x00000001 << 3);

    // 硬件控制

    cls = class_create(THIS_MODULE, "orangepi_calss");
    dev = device_create(cls, NULL, orangepi_uart_dev.devno, NULL, "orangepi_uart%d", 0);

    return 0;
}
module_init(orangepi_uart_init);

static void __exit orangepi_uart_exit(void)
{
    device_destroy(cls, orangepi_uart_dev.devno);
    class_destroy(cls);
    cdev_del(&orangepi_uart_dev.cdev);
    unregister_chrdev_region(orangepi_uart_dev.devno, 1);

    iounmap(uart_clk_cfg);
    iounmap(gpio_cfg2);
    iounmap(gpio_cfg3);
    iounmap(uart_lcr);
    iounmap(apb2_clk);
    iounmap(uart3_reg0);
    iounmap(uart3_reg1);
}
module_exit(orangepi_uart_exit);


MODULE_LICENSE("GPL v2");
