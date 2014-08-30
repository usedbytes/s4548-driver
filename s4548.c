/*
 *  s4548.c
 *
 *  Copyright (C) 2012 Brian Starkey
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Driver for S-4548ATC i2c LCD (101x40px)
 *
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include <linux/init.h>
#include <linux/errno.h>
#include <linux/bcd.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <linux/random.h>
#include <linux/mman.h>

#include "s4548.h"


#define S4548_NAME "s4548"

#define S4548_MAX_DEVICES 5

/*
static const unsigned char * boot_msg[] = {
    0x24, 0x54, 0x54, 0x48, 0x00, 0x38, 0x05, 0x05, 0x3e, 0x00, 0x14, 0x34, 0x2c, 0x28, 0x00, 0x20, 0x78,
    0x24, 0x00, 0x18, 0x2c, 0x34, 0x10, 0x00, 0x3c, 0x20, 0x3c, 0x20, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x7c, 0x54, 0x54, 0x28, 0x00, 0x18, 0x24, 0x24, 0x18, 0x00, 0x18, 0x24, 0x24, 0x18, 0x00, 0x20,
    0x78, 0x24, 0x00, 0x5c, 0x00, 0x3c, 0x20, 0x20, 0x1c, 0x00, 0x18, 0x25, 0x25, 0x3e, 0x00, 0x02, 0x04,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x48, 0x48, 0x30, 0x00, 0x7c, 0x00, 0x18, 0x2c, 0x34, 0x10, 0x00,
    0x18, 0x24, 0x24, 0x3c, 0x00, 0x14, 0x34, 0x2c, 0x28, 0x00, 0x18, 0x2c, 0x34, 0x10, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x78, 0x04, 0x38, 0x04, 0x78, 0x00, 0x18, 0x24, 0x24, 0x3c, 0x00, 0x5c, 0x00, 0x20,
    0x78, 0x24, 0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00
};
*/
static int s4548_num_screens = 0;
static int s4548_major;
static struct class * s4548_class = NULL;

static unsigned int s4548_rate = CONFIG_S4548_RATE;
module_param(s4548_rate, uint, S_IRUGO);
MODULE_PARM_DESC(s4548_rate, "Refresh rate (hertz)");

static unsigned char s4548_page_buffer[S4548_PAGE_WIDTH + 2];

/* S4548 Data Structure (per screen) */

struct s4548 {
    int major;
    int screen_num;
    
    unsigned char cache[S4548_NUM_PAGES][S4548_PAGE_WIDTH];
	unsigned char * buffer;
    unsigned char auto_update;
    unsigned char in_use;
    
    struct i2c_client * client;
    
    struct delayed_work work_item;
    struct cdev cdev;
};

/* Screen writing and control */

static int s4548_send_command(struct s4548 * screen, unsigned char cmd)
{
    struct i2c_msg msg = {
        .addr = screen->client->addr,
        .flags = I2C_M_IGNORE_NAK,
        .buf = &cmd,
        .len = 1
    };

    return i2c_transfer(screen->client->adapter, &msg, 1) == 1 ? 0 : -1;
}

static int s4548_write_page(struct s4548 * screen, unsigned char * page,
                            unsigned char pageno)
{
    struct i2c_msg msg = {
        .addr = screen->client->addr,
        .flags = I2C_M_IGNORE_NAK,
        .buf = s4548_page_buffer,
        .len = S4548_PAGE_WIDTH + 2
    };

    s4548_page_buffer[0] = S4548_CMD_SET | ((pageno % 5) << 5);
    s4548_page_buffer[1] = 0x00;
    
    memcpy(s4548_page_buffer + 2, page, S4548_PAGE_WIDTH);
    
    return i2c_transfer(screen->client->adapter, &msg, 1);    
}

static void s4548_clear(struct s4548 * screen)
{
    char blank_page[S4548_PAGE_WIDTH] = { 0x0 };
    int i;
    
    for (i = 0; i < S4548_NUM_PAGES; i++) {
        s4548_write_page(screen, blank_page, i);
    }
}

/* Update Work */

static struct workqueue_struct *s4548_workqueue;

static void s4548_queue(struct s4548 * screen)
{
	queue_delayed_work(s4548_workqueue, &screen->work_item,
		HZ / s4548_rate);
}

static void s4548_update(struct work_struct *work)
{
	unsigned short i, offset;
    
    struct s4548 * screen = container_of((struct delayed_work *)work, 
                                            struct s4548, work_item);
    
	for (i = 0; i < S4548_NUM_PAGES; i++) {
        offset = i * S4548_PAGE_WIDTH;
        if (memcmp(screen->cache[i], 
                   screen->buffer + offset, S4548_PAGE_WIDTH)) {
            //printk(KERN_ALERT "Page %i changed. Updating\n", i);
            /* Transpose? 
            for (j = 0; j < S4548_PAGE_WIDTH; j++) {
                for (k = 0; k < S4548_NUM_PAGES; k++) {
                    *((char *)screen->cache + (k * S4548_PAGE_WIDTH) + (S4548_PAGE_WIDTH - j - 1)) = *(screen->buffer + (j * S4548_NUM_PAGES) + k);
                }
            }
            */
            
            /* Don't transpose */
            memcpy(screen->cache[i], screen->buffer + offset, 
                    S4548_PAGE_WIDTH);
            
            s4548_write_page(screen, screen->cache[i], S4548_NUM_PAGES - i - 1);
        }
    }
    
	if (screen->auto_update) s4548_queue(screen);
}

/* Char Device */
ssize_t s4548_fread(struct file *filp, char __user *buf, size_t count,
                loff_t *f_pos);
ssize_t s4548_fwrite(struct file *filp, const char __user *buf, size_t count,
                loff_t *f_pos);
int s4548_fopen(struct inode *inode, struct file *filp);
int s4548_frelease(struct inode *inode, struct file *filp);
loff_t s4548_llseek(struct file *filp, loff_t off, int from);
long s4548_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
int s4548_mmap(struct file *filp, struct vm_area_struct *vma);

struct file_operations s4548_fops = {
	.owner = THIS_MODULE,
    .read = s4548_fread,
	.write = s4548_fwrite,
	.open = s4548_fopen,
	.release = s4548_frelease,
    .llseek = s4548_llseek,
    .unlocked_ioctl = s4548_ioctl,
    .mmap = s4548_mmap
};

static int s4548_setup_cdev(struct s4548 *screen)
{
    struct device * device;
    int err, devno = MKDEV(screen->major, screen->screen_num);
    
    cdev_init(&screen->cdev, &s4548_fops);
    
    screen->cdev.owner = THIS_MODULE;
    screen->cdev.ops = &s4548_fops;
    
    err = cdev_add (&screen->cdev, devno, 1);
    /* Fail gracefully if need be */
    if (err) {
        printk(KERN_NOTICE "Error %d adding s4548-%d", err, screen->screen_num);
        return err;
    }
    
    printk(KERN_INFO S4548_NAME " : Character device registered, "
        "Major: %i, Minor: %i\n", screen->major, screen->screen_num);

    device = device_create(s4548_class, NULL, /* no parent device */ 
        devno, NULL, /* no additional data */
        S4548_NAME "-%d", screen->screen_num);
        
    if (IS_ERR(device)) {
        err = PTR_ERR(device);
        printk(KERN_WARNING S4548_NAME " : Error %d while trying to create %s-%d",
        err, S4548_NAME, screen->screen_num);
        cdev_del(&screen->cdev);
        return err;
    }
    
    return 0;
}

int s4548_fopen(struct inode *inode, struct file *filp)
{
    struct s4548 *screen; /* device information */
    screen = container_of(inode->i_cdev, struct s4548, cdev);
    if (screen->in_use) {
        return -EBUSY;
    } else {
        screen->in_use = 1;
        screen->auto_update = 1;
        s4548_queue(screen);
    }
    filp->private_data = screen; /* for other methods */

    /* now trim to 0 the length of the device if open was write-only */
    printk(KERN_ALERT "Open flags were: %i\n", filp->f_flags);
    return 0;          /* success */
}

int s4548_frelease(struct inode *inode, struct file *filp)
{
    struct s4548 *screen = filp->private_data;
    
    screen->auto_update = 0;
    s4548_queue(screen);
    screen->in_use = 0;
    return 0;
}

ssize_t s4548_fread(struct file *filp, char __user *buf, size_t count,
                loff_t *f_pos)
{
    struct s4548 *screen = filp->private_data; 
    ssize_t retval = 0;
    if (*f_pos + count > (S4548_PAGE_WIDTH * S4548_NUM_PAGES))
        count = (S4548_PAGE_WIDTH * S4548_NUM_PAGES) - *f_pos;
    if (copy_to_user(buf, screen->buffer + *f_pos, count)) {
        retval = -EFAULT;
        goto out;
    }
    *f_pos += count;
    retval = count;
    
  out:
    return retval;
}

ssize_t s4548_fwrite(struct file *filp, const char __user *buf, size_t count,
                loff_t *f_pos)
{
    struct s4548 *screen = filp->private_data;
    ssize_t retval = -ENOMEM; /* value used in "goto out" statements */
    if (*f_pos + count > (S4548_PAGE_WIDTH * S4548_NUM_PAGES))
        count = (S4548_PAGE_WIDTH * S4548_NUM_PAGES) - *f_pos;

    if (!count) return -ENOSPC;
    
    if (copy_from_user(screen->buffer + *f_pos, buf, count)) {
        retval = -EFAULT;
        goto out;
    }
    *f_pos += count;
    retval = count;
    
    if (!screen->auto_update)
        s4548_update((struct work_struct *)&screen->work_item);
    
  out:
    return retval;
}

loff_t s4548_llseek(struct file *filp, loff_t off, int from)
{
    loff_t newpos;

    switch(from) {
      case 0: /* SEEK_SET */
        newpos = off;
        break;

      case 1: /* SEEK_CUR */
        newpos = filp->f_pos + off;
        break;

      case 2: /* SEEK_END */
        newpos = S4548_SIZE + off;
        break;

      default: /* can't happen */
        return -EINVAL;
    }
    if ((newpos < 0) || (newpos > S4548_SIZE)) return -EINVAL;
    filp->f_pos = newpos;
    return newpos;
}

long s4548_ioctl (struct file *filp, unsigned int cmd, unsigned long arg) {
     struct s4548 *screen = filp->private_data;
     return s4548_send_command(screen, _IOC_NR(cmd));
}

int s4548_mmap(struct file *filp, struct vm_area_struct *vma) {
    
    struct s4548 * screen = filp->private_data;
    
    return vm_insert_page(vma, vma->vm_start, virt_to_page(screen->buffer));
}

/* i2c Client */

void s4548_shutdown(struct i2c_client * client) {
    struct s4548 * screen = i2c_get_clientdata(client);
    s4548_send_command(screen, S4548_CMD_OFF);
}

static int s4548_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
	struct s4548 *screen;
    int ret;
    
    //printk(KERN_ALERT "s4548_probe\n");
	if (s4548_num_screens >= S4548_MAX_DEVICES) {
        printk(KERN_ERR S4548_NAME ": ERROR: "
			"Maximum number of devices reached\n");
        return -ENODEV;
    }
    
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C /*| I2C_FUNC_PROTOCOL_MANGLING*/))
		return -ENODEV;
    
    screen = kzalloc(sizeof(struct s4548), GFP_KERNEL);
	if (!screen)
		return -ENOMEM;
    
    screen->buffer = (unsigned char *) get_zeroed_page(GFP_KERNEL);
	if (screen->buffer == NULL) {
		printk(KERN_ERR S4548_NAME ": ERROR: "
			"can't get a free page\n");
		ret = -ENOMEM;
	}
    
    //printk(KERN_ALERT "Buffer is at: %p\n", screen->buffer);

    screen->screen_num = s4548_num_screens;
    screen->in_use = 0;
    screen->major = s4548_major;    
    
    i2c_set_clientdata(client, screen);
    screen->client = client;
    
    //work = (struct s4548_work_t *)kmalloc(sizeof(struct s4548_work_t), GFP_KERNEL);
    //if (work) {
    INIT_DELAYED_WORK(&screen->work_item, s4548_update);
    
    
    s4548_clear(screen);
    s4548_setup_cdev(screen);
    
  
    s4548_num_screens++;
    return 0;
}

static int __devexit s4548_remove(struct i2c_client *client)
{

    //printk(KERN_ALERT "s4548_remove\n");
	
    struct s4548 * screen = i2c_get_clientdata(client);
    //s4548_clear(screen);
    //s4548_remove_screen(screen);

    screen->auto_update = 0;
    s4548_send_command(screen, S4548_CMD_OFF);
    
    if (!cancel_delayed_work(&screen->work_item)) {
        flush_workqueue(s4548_workqueue);
    }

    device_destroy(s4548_class, MKDEV(screen->major, screen->screen_num));
    cdev_del(&screen->cdev);
    
    free_page((unsigned long) screen->buffer);
	kfree(screen);
    
	return 0;
}

static const struct i2c_device_id s4548_id[] = {
	{ "s4548", 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, s4548_id);

static struct i2c_driver s4548_driver = {
	.driver = {
		.name	= "s4548",
		.owner	= THIS_MODULE,
	},
	.probe		= s4548_probe,
	.remove		= __devexit_p(s4548_remove),
        .shutdown       = s4548_shutdown,
        //.suspend        = s4548_screen_off,
        //.resume         = s4548_screen_on,
    //.detect     = s4548_detect,
	.id_table	= s4548_id,
};

/* Init and Exit */

static void s4548_cleanup( void ) {
    
    destroy_workqueue(s4548_workqueue);   
    
    if (s4548_class)
        class_destroy(s4548_class);
    
    unregister_chrdev_region(MKDEV(s4548_major, 0), S4548_MAX_DEVICES);
    return;
}

static __init int s4548_init(void)
{
    int err = -EFAULT;
    dev_t dev = 0;
    
    s4548_workqueue = create_workqueue("s4548_queue");
    if (s4548_workqueue == NULL)
		goto exit;

    err = alloc_chrdev_region(&dev, 0, S4548_MAX_DEVICES, S4548_NAME);
    if (err < 0) {
        printk(KERN_WARNING S4548_NAME " : alloc_chrdev_region() failed\n");
        return err;
    }
    s4548_major = MAJOR(dev);
    
    /* Create device class (before allocation of the array of devices) */
    s4548_class = class_create(THIS_MODULE, S4548_NAME);
    if (IS_ERR(s4548_class)) {
        err = PTR_ERR(s4548_class);
        goto exit;
    }
    
    return i2c_add_driver(&s4548_driver);

  exit:
    s4548_cleanup();
    return -EFAULT;
}

static __exit void s4548_exit(void)
{
    i2c_del_driver(&s4548_driver);
    s4548_cleanup();
}

module_init(s4548_init);
module_exit(s4548_exit);

MODULE_AUTHOR("Brian Starkey");
MODULE_DESCRIPTION("S-4548ATC LCD driver");
MODULE_LICENSE("GPL");
