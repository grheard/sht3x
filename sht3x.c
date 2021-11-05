#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include "sht3x_ioctl.h"


MODULE_LICENSE("GPL");
MODULE_AUTHOR("grheard <grheard@gmail.com>");
MODULE_DESCRIPTION("SHT3x I2C driver");
MODULE_VERSION("0.1");


#define PRINTK(x,y, ...) printk(x "SHT3X: %s "  y,__FUNCTION__, ## __VA_ARGS__)

#define I2C_BUS_AVAILABLE   (          1 )              // I2C Bus available in our Raspberry Pi
#define SLAVE_DEVICE_NAME   (    "SHT3X" )              // Device and Driver Name
#define SLAVE_ADDR          (       0x44 )              // SHT31 default slave id
#define SLAVE_ADDR_ALT      (       0x45 )              // SHT31 alternate slave id

#define COMMAND_LEN 2
#define STATUS_LEN 3
#define MEASUREMENT_LEN 6


static unsigned char cmd_break[COMMAND_LEN] = {0x30,0x93};

static unsigned char cmd_status_read[COMMAND_LEN] = {0xf3,0x2d};
static unsigned char cmd_status_clear[COMMAND_LEN] = {0x30,0x41};

static unsigned char cmd_heater_enable[COMMAND_LEN] = {0x30,0x6d};
static unsigned char cmd_heater_disable[COMMAND_LEN] = {0x30,0x66};

static unsigned char cmd_singleshot_low[COMMAND_LEN] = {0x2c,0x10};
static unsigned char cmd_singleshot_med[COMMAND_LEN] = {0x2c,0x0D};
static unsigned char cmd_singleshot_hi[COMMAND_LEN] = {0x2c,0x06};

static unsigned char cmd_periodic_0p5_low[COMMAND_LEN] = {0x20,0x2f};
static unsigned char cmd_periodic_0p5_med[COMMAND_LEN] = {0x20,0x24};
static unsigned char cmd_periodic_0p5_hi[COMMAND_LEN] = {0x20,0x32};

static unsigned char cmd_periodic_1_low[COMMAND_LEN] = {0x21,0x2d};
static unsigned char cmd_periodic_1_med[COMMAND_LEN] = {0x21,0x26};
static unsigned char cmd_periodic_1_hi[COMMAND_LEN] = {0x21,0x30};

static unsigned char cmd_periodic_2_low[COMMAND_LEN] = {0x22,0x2b};
static unsigned char cmd_periodic_2_med[COMMAND_LEN] = {0x22,0x20};
static unsigned char cmd_periodic_2_hi[COMMAND_LEN] = {0x22,0x36};

static unsigned char cmd_periodic_4_low[COMMAND_LEN] = {0x23,0x29};
static unsigned char cmd_periodic_4_med[COMMAND_LEN] = {0x23,0x22};
static unsigned char cmd_periodic_4_hi[COMMAND_LEN] = {0x23,0x34};

static unsigned char cmd_periodic_10_low[COMMAND_LEN] = {0x27,0x2a};
static unsigned char cmd_periodic_10_med[COMMAND_LEN] = {0x27,0x21};
static unsigned char cmd_periodic_10_hi[COMMAND_LEN] = {0x27,0x37};

static unsigned char* measurement_commands[18] = {
    cmd_singleshot_low,
    cmd_singleshot_med,
    cmd_singleshot_hi,
    cmd_periodic_0p5_low,
    cmd_periodic_0p5_med,
    cmd_periodic_0p5_hi,
    cmd_periodic_1_low,
    cmd_periodic_1_med,
    cmd_periodic_1_hi,
    cmd_periodic_2_low,
    cmd_periodic_2_med,
    cmd_periodic_2_hi,
    cmd_periodic_4_low,
    cmd_periodic_4_med,
    cmd_periodic_4_hi,
    cmd_periodic_10_low,
    cmd_periodic_10_med,
    cmd_periodic_10_hi
};

static int measurement_mode = SHT3X_SINGLE_SHOT_LOW;
static int slave_addr = SLAVE_ADDR;

static unsigned char measurement[6];
static int measurement_count = 0;
static int measurement_status = 0;
DEFINE_MUTEX(measurement_mutex);
DECLARE_WAIT_QUEUE_HEAD(measurement_wait);

static int periodic_time_ms(void);
static int break_and_cleanup(void);


static int i2c_probe(struct i2c_client* client, const struct i2c_device_id* id);
static int i2c_remove(struct i2c_client* client);

static struct i2c_adapter* sht_i2c_adapter = NULL;
static struct i2c_client* sht_i2c_client  = NULL;
static struct i2c_board_info sht_i2c_board_info = {I2C_BOARD_INFO(SLAVE_DEVICE_NAME, SLAVE_ADDR)};

static const struct i2c_device_id i2c_id[] = {
        { SLAVE_DEVICE_NAME, 0 },
        { }
};
MODULE_DEVICE_TABLE(i2c, i2c_id);

static struct i2c_driver sht_i2c_driver = {
    .driver = {
        .name = SLAVE_DEVICE_NAME,
        .owner = THIS_MODULE,
    },
    .probe = i2c_probe,
    .remove = i2c_remove,
    .id_table = i2c_id,
};


static int __init sht3x_driver_init(void);
static void __exit sht3x_driver_exit(void);

module_init(sht3x_driver_init);
module_exit(sht3x_driver_exit);


static dev_t dev = 0;
static struct class* dev_class;
static struct cdev dev_cdev;
static char deventry[] = "sht3x  ";
static atomic_t dev_use_count = ATOMIC_INIT(0);

static int fops_open(struct inode* inode, struct file* file);
static int fops_release(struct inode* inode, struct file* file);
static ssize_t fops_read(struct file* filp, char __user* buf, size_t len,loff_t* off);
//static ssize_t fops_write(struct file* filp, const char* buf, size_t len, loff_t* off);
static long fops_ioctl(struct file* file, unsigned int cmd, unsigned long arg);
//static unsigned int fops_poll(struct file* filp, struct poll_table_struct* wait);

static struct file_operations fops =
{
        .owner          = THIS_MODULE,
        .read           = fops_read,
        //.write          = fops_write,
        .open           = fops_open,
        .unlocked_ioctl = fops_ioctl,
        .release        = fops_release,
        //.poll           = fops_poll
};


static struct timer_list periodic_timer;
static void periodic_timer_callback(struct timer_list * data);

static void worker_read_measurement(struct work_struct *work);
DECLARE_WORK(worker,worker_read_measurement);
static struct workqueue_struct* workqueue = 0;


module_param(measurement_mode, int, S_IRUSR);
module_param(slave_addr, int, S_IRUSR);


static int __init sht3x_driver_init(void)
{
    if (slave_addr != SLAVE_ADDR && slave_addr != SLAVE_ADDR_ALT)
    {
        PRINTK(KERN_ERR,"Slave addr (%02X) is invalid.\n",slave_addr);
        return -1;
    }

    workqueue = create_singlethread_workqueue(SLAVE_DEVICE_NAME);
    if (!workqueue)
    {
        PRINTK(KERN_ERR,"Cannot create workqueue.\n");
        return -1;
    }

    sht_i2c_board_info.addr = slave_addr;

    sht_i2c_adapter = i2c_get_adapter(I2C_BUS_AVAILABLE);

    if (sht_i2c_adapter == NULL)
    {
        PRINTK(KERN_ERR,"Could not get i2c adapter for bus #%d\n",I2C_BUS_AVAILABLE);
        return -1;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,8,0)
    sht_i2c_client = i2c_new_client_device(sht_i2c_adapter, &sht_i2c_board_info);

    if (sht_i2c_client == ERR_PTR)
#else
    sht_i2c_client = i2c_new__device(sht_i2c_adapter, &sht_i2c_board_info);

    if (sht_i2c_client == NULL)
#endif
    {
        PRINTK(KERN_ERR,"Could not create new i2c device.\n");
        return -1;
    }

    i2c_add_driver(&sht_i2c_driver);
    i2c_put_adapter(sht_i2c_adapter);

    // Allocating Major number
    if (alloc_chrdev_region(&dev, 0, 1, "sht3x") < 0)
    {
        PRINTK(KERN_ERR,"Cannot allocate major number\n");
        goto r_i2c;
    }

    // Creating cdev structure
    cdev_init(&dev_cdev,&fops);

    // Adding character device to the system
    if (cdev_add(&dev_cdev,dev,1) < 0)
    {
        PRINTK(KERN_INFO,"Cannot add the device to the system\n");
        goto r_class;
    }

    // Creating struct class
    if ((dev_class = class_create(THIS_MODULE,"sht3x_class")) == NULL)
    {
        PRINTK(KERN_INFO,"Cannot create the struct class\n");
        goto r_class;
    }

    // Creating device
    snprintf(&deventry[5],3,"%02X",slave_addr);
    if (device_create(dev_class,NULL,dev,NULL,deventry) == NULL)
    {
        PRINTK(KERN_INFO,"Cannot create the Device 1\n");
        goto r_device;
    }

    timer_setup(&periodic_timer, periodic_timer_callback, 0);

    PRINTK(KERN_INFO,"Driver initialized for slave at addr 0x%02X\n",slave_addr);
    return 0;

r_device:
    class_destroy(dev_class);
r_class:
    unregister_chrdev_region(dev,1);
r_i2c:
    i2c_unregister_device(sht_i2c_client);
    i2c_del_driver(&sht_i2c_driver);

    return -1;
}


static void __exit sht3x_driver_exit(void)
{
    del_timer(&periodic_timer);

    device_destroy(dev_class,dev);
    class_destroy(dev_class);
    cdev_del(&dev_cdev);
    unregister_chrdev_region(dev, 1);

    i2c_unregister_device(sht_i2c_client);
    i2c_del_driver(&sht_i2c_driver);

    destroy_workqueue(workqueue);
}


static int i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    // Send a break in case a periodic measurement is running.
    i2c_master_send(client, cmd_break, COMMAND_LEN);
    // Clear the status
    i2c_master_send(client, cmd_status_clear, COMMAND_LEN);

    return 0;
}


static int i2c_remove(struct i2c_client *client)
{
    // Send a break in case a periodic measurement is running.
    i2c_master_send(sht_i2c_client, cmd_break, COMMAND_LEN);
    // Clear the status
    i2c_master_send(sht_i2c_client, cmd_status_clear, COMMAND_LEN);

    return 0;
}


static int fops_open(struct inode* inode, struct file* file)
{
    PRINTK(KERN_INFO,"%p\n",file);

    if (atomic_inc_return(&dev_use_count) == 1)
    {
        // Send a break in case a periodic measurement is running.
        i2c_master_send(sht_i2c_client, cmd_break, COMMAND_LEN);
        // Clear the status
        i2c_master_send(sht_i2c_client, cmd_status_clear, COMMAND_LEN);
    }

    file->private_data = (void *)measurement_count;
    return 0;
}


static int fops_release(struct inode* inode, struct file* file)
{
    PRINTK(KERN_INFO,"%p\n",file);

    if (atomic_dec_and_test(&dev_use_count))
    {
        if (measurement_mode != SHT3X_SINGLE_SHOT_LOW)
        {
            break_and_cleanup();
        }
    }

    return 0;
}


static long fops_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
    int i2cret;
    unsigned char status[STATUS_LEN];

    PRINTK(KERN_INFO,"ioctl cmd=%0X arg=%ld\n",cmd,arg);

    switch (cmd)
    {
        case SHT3X_BREAK:
            if (break_and_cleanup())
            {
                return -EIO;
            }
            break;

        case SHT3X_HEATER_CONTROL:
            switch (arg)
            {
                case SHT3X_HEATER_DISABLE:
                    i2cret = i2c_master_send(sht_i2c_client,cmd_heater_disable,COMMAND_LEN);
                    break;

                case SHT3X_HEATER_ENABLE:
                    i2cret = i2c_master_send(sht_i2c_client,cmd_heater_enable,COMMAND_LEN);
                    break;

                default:
                    return -EINVAL;
            }
            if (i2cret != COMMAND_LEN)
            {
                PRINTK(KERN_ERR,"Heater command %ld returned error %d\n",arg,i2cret);
                return -EIO;
            }
            break;

        case SHT3X_STATUS:
            switch (arg)
            {
                case SHT3X_STATUS_READ:
                    i2cret = i2c_master_send(sht_i2c_client,cmd_status_read,COMMAND_LEN);
                    if (i2cret != COMMAND_LEN)
                    {
                        PRINTK(KERN_ERR,"Status read command returned error %d\n",i2cret);
                        return -EIO;
                    }
                    i2cret = i2c_master_recv(sht_i2c_client,status,STATUS_LEN);
                    if (i2cret != COMMAND_LEN)
                    {
                        PRINTK(KERN_ERR,"Status read returned error %d\n",i2cret);
                        return -EIO;
                    }
                    return ((long)status[0] << 16) | ((long)status[1] << 8) | (long)status[2];

                case SHT3X_STATUS_CLEAR:
                    i2cret = i2c_master_send(sht_i2c_client,cmd_status_clear,COMMAND_LEN);
                    if (i2cret != COMMAND_LEN)
                    {
                        PRINTK(KERN_ERR,"Status clear command returned error %d\n",i2cret);
                        return -EIO;
                    }
                    break;

                default:
                    return -EINVAL;
            }
            break;

        case SHT3X_MEASUREMENT_MODE:
            if (arg >= sizeof(measurement_commands))
            {
                PRINTK(KERN_ERR,"Measurement mode value %ld is invalid.\n",arg);
                return -EINVAL;
            }
            else if (measurement_mode == arg)
            {
                return 0;
            }
            else if (measurement_mode != SHT3X_SINGLE_SHOT_LOW)
            {
                PRINTK(KERN_ERR,"Break must be sent before changing the measurement mode.\n");
                return -EINVAL;
            }

            measurement_mode = arg;
            PRINTK(KERN_INFO,"Measurement mode changed to %d\n",measurement_mode);

            if (arg == SHT3X_SINGLE_SHOT_LOW
                || arg == SHT3X_SINGLE_SHOT_MED
                || arg == SHT3X_SINGLE_SHOT_HIGH)
            {
                // Sending an i2c command isn't needed, the read will execute single-shot command in-line.
                break;
            }

            i2cret = i2c_master_send(sht_i2c_client,measurement_commands[arg],COMMAND_LEN);
            if (i2cret != COMMAND_LEN)
            {
                PRINTK(KERN_ERR,"Measurement command %ld returned error %d\n",arg,i2cret);
                return -EIO;
            }

            PRINTK(KERN_INFO,"timer set for %d\n",periodic_time_ms());
            mod_timer(&periodic_timer,jiffies + msecs_to_jiffies(periodic_time_ms()));
            break;

        default:
            PRINTK(KERN_ERR,"Invalid ioctl %0X\n",cmd);
            return -EINVAL;
    }

    return 0;
}


static ssize_t fops_read(struct file* filp, char __user* buf, size_t len,loff_t* off)
{
    unsigned char single_measurement[MEASUREMENT_LEN];
    int _count;

    // Adjust the read length to a maximum of a single measurement
    if (len > MEASUREMENT_LEN) len = MEASUREMENT_LEN;

    if (measurement_mode == SHT3X_SINGLE_SHOT_LOW
        || measurement_mode == SHT3X_SINGLE_SHOT_MED
        || measurement_mode == SHT3X_SINGLE_SHOT_HIGH)
    {
        // Single shot measurement

        int ret = i2c_master_send(sht_i2c_client,measurement_commands[measurement_mode],COMMAND_LEN);
        if (ret != COMMAND_LEN)
        {
            PRINTK(KERN_ERR,"i2c_master_send error %d\n",ret);
            return -EIO;
        }

        ret = i2c_master_recv(sht_i2c_client,single_measurement,MEASUREMENT_LEN);
        if (ret != MEASUREMENT_LEN)
        {
            PRINTK(KERN_ERR,"i2c_master_recv error %d\n",ret);
            return -EIO;
        }

        if (copy_to_user(buf,single_measurement,len))
        {
            return -EFAULT;
        }

        return len;
    }

    // Periodic measurement handling.

    _count = (int)filp->private_data;

    mutex_lock(&measurement_mutex);
    while (_count == measurement_count && measurement_status == 0)
    {
        mutex_unlock(&measurement_mutex);

        if (filp->f_flags & O_NONBLOCK)
        {
            return -EAGAIN;
        }

        if (wait_event_interruptible(measurement_wait,(_count != measurement_count || measurement_status != 0)))
        {
            return -ERESTARTSYS;
        }

        mutex_lock(&measurement_mutex);
    }

    if (measurement_status != 0)
    {
        mutex_unlock(&measurement_mutex);
        return -EIO;
    }

    if (copy_to_user(buf,measurement,len))
    {
        mutex_unlock(&measurement_mutex);
        return -EFAULT;
    }

    _count = measurement_count;

    mutex_unlock(&measurement_mutex);

    filp->private_data = (void *)_count;

    return len;
}


void periodic_timer_callback(struct timer_list * data)
{
    if (!queue_work(workqueue,&worker))
    {
        PRINTK(KERN_WARNING,"worker still queued.");
    }
}


static void worker_read_measurement(struct work_struct* work)
{
    static int read_failed = 0;
    int i2cret;

    unsigned char sensor_data[MEASUREMENT_LEN];

    i2cret = i2c_master_recv(sht_i2c_client,sensor_data,MEASUREMENT_LEN);
    if (i2cret != MEASUREMENT_LEN)
    {
        read_failed++;
        PRINTK(KERN_WARNING,"Periodic read failed (%d) with %d\n",read_failed,i2cret);
        mod_timer(&periodic_timer,jiffies + msecs_to_jiffies(100));

        mutex_lock(&measurement_mutex);
        measurement_status = 1;
        mutex_unlock(&measurement_mutex);
    }
    else
    {
        read_failed = 0;

        mutex_lock(&measurement_mutex);
        measurement_count++;
        memcpy(measurement,sensor_data,MEASUREMENT_LEN);
        mutex_unlock(&measurement_mutex);

        mod_timer(&periodic_timer,jiffies + msecs_to_jiffies(periodic_time_ms()));
    }
    wake_up_interruptible(&measurement_wait);
}


static int break_and_cleanup(void)
{
    int i2cret;

    del_timer(&periodic_timer);
    cancel_work_sync(&worker);

    mutex_lock(&measurement_mutex);
    measurement_status = 0;
    mutex_unlock(&measurement_mutex);

    measurement_mode = SHT3X_SINGLE_SHOT_LOW;

    i2cret = i2c_master_send(sht_i2c_client,cmd_break,COMMAND_LEN);
    if (i2cret != COMMAND_LEN)
    {
        PRINTK(KERN_ERR,"Break command returned error %d\n",i2cret);
        return 1;
    }

    return 0;
}


static int periodic_time_ms(void)
{
    int time_ms;
    switch (measurement_mode)
    {
        case SHT3X_PERIODIC_0P5_LOW:
        case SHT3X_PERIODIC_0P5_MED:
        case SHT3X_PERIODIC_0P5_HIGH:
            time_ms = 2000;
            break;
        case SHT3X_PERIODIC_1_LOW:
        case SHT3X_PERIODIC_1_MED:
        case SHT3X_PERIODIC_1_HIGH:
            time_ms = 1000;
            break;
        case SHT3X_PERIODIC_2_LOW:
        case SHT3X_PERIODIC_2_MED:
        case SHT3X_PERIODIC_2_HIGH:
            time_ms = 500;
            break;
        case SHT3X_PERIODIC_4_LOW:
        case SHT3X_PERIODIC_4_MED:
        case SHT3X_PERIODIC_4_HIGH:
            time_ms = 250;
            break;
        case SHT3X_PERIODIC_10_LOW:
        case SHT3X_PERIODIC_10_MED:
        case SHT3X_PERIODIC_10_HIGH:
            time_ms = 100;
            break;

        default:
            PRINTK(KERN_ERR,"How the hell did we get here? %d\n",measurement_mode);
            break;
    }

    return time_ms;
}
