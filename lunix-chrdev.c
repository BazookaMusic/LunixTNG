/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * DRAGONAS SOTIRIS 03114187
 * ZAROGIANNIS GIANNIS 03114186
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	
	WARN_ON ( !(sensor = state->sensor));
	/* ? */

	debug("buffTime=%d\tsensorTime=%d\n", state->buf_timestamp, sensor->msr_data[state->type]->last_update);
	return state->buf_timestamp != sensor->msr_data[state->type]->last_update;
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	
	debug("entering\n");

	sensor = state->sensor;

	uint32_t newValue;

	unsigned long flags;

	if(lunix_chrdev_state_needs_refresh(state))
	{
		spin_lock_irqsave(&sensor->lock, flags);
		newValue = sensor->msr_data[state->type]->values[0];
		state->buf_timestamp = sensor->msr_data[state->type]->last_update;
		spin_unlock_irqrestore(&sensor->lock, flags);
		debug("newValue=%d\ttime=%d", newValue, state->buf_timestamp);
	}
	else
	{
		debug("same timestamp\n");
		return -EAGAIN;
	}

	if(state->output_operation == COOKED)
	{
		long fixedValue;
		if(state->type == BATT)
		{
			fixedValue = lookup_voltage[newValue];
		}
		else if(state->type == TEMP)
		{
			fixedValue = lookup_temperature[newValue];
		}
		else
		{
			fixedValue = lookup_light[newValue];
		}
		long intV = fixedValue / 1000;
		long decV = fixedValue % 1000;

		debug("cookedValue=%d.%d\n", intV, decV);

		int length = sprintf(state->buf_data, "%d.%03d\n", intV, decV);
		state->buf_lim = length;
	}
	else
	{
		debug("rawValue=%d\n", newValue);
		int length = sprintf(state->buf_data, "%d\n", newValue);
		state->buf_lim = length;
	}

	debug("leaving\n");
	return 0;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	/* ? */
	int ret;

	debug("entering\n");
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */

	int minor;
	minor = iminor(inode);
	int sensorNum = minor / 8;
	int type = minor % 8;
	
	/* Allocate a new Lunix character device private state structure */
	/* ? */

	struct lunix_chrdev_state_struct *state = 
					kzalloc(sizeof(struct lunix_chrdev_state_struct), GFP_KERNEL);
	sema_init(&state->lock, 1);
	state->sensor = &lunix_sensors[sensorNum];
	state->type = type;
	state->output_operation = COOKED;

	filp->private_data = state;

	ret = 0;

out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	/* ? */

	debug("releasing\n");
	kfree(filp->private_data);
	debug("released\n");

	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/* Why? */

	struct lunix_chrdev_state_struct *state;
	state = filp->private_data;

	if (down_interruptible(&state->lock))
		return -ERESTARTSYS;

	if(cmd == COOKED)
	{
		state->output_operation = COOKED;
	}
	else if(cmd == RAW)
	{
		state->output_operation = RAW;
	}
	else
	{
		up(&state->lock);
		return -EINVAL;
	}

	up(&state->lock);

	return 0;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	debug("entering\n");

	ssize_t ret;

	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

	if (down_interruptible(&state->lock))
		return -ERESTARTSYS;

	if (*f_pos == 0) {
		while (lunix_chrdev_state_update(state) == -EAGAIN) {
			
			debug("inside while loop\n");

			up(&state->lock); /* release the lock */

			if (filp->f_flags & O_NONBLOCK)
			{
				debug("NON BLOCKING\n");
				if (down_interruptible(&state->lock))
					return -ERESTARTSYS;
				continue;
			}

			if (wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(state)))
			{
				debug("signal received 1\n");
				return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
			}
			if (down_interruptible(&state->lock))
			{
				debug("signal received 2\n");
				return -ERESTARTSYS;
			}

			debug("exiting while loop\n");
		}
	}

	state->buf_data[state->buf_lim] = '\0';
	debug("bufData=%s\tbufLim=%d\n", state->buf_data, state->buf_lim);
	
	cnt = min(cnt, (size_t)(state->buf_lim - *f_pos));

	if (copy_to_user(usrbuf, state->buf_data + *f_pos, cnt))
	{
		up (&state->lock);

		debug("copy to user error\n");
		
		return -EFAULT;
	}

	if(*f_pos + cnt < state->buf_lim)
		*f_pos += cnt;
	else
		/* Auto-rewind on EOF mode */
		*f_pos = 0;

	ret = cnt;

out:

	/* Unlock? */
	up(&state->lock);

	debug("leaving\n");

	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops = 
{
    .owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
	
	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;
	
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	/* ? */
	/* register_chrdev_region? */

	ret = register_chrdev_region(LUNIX_CHRDEV_MAJOR, lunix_minor_cnt, "LunixSensors");

	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}	
	/* ? */
	/* cdev_add? */

	ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt);

	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
		
	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
