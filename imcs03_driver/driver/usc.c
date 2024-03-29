/*
 * USB H8 based motor controller driver 
 *
 * based on USB Skeleton driver.
 */
/*
 * USB Skeleton driver - 2.0
 *
 * Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * This driver is based on the 2.6.3 version of drivers/usb/usb-skeleton.c 
 * but has been rewritten to be easy to read and use, as no locks are now
 * needed anymore.
 *
 */

#define DEBUG

//#include <linux/config.h>
//#include <linux/autoconf.h>			//kernel 2.6.18
#include "/usr/src/linux-headers-3.2.0-4-amd64/include/generated/autoconf.h"
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <asm/uaccess.h>
//#include <linux/usb.h>
#include "/usr/src/linux-headers-3.2.0-4-common/include/linux/usb.h"
// adding 20060831 abe
#include "usc.h"
#include "usensorc.h"
//#include "urbtc.h"
//#include "urobotc.h"

static struct usb_device_id usc_table [] = {
	{ USB_DEVICE(0xff8, 0x0020) },	/* iXs Reserch version */
	{ }					/* Terminating entry */
};
MODULE_DEVICE_TABLE (usb, usc_table);

#define USB_USC_MINOR_BASE	133

/* our private defines. if this grows any larger, use your own .h file */
#define MAX_TRANSFER		(PAGE_SIZE - 512)
#define WRITES_IN_FLIGHT	8

// adding 20060831
#define EP_READREQ      1
#define EP_DDR          2
#define EP_READ         5
#define EP_DR           6

//#define EP_READREQ	1
//#define EP_SCMD		2
//#define EP_READ		5
//#define EP_CCMD		6

/* Structure to hold all of our device specific stuff */
struct usb_usc {
	struct usb_device *udev;
	struct usb_interface *interface;
	struct semaphore limit_sem;

	int read_status;
	unsigned char *readreq_buffer;
	size_t readreq_size;

	int readbuf_enabled;
	wait_queue_head_t readbuf_wait;
	struct urb *readbuf_urb;
	struct udata *readbuf_work;
	struct semaphore readbuf_sem;
	struct udata *readbuf_buffered;
	size_t readbuf_size;
	unsigned short readbuf_last_buffered; /* same to uin.time */
	unsigned short readbuf_last_read; /* same to uin.time */

	int write_status;
	unsigned char *write_counter_buffer;
	size_t write_counter_size;

	struct kref		kref;
};
#define to_usc_dev(d) container_of(d, struct usb_usc, kref)

static struct usb_driver usc_driver;

static void usc_delete(struct kref *kref)
{	
	struct usb_usc *dev = to_usc_dev(kref);

	usb_put_dev(dev->udev);
	kfree (dev->readreq_buffer);
	usb_free_urb(dev->readbuf_urb);
	kfree (dev->readbuf_work);
	kfree (dev->readbuf_buffered);
	kfree (dev);
}

static int usc_open(struct inode *inode, struct file *file)
{
	struct usb_usc *dev;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;

	subminor = iminor(inode);

	interface = usb_find_interface(&usc_driver, subminor);
	if (!interface) {
		err ("%s - error, can't find device for minor %d",
		     __FUNCTION__, subminor);
		retval = -ENODEV;
		goto exit;
	}

	dev = usb_get_intfdata(interface);
	if (!dev) {
		retval = -ENODEV;
		goto exit;
	}

	dev->read_status = USC_STATUS_READ_REQUEST;
	dev->readbuf_enabled = 0;
	dev->write_status = USC_STATUS_WRITE_DESIRE;

	/* increment our usage count for the device */
	kref_get(&dev->kref);

	/* save our object in the file's private structure */
	file->private_data = dev;

exit:
	return retval;
}

static int usc_release(struct inode *inode, struct file *file)
{
	struct usb_usc *dev = (struct usb_usc *)file->private_data;

	if (dev == NULL)
		return -ENODEV;

	if (dev->interface) {
		if (dev->readbuf_enabled)
			usb_kill_urb(dev->readbuf_urb);
	}
		
	/* decrement the count on our device */
	kref_put(&dev->kref, usc_delete);
	return 0;
}

//static void usc_read_bulk_callback(struct urb *urb, struct pt_regs *regs)
static void usc_read_bulk_callback(struct urb *urb)			//chg t.miyo
{
	struct usb_usc *dev = (struct usb_usc *)urb->context;
	int res;

	/* sync/async unlink faults aren't errors */
	if (urb->status && 
	    !(urb->status == -ENOENT || 
	      urb->status == -ECONNRESET ||
	      urb->status == -ESHUTDOWN)) {
		dbg("%s - nonzero read bulk status received: %d",
		    __FUNCTION__, urb->status);
		dev->readbuf_enabled = 0;
		return;
	}
	if (urb->actual_length > 0) {
		if (down_trylock(&dev->readbuf_sem) == 0) {
			memcpy(dev->readbuf_buffered, dev->readbuf_work, urb->actual_length);
			dev->readbuf_last_buffered = dev->readbuf_work->time;
			up(&dev->readbuf_sem);
			wake_up(&dev->readbuf_wait);
		}
	}
	if (dev->readbuf_enabled) {
		usb_fill_bulk_urb(dev->readbuf_urb, dev->udev,
				  usb_rcvbulkpipe(dev->udev, EP_READ),
				  dev->readbuf_urb->transfer_buffer,
				  dev->readbuf_urb->transfer_buffer_length,
				  usc_read_bulk_callback, dev);
		if ((res = usb_submit_urb(dev->readbuf_urb, GFP_ATOMIC))) {
			dev->readbuf_enabled = 0;
		}
	}
}

static ssize_t usc_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	struct usb_usc *dev = (struct usb_usc *)file->private_data;
	int retval = 0;
	int bytes_read;
	
	if (count != sizeof(struct udata))
		return -EINVAL;

	switch (dev->read_status) {
	case USC_STATUS_READ_REQUEST:
		/* do a blocking bulk read to get data from the device */
		retval = usb_bulk_msg(dev->udev,
				      usb_rcvbulkpipe(dev->udev, EP_READREQ),
				      dev->readreq_buffer,
				      min(dev->readreq_size, count),
				      &bytes_read, 10000);

		/* if the read was successful, copy the data to userspace */
		if (!retval) {
			if (copy_to_user(buffer, dev->readreq_buffer, bytes_read))
				retval = -EFAULT;
			else
				retval = bytes_read;
		}
		break;
	case USC_STATUS_READ_CONTINUOUS:
		if (dev->readbuf_enabled) {
			wait_event_interruptible(dev->readbuf_wait,
						 dev->readbuf_last_read != dev->readbuf_last_buffered || !dev->udev);
			if (signal_pending(current)) {
				retval = -EINTR;
				goto out;
			}
			if (!dev->udev) {
				retval = -ENODEV;
				goto out;
			}
			dev->readbuf_last_read = dev->readbuf_last_buffered;
			down(&dev->readbuf_sem);
			if (copy_to_user(buffer, dev->readbuf_buffered, dev->readbuf_size))
				retval = -EFAULT;
			else
				retval = dev->readbuf_size;
			up(&dev->readbuf_sem);
		} else {
			/* do a blocking bulk read to get data from the device */
			down(&dev->readbuf_sem);
			retval = usb_bulk_msg(dev->udev,
					      usb_rcvbulkpipe(dev->udev, EP_READ),
					      dev->readbuf_buffered,
					      min(dev->readbuf_size, count),
					      &bytes_read, 10000);

			/* if the read was successful, copy the data to userspace */
			if (!retval) {
				if (copy_to_user(buffer, dev->readbuf_buffered, bytes_read))
					retval = -EFAULT;
				else
					retval = bytes_read;
			}
			up(&dev->readbuf_sem);
		}
		break;
	default:
		/* should not occur */
		retval = -EINVAL;
		break;
	}
out:
	return retval;
}

//static void usc_write_bulk_callback(struct urb *urb, struct pt_regs *regs)
static void usc_write_bulk_callback(struct urb *urb)		//chg t.miyo
{
	struct usb_usc *dev = (struct usb_usc *)urb->context;

	/* sync/async unlink faults aren't errors */
	if (urb->status && 
	    !(urb->status == -ENOENT || 
	      urb->status == -ECONNRESET ||
	      urb->status == -ESHUTDOWN)) {
		dbg("%s - nonzero write bulk status received: %d",
		    __FUNCTION__, urb->status);
	}

	/* free up our allocated buffer */
	//usb_buffer_free(urb->dev, urb->transfer_buffer_length, 
	//		urb->transfer_buffer, urb->transfer_dma);
	usb_free_coherent(urb->dev, urb->transfer_buffer_length, 				//chg t.miyo
			urb->transfer_buffer, urb->transfer_dma);
	up(&dev->limit_sem);
}

static ssize_t usc_write_sync(struct file *file, const char *user_buffer, size_t count, loff_t *ppos)
{
	struct usb_usc *dev = (struct usb_usc *)file->private_data;
	int retval = 0;
	size_t writesize = min(count, dev->write_counter_size);
	int bytes_written;

	if (copy_from_user(dev->write_counter_buffer, user_buffer, writesize)) {
		retval = -EFAULT;
		goto error;
	}

	/* do a blocking bulk write to the device */
	retval = usb_bulk_msg(dev->udev,
			      usb_sndbulkpipe(dev->udev, EP_DR),
			      dev->write_counter_buffer,
			      writesize,
			      &bytes_written, 10000);

	if (!retval)
		retval = bytes_written;

error:
	return retval;
}

static ssize_t usc_write_async(struct file *file, const char *user_buffer, size_t count, loff_t *ppos)
{
	struct usb_usc *dev = (struct usb_usc *)file->private_data;
	int retval = 0;
	struct urb *urb = NULL;
	char *buf = NULL;
	size_t writesize = min(count, (size_t)MAX_TRANSFER);

	/* limit the number of URBs in flight to stop a user from using up all RAM */
	if (down_interruptible(&dev->limit_sem)) {
		retval = -ERESTARTSYS;
		goto exit;
	}

	/* create a urb, and a buffer for it, and copy the data to the urb */
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		retval = -ENOMEM;
		goto error;
	}

	//buf = usb_buffer_alloc(dev->udev, writesize, GFP_KERNEL, &urb->transfer_dma);
	buf = usb_alloc_coherent(dev->udev, writesize, GFP_KERNEL, &urb->transfer_dma);			//chg t.miyo
	if (!buf) {
		retval = -ENOMEM;
		goto error;
	}

	if (copy_from_user(buf, user_buffer, writesize)) {
		retval = -EFAULT;
		goto error;
	}

	/* initialize the urb properly */
	usb_fill_bulk_urb(urb, dev->udev,
			  usb_sndbulkpipe(dev->udev, EP_DDR),
			  buf, writesize, usc_write_bulk_callback, dev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	/* send the data out the bulk port */
	retval = usb_submit_urb(urb, GFP_KERNEL);
	if (retval) {
		err("%s - failed submitting write urb, error %d", __FUNCTION__, retval);
		goto error;
	}

	/* release our reference to this urb, the USB core will eventually free it entirely */
	usb_free_urb(urb);

exit:
	return writesize;

error:
	//usb_buffer_free(dev->udev, writesize, buf, urb->transfer_dma);
	usb_free_coherent(dev->udev, writesize, buf, urb->transfer_dma);		//chg t.miyo
	usb_free_urb(urb);
	up(&dev->limit_sem);
	return retval;
}

static ssize_t usc_write(struct file *file, const char *user_buffer, size_t count, loff_t *ppos)
{
	struct usb_usc *dev = (struct usb_usc *)file->private_data;

	/* verify that we actually have some data to write */
	if (count == 0)
		return 0;

	switch (dev->write_status) {
	case USC_STATUS_WRITE_COUNTER:
		return usc_write_sync(file, user_buffer, count, ppos);
	case USC_STATUS_WRITE_DESIRE:
		return usc_write_async(file, user_buffer, count, ppos);
	default:
		/* should not occur */
		return -EINVAL;
	}
}

/*static int
usc_ioctl(struct inode *inode, struct file *file,
	    unsigned int cmd, unsigned long arg)*/
static long
usc_ioctl( struct file *file,
	    unsigned int cmd, unsigned long arg)				//chg t.miyo

{
	struct usb_usc *dev = (struct usb_usc *)file->private_data;
	struct usb_device *udev = dev->udev;
	int retval = 0;

	switch(cmd) {
	case USC_GET_VENDOR:
		if(copy_to_user((int*)arg, &udev->descriptor.idVendor, sizeof(int)))
			return -EFAULT;
		break;
	case USC_GET_PRODUCT:
		if(copy_to_user((int*)arg, &udev->descriptor.idProduct, sizeof(int)))
			return -EFAULT;
		break;
	case USC_REQUEST_READ:
		dev->read_status = USC_STATUS_READ_REQUEST;
		break;
	case USC_CONTINUOUS_READ:
		dev->read_status = USC_STATUS_READ_CONTINUOUS;
		break;
	case USC_BUFREAD:
		if (dev->read_status != USC_STATUS_READ_CONTINUOUS)
			return -EINVAL;
		if (!dev->readbuf_enabled) {
			usb_fill_bulk_urb(dev->readbuf_urb, dev->udev,
					  usb_rcvbulkpipe(dev->udev, EP_READ),
					  dev->readbuf_urb->transfer_buffer,
					  dev->readbuf_urb->transfer_buffer_length,
					  usc_read_bulk_callback, dev);
			if (!(retval = usb_submit_urb(dev->readbuf_urb, GFP_KERNEL)))
				dev->readbuf_enabled = 1;
		}
		break;
	case USC_WAITREAD:
		dev->readbuf_enabled = 0;
		break;
	case USC_GET_READ_STATUS:
		if (copy_to_user((int*)arg, &dev->read_status, sizeof(int)))
			return -EFAULT;
		break;

	case USC_DR_SET:
		dev->write_status = USC_STATUS_WRITE_COUNTER;
		break;
	case USC_DDR_SET:
		dev->write_status = USC_STATUS_WRITE_DESIRE;
		break;
	case USC_GET_WRITE_STATUS:
		if (copy_to_user((int*)arg, &dev->write_status, sizeof(int)))
			return -EFAULT;
		break;
	default:
		return -ENOIOCTLCMD;
	}

	return retval;
}

static struct file_operations usc_fops = {
	.owner =	THIS_MODULE,
	.read =		usc_read,
	.write =	usc_write,
	//.ioctl = 	usc_ioctl,
	.unlocked_ioctl = 	usc_ioctl,				//chg t.miyo
	.open =		usc_open,
	.release =	usc_release,
};

/* 
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 */
static struct usb_class_driver usc_class = {
	.name =		"usc%d",
	.fops =		&usc_fops,
	.minor_base =	USB_USC_MINOR_BASE,
};

static int usc_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_usc *dev = NULL;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	size_t buffer_size;
	int i;
	int retval = -ENOMEM;

	/* allocate memory for our device state and initialize it */
	dev = kmalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL) {
		err("Out of memory");
		goto error;
	}
	memset(dev, 0, sizeof(*dev));
	kref_init(&dev->kref);
	sema_init(&dev->limit_sem, WRITES_IN_FLIGHT);
	//init_MUTEX(&dev->readbuf_sem);
	sema_init(&dev->readbuf_sem,1);			//chg t.miyo
	init_waitqueue_head(&dev->readbuf_wait);

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	dev->readreq_buffer = NULL;
	dev->readbuf_urb = NULL;
	dev->readbuf_work = NULL;
	dev->readbuf_buffered = NULL;
	dev->readbuf_last_read = 0;
	dev->readbuf_last_buffered = 0;

	iface_desc = interface->cur_altsetting;
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;

		dbg("endpoint %d(%d) %s",
		    endpoint->bEndpointAddress,
		    endpoint->bEndpointAddress & ~USB_ENDPOINT_DIR_MASK,
		    endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK? "in": "out");
		if (((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK)
		     == USB_DIR_IN) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
		     == USB_ENDPOINT_XFER_BULK)) {
			switch (endpoint->bEndpointAddress & ~USB_ENDPOINT_DIR_MASK) {
			case EP_READREQ:
				buffer_size = le16_to_cpu(endpoint->wMaxPacketSize);
				dev->readreq_size = buffer_size;
				dev->readreq_buffer = kmalloc(buffer_size, GFP_KERNEL);
				if (!dev->readreq_buffer) {
					err("Could not allocate readreq_buffer");
					goto error;
				}
				break;
			case EP_READ:
				buffer_size = sizeof(struct udata);
				dev->readbuf_urb = usb_alloc_urb(0, GFP_KERNEL);
				if (!dev->readbuf_urb) {
					err("Could not allocate readbuf_urb");
					goto error;
				}
				dev->readbuf_size = buffer_size;
				dev->readbuf_work = kmalloc(buffer_size, GFP_KERNEL);
				if (!dev->readbuf_work) {
					err("Could not allocate readbuf_work");
					goto error;
				}
				dev->readbuf_buffered = kmalloc(buffer_size, GFP_KERNEL);
				if (!dev->readbuf_buffered) {
					err("Could not allocate readbuf_buffer");
					goto error;
				}
				usb_fill_bulk_urb(dev->readbuf_urb, dev->udev,
						  usb_rcvbulkpipe(dev->udev, endpoint->bEndpointAddress),
						  dev->readbuf_work, dev->readbuf_size,
						  usc_read_bulk_callback, dev);
				break;
			default:
				break;
			}
		}

		if (((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK)
		     == USB_DIR_OUT) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
		     == USB_ENDPOINT_XFER_BULK)) {
			switch (endpoint->bEndpointAddress & ~USB_ENDPOINT_DIR_MASK) {
			case EP_DR:
				buffer_size = le16_to_cpu(endpoint->wMaxPacketSize);
				dev->write_counter_size = buffer_size;
				dev->write_counter_buffer = kmalloc(buffer_size, GFP_KERNEL);
				if (!dev->write_counter_buffer) {
					err("Could not allocate write_counter_buffer");
					goto error;
				}
				break;
			case EP_DDR:
			default:
				break;
			}
		}
	}

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	/* we can register the device now, as it is ready */
	retval = usb_register_dev(interface, &usc_class);
	if (retval) {
		/* something prevented us from registering this driver */
		err("Not able to get a minor for this device.");
		usb_set_intfdata(interface, NULL);
		goto error;
	}

	/* let the user know what node this device is now attached to */
	//info("USB Sensor device now attached to usc - %d", interface->minor);
	//chg t.miyo
	/* dev_info(&interface->dev,"USB Sensor device now attached to usc - %d", interface->minor); */
	
	return 0;

error:
	if (dev) {
		if (dev->readreq_buffer)
			kfree(dev->readreq_buffer);
		if (dev->readbuf_urb)
			usb_free_urb(dev->readbuf_urb);
		if (dev->readbuf_work)
			kfree(dev->readbuf_work);
		if (dev->readbuf_buffered)
			kfree(dev->readbuf_buffered);
		kref_put(&dev->kref, usc_delete);
	}
	return retval;
}

static void usc_disconnect(struct usb_interface *interface)
{
	struct usb_usc *dev;
	int minor = interface->minor;

	/* prevent usc_open() from racing usc_disconnect() */
	//lock_kernel();		//debug t.miyo

	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	/* give back our minor */
	usb_deregister_dev(interface, &usc_class);

	//unlock_kernel();		//debug t.miyo

	/* decrement our usage count */
	kref_put(&dev->kref, usc_delete);
	wake_up(&dev->readbuf_wait);

	//info("USB Sensor #%d now disconnected", minor);
	//chg t.miyo
	/* dev_info(&interface->dev,"USB Sensor #%d now disconnected", minor); */
}

static struct usb_driver usc_driver = {
	.name =		"usc",
	.probe =	usc_probe,
	.disconnect =	usc_disconnect,
	.id_table =	usc_table,
};

static int __init usb_usc_init(void)
{
	int result;

	/* register this driver with the USB subsystem */
	result = usb_register(&usc_driver);
	if (result)
		err("usb_register failed. Error number %d", result);

	return result;
}

static void __exit usb_usc_exit(void)
{
	/* deregister this driver with the USB subsystem */
	usb_deregister(&usc_driver);
}

module_init (usb_usc_init);
module_exit (usb_usc_exit);

MODULE_LICENSE("GPL");
/*
  Local Variables:
  c-file-style: "linux"
  End:
*/
