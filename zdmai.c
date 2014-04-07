/*  mydrv.c  */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/dmaengine.h>
#include <linux/vmalloc.h>
#include <linux/pagemap.h>

#include <asm/uaccess.h>
#include <linux/amba/xilinx_dma.h>

#include "zdmai.h"

/* Standard module information*/
MODULE_LICENSE("GPL");
MODULE_AUTHOR
("Yohei Matsumoto - Tokyo Univ. Marine Science and Technology <yhmtmt@kaiyodai.ac.jp>");
MODULE_DESCRIPTION
("zdmai - test driver module for led and switch of zedboard connected by AXI_GPIO ");

#define DRIVER_NAME "zdmai"

////////////////////////////////////////////////////////// Parameters
int zdmai_major = 0; // major number (dynamically allocated in probe)
int zdmai_minor = 0; // minor number (zero fixed)
int zdmai_nr_devs = 1; // only one device node is supported.

int num_max_dsc = 11;
unsigned long max_byte_per_dsc = 0x7FFFFF;

module_param( max_byte_per_dsc, int , S_IRUGO );

/////////////////////////////////////////////////////////// Driver's local data
struct zdmai_local {
  // for char device
  struct cdev cdev;
  struct semaphore sem; // for interface mutex

  // for dma
  struct dma_chan * tx_chan;
  struct dma_chan * rx_chan;
  struct semaphore rx_sem;
  struct semaphore tx_sem;
};

static struct zdmai_local * lp;

////////////////////////////////////////////////////////// file operation override
ssize_t zdmai_read(struct file * filp, char __user * buf, size_t count,
		   loff_t * f_pos);

ssize_t zdmai_write(struct file * filp, const char __user * buf, size_t count,
		    loff_t * f_pos);

long zdmai_ioctl(struct file * filp, unsigned int cmd, unsigned long arg);

loff_t zdmai_llseek(struct file * filp, loff_t off, int whence);
int zdmai_open(struct inode * inode, struct file * filp);
int zdmai_release(struct inode * inode , struct file * filp);

////////////////////////////////////////////////////////// file operation object
struct file_operations zdmai_fops = {
  .owner = THIS_MODULE,
  .unlocked_ioctl = zdmai_ioctl,
  .read = zdmai_read,
  .write = zdmai_write,
  .llseek = zdmai_llseek,
  .open = zdmai_open,
  .release = zdmai_release,
};

/////////////////////////////////////////////////////////// fop implementation
static void zdmai_slave_tx_callback(void * completion){
  complete(completion);
}

static void zdmai_slave_rx_callback(void * completion){
  complete(completion);
}

int zdmai_dma_trans_pages(struct dma_device * dev, 
			  struct dma_chan * chan, 
			  const char __user * buf, size_t count,
			  enum dma_ctrl_flags flags,
			  enum dma_transfer_direction tdir)
{
  int num_pages, ipage, sglen, err, ret = count;
  struct dma_async_tx_descriptor * dsc;
  unsigned long len;
  dma_cookie_t cookie;
  struct xilinx_dma_config config;
  enum dma_status status;
  enum dma_data_direction ddir;
  struct scatterlist * sg;
  struct page ** pages;
  unsigned long tmo = msecs_to_jiffies(10000);
  struct completion cmp;
  void (*dma_async_tx_callback)(void * dma_async_param);

  //  unsigned long start = ((unsigned long) buf >> PAGE_SHIFT) << PAGE_SHIFT;
  unsigned long offset = (unsigned long) buf & ~PAGE_MASK;
  unsigned long fpg = ((unsigned long) buf) >> PAGE_SHIFT;
  unsigned long lpg = ((unsigned long) buf + (unsigned long) count - 1) >> PAGE_SHIFT;

  num_pages = lpg - fpg + 1;
  if(num_pages == 0)
    return 0;

  pages = kmalloc(num_pages * sizeof(struct page *), GFP_KERNEL);

  if(NULL == pages){
    return -ENOMEM;
  }

  // getting user page
  down_read(&current->mm->mmap_sem);
  err = get_user_pages(current,
		       current->mm,
		       (unsigned long) buf & PAGE_MASK,
		       num_pages,
		       tdir == DMA_DEV_TO_MEM, 1,
		       pages, NULL);
  /*
  pr_info("Direction %s ", tdir == DMA_DEV_TO_MEM ? "rx" : "tx");
  pr_info("bufp=%08x size=%d start=%08x ",
	  (unsigned int) buf, (unsigned int) count, (unsigned int) start);
  pr_info("offset=%08x fpg=%08x lpg=%08x pages=%d(%d)\n",  
	  (unsigned int) offset, (unsigned int) fpg,
	  (unsigned int) lpg, num_pages, err);
  */
  up_read(&current->mm->mmap_sem);
  
  if(err != num_pages){
    return err < 0 ? err : -EINVAL;
  }

  if(tdir == DMA_MEM_TO_DEV){
    dma_async_tx_callback = zdmai_slave_tx_callback;
    ddir = DMA_TO_DEVICE;
  }else{
    dma_async_tx_callback = zdmai_slave_rx_callback;
    ddir = DMA_FROM_DEVICE;
  }
 
  config.coalesc  = 1;
  config.delay = 0;

  dev->device_control(chan, DMA_SLAVE_CONFIG,
			 (unsigned long) & config);

  sg = vmalloc(num_pages * sizeof(struct scatterlist));
  if(sg == NULL)
    return 0;

  sg_init_table(sg, num_pages);

  // filling scatter gather page information
  len = PAGE_SIZE - offset;
  if(count < len)
    len = count;
  //  pr_info("Set initial page.\n");
  sg_set_page(&sg[0], pages[0],
	      min_t(size_t, PAGE_SIZE - offset, count), offset);
  count -= len;

  for(ipage = 1; ipage < num_pages - 1; ipage++){
    //pr_info("Set %d th page.\n", ipage);
    sg_set_page(&sg[ipage], pages[ipage], PAGE_SIZE, 0);
    count -= PAGE_SIZE;
  }
  
  if(num_pages > 1){
    //    pr_info("Set last page.\n");
    sg_set_page(&sg[ipage], pages[ipage], count, 0);
  }

  // mapping scatter gather buffer.
  //  pr_info("dma map sg .. ");
  sglen = dma_map_sg(dev->dev, sg, num_pages, ddir);
  //  pr_info(" done. sglen=%d\n", sglen);
  if(dma_mapping_error(dev->dev, sglen)){
    pr_err("Oops dma_mapping_error failed.\ n");
    ret = -ENOMEM;
    goto out;
  }

  //  pr_info("prep desc .. ");
  // prepareing hardware specific descriptor chain
  dsc = dev->device_prep_slave_sg(chan, sg, sglen,
				  tdir, flags, NULL);
  if(!dsc){
    pr_info("DMA failed to prepare descriptor chain.\n");
    ret = -ENOMEM;
    goto out;
  }
  //  pr_info(" done\n");

  // registering descriptor
  cookie = dsc->tx_submit(dsc);
  if(dma_submit_error(cookie)){
    pr_info("DMA failed to submit data.\n");
    goto out;
  }

  // setting completion callback
  //  pr_info("issue .. ");
  init_completion(&cmp);
  dsc->callback = dma_async_tx_callback;
  dsc->callback_param = &cmp;

  // launching dma
  dma_async_issue_pending(chan);
  //  pr_info(" done.");

  // waiting completion
  tmo = wait_for_completion_timeout(&cmp, tmo);
  status = dma_async_is_tx_complete(chan, cookie, NULL, NULL);

  if(tmo == 0){
    pr_info("DMA time out.\n");
    ret = -EIO;
    goto out;
  }else if(status != DMA_SUCCESS){
    pr_info("DMA failed.\n");
    ret = -EIO;
    goto out;
  }
  //  pr_info("dma done.\n");

 out:
  //  pr_info("unmapping.\n");
  dma_unmap_sg(dev->dev, sg, num_pages, ddir);
  //  pr_info("freeing sg. %08x\n", (unsigned int) sg);
  vfree(sg);
  
  return ret;
}

ssize_t zdmai_read(struct file * filp, char __user * buf, size_t count,
		   loff_t * f_pos)
{
  ssize_t retval = 0;
  struct zdmai_local * lp = filp->private_data;
  struct dma_device * rx_dev = lp->rx_chan->device;
  enum dma_ctrl_flags flags;
  flags = DMA_CTRL_ACK | DMA_COMPL_SKIP_DEST_UNMAP | DMA_PREP_INTERRUPT;

  if(down_interruptible(&lp->rx_sem))
    return -ERESTARTSYS;

  retval = zdmai_dma_trans_pages(rx_dev, lp->rx_chan, buf, count, 
				 flags, DMA_DEV_TO_MEM);

  up(&lp->rx_sem);

  return retval;
}

ssize_t zdmai_write(struct file * filp, const char __user * buf, size_t count,
		    loff_t * f_pos)
{
  ssize_t retval;
  struct zdmai_local * lp = filp->private_data;
  struct dma_device * tx_dev = lp->tx_chan->device;
  enum dma_ctrl_flags flags;

  flags = DMA_CTRL_ACK | DMA_COMPL_SKIP_SRC_UNMAP | DMA_PREP_INTERRUPT;
  
  if(down_interruptible(&lp->tx_sem))
    return -ERESTARTSYS;
  
  retval = zdmai_dma_trans_pages(tx_dev, lp->tx_chan, buf, count, 
				 flags, DMA_MEM_TO_DEV);
 
  up(&lp->tx_sem);

  return retval;
}
    
long zdmai_ioctl(struct file * filp, unsigned int cmd, unsigned long arg)
{
  int err = 0;
  int retval = 0;
  struct zdmai_local * lp = filp->private_data;

  if (_IOC_TYPE(cmd) != ZDMAI_IOC_MAGIC) return -ENOTTY;
  if (_IOC_NR(cmd) > ZDMAI_IOC_MAXNR) return -ENOTTY;
  
  if(_IOC_DIR(cmd) & _IOC_READ)
    err = !access_ok(VERIFY_WRITE, (void __user*) arg, _IOC_SIZE(cmd));
  else if(_IOC_DIR(cmd) & _IOC_WRITE)
    err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
  if(err) return -EFAULT;
  
  if(down_interruptible(&lp->sem))
    return -ERESTARTSYS;

  switch(cmd){
  default:
    break;
  }

  up(&lp->sem);
  return retval;
}

loff_t zdmai_llseek(struct file * filp, loff_t off, int whence)
{
  loff_t newpos = off;
  return newpos;
}

int zdmai_open(struct inode * inode, struct file * filp)
{
  struct zdmai_local * lp;
 
  lp = container_of(inode->i_cdev, struct zdmai_local, cdev);
  filp->private_data = lp;
  return 0;
}


int zdmai_release(struct inode * inode , struct file * filp)
{
  return 0;
}

////////////////////////////////////////////////// fop end.

////////////////////////////////////////////////// platform driver functions

static bool zdmai_filter(struct dma_chan * chan, void * param){
  if(*((int*)chan->private) == *(int *)param)
    return true;
  return false;
}

static void zdmai_dma_free(struct zdmai_local * lp);

static int __init zdmai_dma_init(struct zdmai_local * lp){
  dma_cap_mask_t mask;
  enum dma_data_direction direction;
  u32 match;

  lp->tx_chan = lp->rx_chan = NULL;

  dma_cap_zero(mask);
  dma_cap_set(DMA_SLAVE | DMA_PRIVATE, mask);

  direction = DMA_MEM_TO_DEV;
  match = (direction & 0xFF) | XILINX_DMA_IP_DMA;
  lp->tx_chan = dma_request_channel(mask, zdmai_filter, (void*)&match);

  if(!lp->tx_chan){
    pr_info("zdmai: Did not find tx device. %x\n", (unsigned int)lp->tx_chan);
    goto out;
  }
  
  direction = DMA_DEV_TO_MEM;
  match = (direction & 0xFF) | XILINX_DMA_IP_DMA;
  lp->rx_chan = dma_request_channel(mask, zdmai_filter, (void*)&match);
  
  if(!lp->rx_chan){
    pr_info("zdmai: Did not find rx device. %x \n", (unsigned int) lp->rx_chan);
    goto out;
  }

  // initializing semaphore
  sema_init(&lp->sem, 1);
  sema_init(&lp->rx_sem, 1);
  sema_init(&lp->tx_sem, 1);
    
  return 0;

 out:
  zdmai_dma_free(lp);
  return -1;
}

static void zdmai_dma_free(struct zdmai_local * lp){
  if(lp->rx_chan)
    dma_release_channel(lp->rx_chan);
  if(lp->tx_chan)
    dma_release_channel(lp->tx_chan);
}

static int zdmai_cdev_init(struct zdmai_local * lp){
  dev_t devno;
  int rc = 0;
  
  if(zdmai_major){
    devno = MKDEV(zdmai_major, zdmai_minor);
    rc = register_chrdev_region(devno, zdmai_nr_devs, DRIVER_NAME);
  }else{
    rc = alloc_chrdev_region(&devno, zdmai_minor, zdmai_nr_devs, DRIVER_NAME);
    zdmai_major = MAJOR(devno);
  }
  
  printk(KERN_INFO "zdmai allocate cdev %d %d", zdmai_major, zdmai_minor);
  
  if(rc < 0){
    printk(KERN_WARNING "%s: can't get major %d\n", DRIVER_NAME, zdmai_major);
    return rc;
  }
  
  cdev_init(&lp->cdev, &zdmai_fops);
  lp->cdev.owner = THIS_MODULE;
  rc = cdev_add(&lp->cdev, devno, 1);
  if(rc){
    printk(KERN_NOTICE "Error %d adding %s%d", rc, DRIVER_NAME, 0);
    goto error;
  }

  return 0;

 error:
  unregister_chrdev_region(MKDEV(zdmai_major, zdmai_minor), zdmai_nr_devs);
  return -1;
}

static void zdmai_cdev_free(struct zdmai_local * lp){
  dev_t devno = MKDEV(zdmai_major, zdmai_minor);
  cdev_del(&lp->cdev);
  unregister_chrdev_region(devno, zdmai_nr_devs);
 }


static int __init zdmai_init(void)
{
  int rc = 0;

  ////////////////////////// allocating local data structure /////////////////////////////
  lp = (struct zdmai_local *) kmalloc(sizeof(struct zdmai_local), GFP_KERNEL);
  if (!lp) {
    printk(KERN_INFO "Cound not allocate zdmai device\n");
    return -ENOMEM;
  }
  
  if(zdmai_cdev_init(lp) < 0)
    goto out_free_local;

  /////////////////////// initializing dma device ////////////////////////////////////
  if(zdmai_dma_init(lp) < 0)
    goto out_free_cdev;

  printk(KERN_INFO "start zdmai.");  

  return rc;

 out_free_cdev:
  cdev_del(&lp->cdev);
 out_free_local:
  kfree(lp);
  return rc;
}


static void __exit zdmai_exit(void)
{
  zdmai_cdev_free(lp);
  zdmai_dma_free(lp);
  kfree(lp);

  printk(KERN_INFO "end zdmai.\n");
}

module_init(zdmai_init);
module_exit(zdmai_exit);

