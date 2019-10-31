#include <linux/module.h>       /* Needed by all modules */
#include <linux/kernel.h>       /* Needed for KERN_ALERT */
#include <linux/init.h>
#include <asm/uaccess.h>

#include <linux/fs.h>
#include <linux/types.h>
#include <linux/cdev.h>

#define KERL_ERR 3

//定义设备名称
#define DEVICE_NAME "test"
#define BUF_SIZE 	1024
static char tmpbuf[BUF_SIZE];

//定义主次设备号
static unsigned int TestMajor=0;
static unsigned int TestMinor=0;

/* 注册字符设备 */
static struct cdev *test_cdev;
static dev_t dev;

static int test_chardev_open(struct inode *inode,struct file *file)
{


    printk("open major=%d, minor=%d\n", imajor(inode), iminor(inode));
	return 0;
}
static int test_chardev_release(struct inode *inode,struct file *file)
{

    printk("close major=%d, minor=%d\n", imajor(inode), iminor(inode));
	return 0;
}



static ssize_t test_chardev_read(struct file *file,char __user *buf,
				size_t const count,loff_t *offset)
{
	if(count < BUF_SIZE)
	{
		if(copy_to_user(buf,tmpbuf,count))
		{
		  printk("copy to user fail \n");
	 	  return -EFAULT;
		}
	}else{
		printk("read size must be less than %d\n", BUF_SIZE);
		return -EINVAL;
}
	*offset += count;
		return count;
}

static ssize_t test_chardev_write(struct file *file, const char __user *buf,size_t const count,loff_t *offset)
{
	if(count < BUF_SIZE)
	{
		if(copy_from_user(tmpbuf,buf,count))
		{
		  printk("copy from user fail \n");
	 	  return -EFAULT;
		}
	}else{
	
		printk("size must be less than %d\n", BUF_SIZE);
		return -EINVAL;
}
		
	*offset += count;
	return count;
}


static int test_chardev_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{

	printk("test-ioctl: param %u %lu\n", cmd, arg);
      switch(cmd)
    {
	case 1:
        {
	printk("The 1 is Press\n");
         if(arg==0)
               printk("The 1 say hello!\n");
         else
               printk("The 1 say goobye!\n"); 
        break;
	}
	case 2:
        {
	printk("The 2 is Press\n");
         if(arg==0)
               printk("The 2 say hello!\n");
         else
               printk("The 2 say goobye!\n"); 
        break;
	}
	case 3:
        {
	printk("The 3 is Press\n");
         if(arg==0)
               printk("The 3 say hello!\n");
         else
               printk("The 3 say goobye!\n"); 
        break;
	}
	case 4:
        {
	printk("The 4 is Press\n");
         if(arg==0)
               printk("The 4 say hello!\n");
         else
               printk("The 4 say goobye!\n"); 
        break;
	}
        default:
	printk(" Unkown key");
        break;
      	}

	return 0;
}


static struct file_operations chardev_fops={
	.owner = THIS_MODULE,
	.open = test_chardev_open,
	.release = test_chardev_release,
	.read = test_chardev_read,
	.write = test_chardev_write,
	.ioctl=test_chardev_ioctl,
};

static int    test_init(void)
{	

	int result;

	 printk("Hello world 1.\n");
	
/*分配设备编号*/
	if(TestMajor)
	{
		dev=MKDEV(TestMajor,TestMinor);//创建设备编号
		result=register_chrdev_region(dev,1,DEVICE_NAME);
	} else {
		result=alloc_chrdev_region(&dev,TestMinor,1,DEVICE_NAME);
		TestMajor=MAJOR(dev);
	}
	if(result<0)
	{
		printk(KERN_WARNING"LED: cannot get major %d \n",TestMajor);
		return result;
	}
	
/* 注册字符设备 */
	test_cdev=cdev_alloc();
	cdev_init(test_cdev,&chardev_fops);
	//test_cdev->ops=&chardev_fops;
	test_cdev->owner=THIS_MODULE;
	result=cdev_add(test_cdev,dev,1);
	if(result)
		printk("<1>Error %d while register led device!\n",result);
	
		

   /* A non 0 return means init_module failed; module can't be loaded.*/
        return 0;

}

static void  test_exit(void)
{
        printk("<0>""Goodbye world 1.\n");
        unregister_chrdev_region(MKDEV(TestMajor,TestMinor),1);
	cdev_del(test_cdev);	
}

module_init(test_init);
module_exit(test_exit);
MODULE_LICENSE("GPL");

