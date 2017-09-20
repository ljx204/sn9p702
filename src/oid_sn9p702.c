#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/random.h>
#include <linux/init.h>
#include <linux/raw.h>
#include <linux/tty.h>
#include <linux/capability.h>
#include <linux/ptrace.h>
#include <linux/device.h>
#include <linux/highmem.h>
#include <linux/crash_dump.h>
#include <linux/backing-dev.h>
#include <linux/bootmem.h>
#include <linux/splice.h>
#include <linux/pfn.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/aio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>   //wake_up_process()
#include <linux/kthread.h> //kthread_create(),kthread_run()
#include <linux/err.h> //IS_ERR(),PTR_ERR()

#include <linux/delay.h>
#include <linux/wait.h>
#include <asm/uaccess.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>  //----- gpio_to_irq()
#include <linux/interrupt.h> //---request_irq()
#include "oid_sn9p702.h"


#define OID_DEBUG  1

#ifdef OID_DEBUG
#define OID_PRINTF    printk
#else
#define OID_PRINTF(...)
#endif




#define OID_SDA   14            //GPIO14
#define OID_RESET   15            //GPIO15
#define OID_SCK 16



#define OID_SCK_L	   	        *GPIO_DATA_0 &= ~(1<<OID_SCK)              
#define OID_SCK_H		          *GPIO_DATA_0 |=  (1<<OID_SCK)              

#define OID_SDA_OUT_L          *GPIO_DATA_0 &= ~(1<<OID_SDA)
#define OID_SDA_OUT_H          *GPIO_DATA_0 |=  (1<<OID_SDA)


#define OID_SCK_OUT            *GPIO_CTRL_0 |=  (1<<OID_SCK) 

#define OID_SDA_OUT	           *GPIO_CTRL_0 |=  (1<<OID_SDA)            
#define OID_SDA_IN		         *GPIO_CTRL_0 &= ~(1<<OID_SDA)             
#define OID_SDA_DATA		       (((*GPIO_DATA_0)>>OID_SDA) & 0x01)       

#define OID_RESET_OUT          *GPIO_CTRL_0 |=  (1<<OID_RESET) 
#define OID_RESET_H            *GPIO_DATA_0 |=  (1<<OID_RESET)  
#define OID_RESET_L            *GPIO_DATA_0 &= ~(1<<OID_RESET) 



volatile unsigned long *GPIO_CTRL_0;    // GPIO0 to GPIO31 direction control register  0-input 1-output
volatile unsigned long *GINT_REDGE_0;   // GPIO0 to GPIO31 rising edge interrupt enable register
volatile unsigned long *GINT_FEDGE_0;   // GPIO0 to GPIO31 falling edge interrupt enable register
volatile unsigned long *GINT_STAT_0;    // GPIO0 to GPIO31 interrupt status register 1-int  0 -no int
volatile unsigned long *GINT_EDGE_0;    // GPIO0 to GPIO31 interrupt edge status register 1-rising 0-falling
volatile unsigned long *GPIO1_MODE;     // GPIO1 purpose selection register,for SPIS or GPIO14-17 mode selection
volatile unsigned long *AGPIO_CFG;      // analog GPIO configuartion,GPIO14-17 purpose
volatile unsigned long *GPIO_DATA_0;    // GPIO0-GPIO32 data register

static unsigned int oid_sda_int_number = 0;
static OID_VAR      stOIDVar;
dev_t dev_num;

struct cdev sn9p702_oid_cdev;

//auto "mknode /dev/sn9p704_oid c dev_num minor_num"

struct class *sn9p702_oid_class = NULL;
struct device *sn9p702_oid_device = NULL;
static struct task_struct *oid_task = NULL;

static DECLARE_WAIT_QUEUE_HEAD(oid_event_wq);
static volatile int oid_event=0;

static DECLARE_WAIT_QUEUE_HEAD(oid_read_wq);
static volatile int oid_read_data=0;


static DEFINE_SPINLOCK(oid_lock); 
static volatile int oid_read_flag=0;


void OID3TimerIsr(void);


static void  oid_init_gpio(void)
{
      //----------   map gpio register   --------
      
      GPIO1_MODE    =(volatile unsigned long *)ioremap(0x10000060,4); // GPIO1 purpose selection register,for SPIS or GPIO14-17 mode selection
      
      //AGPIO_CFG; // use default value, analog GPIO configuartion,GPIO14-17 purpose 
      
      GPIO_CTRL_0  =(volatile unsigned long *)ioremap(0x10000600,4);   //--- GPIO0 to GPIO31 direction control register  0-input 1-output
      GINT_REDGE_0 =(volatile unsigned long *)ioremap(0x10000650,4);  //--GPIO0 to GPIO31 rising edge interrupt enable register
      GINT_FEDGE_0 =(volatile unsigned long *)ioremap(0x10000660,4);  //--GPIO0-31, falling edge interrupt enable register
      GINT_STAT_0  =(volatile unsigned long *)ioremap(0x10000690,4);  //---GPIO0 to GPIO31 interrupt status register 1-int  0 -no int
      GPIO_DATA_0  =(volatile unsigned long *)ioremap(0x10000620,4); 
      
      //GINT_EDGE_0;  //---GPIO0 to GPIO31 interrupt edge status register 1-rising 0-falling

      //---------   set GPIO purpose  -----------
      
      *GPIO1_MODE |=(0x1<<2);               //---set SPIS as GPIO14-17	
      *GPIO1_MODE &=~(0x1<<3);	
      
      //SCK output

      OID_SCK_OUT;
      OID_SDA_IN;
      
      *GINT_REDGE_0 &=~(0x1<<OID_SDA);  
      *GINT_FEDGE_0 |=(0x1<<OID_SDA);  
      
      
      
 
}


static void oid_gpio_unmap(void)
{
      iounmap(GPIO1_MODE);
      iounmap(GPIO_CTRL_0);
      iounmap(GINT_REDGE_0);
      iounmap(GINT_STAT_0);
      iounmap(GPIO_DATA_0);
      iounmap(GINT_FEDGE_0);
}


/********************   Get GPIO Interrupt_Map Number    ******************/
static void oid_get_sda_int_num(void)
{
      oid_sda_int_number = gpio_to_irq(OID_SDA);  
      
      if(oid_sda_int_number != 0)
          OID_PRINTF("GPIO%d get IRQ= %d successfully!\n", OID_SDA, oid_sda_int_number); 
      else
          OID_PRINTF("Get oid_sda_int_number failed!\n");
}

static irqreturn_t oid_int_handler(int irq,void *dev_id,struct pt_regs *regs)
{
//     OID_PRINTF("-------- oid Interrupt triggered! ------\n");   
     
     if(((*GINT_STAT_0)>>OID_SDA)&0x1)  //-----confirm the Interrupt
     {
  //       OID_PRINTF("GPIO Interrupt confirmed!\n"); 
        
         disable_irq_nosync(oid_sda_int_number);
         
       
         *GINT_FEDGE_0 &=~(0x1<<OID_SDA);
         *GINT_STAT_0 &=~(0x1<<OID_SDA); 
         
         oid_event = 1;
        
         wake_up_interruptible(&oid_event_wq);
        
      }
      
      return IRQ_HANDLED;
}

static int register_oid_irq(void)
{
     int int_result;
     
     int_result=request_irq(oid_sda_int_number, oid_int_handler, IRQF_DISABLED,"SN9P702-INT",NULL);
     
     if(int_result!=0)
     {
           OID_PRINTF("OID Interrupt request_irq fail!\n"); 
           
           return -1;
     }
      else
           return 0;
           
      OID_PRINTF("OID Interrupt request_irq success!\n"); 
}


static OID3_READ_ST oid_read(int bitLength)
{
    INT32U i;
  	
  	OID3_READ_ST read_data={0};
  	
  	OID_SCK_H;
  	udelay(4);			
 
  	OID_SDA_OUT;
  	OID_SDA_OUT_L;
  
  	OID_SCK_L;
  				
  	udelay(2);	
  	OID_SCK_H;
  	OID_SDA_IN;
  	
  	for(i=0;i<bitLength;i++)
  	{
  		read_data.ddw64 <<= 1;
  		udelay(4);
  	
  		OID_SCK_L;
  		udelay(2);
  	
  		if(OID_SDA_DATA)
  			read_data.ddw64 |= 1;
  		
  		if(i<bitLength-1)
  		{
  		  OID_SCK_H;
  		}
  	
  	}
  
  
  	return read_data;
  
}



int oid_IsRequest(void)
{
      int ret;
      ret = OID_SDA_DATA;
      
      return !ret;
}

void OID3TimerIsr(void)
{	
	OID3_READ_ST recv_data;
	unsigned long flags;
	
	if(oid_IsRequest())
	{
		recv_data=oid_read(64);
		if (recv_data.type.type & OID3_TYPE_COMMAND)
		{
			
			if ((recv_data.cmd.err&(0x3<<13)) == (0x1<<13))
			{
				// command active
				switch (recv_data.cmd.command)
				{
				case 0xfff8: // OID Processor warns DSP that it has already turned on
					stOIDVar.module_status = 1;
					printk("<-OID power on cmd\r\n");	
					break;
				case 0xfff7: // OID Processor warns DSP that it?¡¥s going to turned off
					stOIDVar.module_status = 0;
					printk("<-OID power off cmd\r\n");
					break;
				case 0xfff1: 
				
					stOIDVar.module_status = 1;
					printk("<-OID reset cmd\r\n"); 
					break;
				case 0xfff6: 
				
					stOIDVar.module_status = 0;
				
					printk("<-OID calibration cmd\r\n"); 
					break;
				default:
					break;
				}
			}
		}
		else
	  {
    		if (recv_data.type.type & OID3_TYPE_CODEMODE3)
    		{
    			if ((recv_data.type.type & OID3_TYPE_READ_ERR) == 0)
    			{
    				if (recv_data.type.type & OID3_TYPE_OID3_POS)
    				{
    					printk("x=%d,y=%d\r\n", recv_data.oid3_pos.x, recv_data.oid3_pos.y);
    				}
    				else // oid3 code index
    				{
    				  INT32U sdata = recv_data.oid3_code.index;
						  spin_lock_irqsave(&oid_lock, flags);
              oid_read_flag = 1;
              oid_read_data = recv_data.oid3_code.index;
              spin_unlock_irqrestore(&oid_lock, flags); 
              wake_up_interruptible(&oid_read_wq);
               
						  
    			//		printk("oid3 recv=0x%X, 0x%X\r\n", recv_data.type.type, recv_data.cmd.command);
    				}
    			}
    		}
    		else // oid2 code index
    		{
    			if ((recv_data.type.type & OID3_TYPE_READ_ERR) == 0)
    			{
    				spin_lock_irqsave(&oid_lock, flags);
            oid_read_flag = 1;
            oid_read_data = recv_data.oid2.index;
            spin_unlock_irqrestore(&oid_lock, flags);
            wake_up_interruptible(&oid_read_wq);
             
    			//	printk("oid2 recv=0x%X, 0x%X\r\n", recv_data.type.type, recv_data.cmd.command);
    			}
    		}
  	}
		//OIDCTRL_LOG("0x%08X%08X\r\n", recv_data.oid3_code.reserved[0], *((INT32U *)&recv_data));
	}
}


static int OID3_Write(INT8U write_data)
{
	INT16U i;
	INT16U ack_w = write_data+1;


	OID_SDA_OUT;
	OID_SDA_OUT_H;
	
	OID_SCK_H;
  udelay(110);
	OID_SCK_L;

	OID_SDA_IN;

	if(OID_SDA_DATA == 0) {
		printk("Error Write sda = 0 \r\n");
		return FALSE;

	}

	  udelay(2);
					
	  OID_SDA_OUT;
	
	  for(i=0;i<8;i++)
		{
			OID_SCK_H;						
			if(write_data&0x80)
				OID_SDA_OUT_H;				
			else	
				OID_SDA_OUT_L;			
			udelay(4);
			OID_SCK_L;
			udelay(2);
			write_data<<=1;
		}

	OID_SDA_OUT_H;
	OID_SDA_IN;

	
#if 0
	if(ack_w == 0x54) return TRUE;

  for ( i = 0; i < 1000; i++ )
	{
		udelay(10);
	        			
		if(oid_IsRequest())
		{
			OID3_READ_ST ack;
			ack = oid_read(16);
			
			if (ack_w == ack.write_ack.command)
			{
				return TRUE;
			}
		}
	}
	return FALSE;
#endif

}


 
#if 0
int OID3_Write_Loop(INT8U wData)
{
	INT32S i;
	while(OID3_Write(wData) == FALSE)
	{
		OID3TimerIsr();
	}
	// Interval between two commends must > 250 ms
	for (i=0; i<250/10; i++)
	{
		if(oid_IsRequest())
		{
			OID3TimerIsr();
			continue;
		}
		mdelay(10);
	}
	return TRUE;
}
#endif


static void oid_power_on(void)
{
      int i;
      OID3_READ_ST recv_data;
      OID_SCK_H;
      mdelay(60);
      OID_SCK_L;
      
      for(i = 0; i < 1000; i++) 
      {
          if(oid_IsRequest()){
            
            OID3TimerIsr();
           
            break; 
          }
      }
  
}


static int oid_read_thread(void *unused)
{
    int event = 0;
    int completed = 0;
    unsigned long flags;
    
    printk("oid read thread begin  .... \r\n");
    while(1) 
    {
        wait_event_interruptible(oid_event_wq, (oid_event != 0));
    //    printk("Wakeup .... \r\n");
        
        local_irq_save(flags);
        
         OID3TimerIsr();
         
         oid_event = 0;
        
         OID_SDA_IN;
        *GINT_FEDGE_0 |=(0x1<<OID_SDA);    //--bit set 1,enable Falling Edge interrupt  
         enable_irq(oid_sda_int_number);
        
        
        local_irq_restore(flags);
        
        if(kthread_should_stop()) break;
      
    }
       
    return 0;
}


void  oid_exit_fun(void)
{
        oid_gpio_unmap();  //-----free GPIO map
       
        if(oid_sda_int_number != 0)
        {
          free_irq(oid_sda_int_number,NULL); //----free irq NON-SHARED
          oid_sda_int_number = 0;
        }
        
       
  
}

/**************** œá¹¹Ìå file_operations ³ÉÔ±º¯Êý *****************/

//open
static int sn9p702_oid_open(struct inode *inode, struct file *file)
{
        int ret=0;
        int i, j;
        int err;
        int timeout; 
        //wait_queue_head_t timeout_wq; 
       // init_waitqueue_head(&timeout_wq); 
    
	     
        oid_init_gpio();
        oid_get_sda_int_num();
        ret = register_oid_irq();    //---register GPIO interrupt
        if(ret < 0) return ret;
          
        if(stOIDVar.module_status == 1) return 0; 
  
        
        //reset OID and get the number
        for(i = 0; i < 5; i++)
        {
           printk("reset again %d \r\n", i);
            
           OID_RESET_L;
           msleep(60);
          // interruptible_sleep_on_timeout(&timeout_wq, (HZ/100)*4);  
           OID_RESET_H;
           msleep(750);
          // interruptible_sleep_on_timeout(&timeout_wq, (HZ/100)*80);  
           
	         OID_SCK_H;
      	   msleep(60);
      	   //interruptible_sleep_on_timeout(&timeout_wq, (HZ/100)*6); 
           OID_SCK_L;
           
         
           for(j = 0; j < 100; j++) 
           {
              msleep(1);
             // interruptible_sleep_on_timeout(&timeout_wq, (HZ/100)); 
              if(stOIDVar.module_status == 1) break; 
           }
           
           if(stOIDVar.module_status == 1) break;
           
           printk("power off \r\n");
           
           *GINT_FEDGE_0 &=~(0x1<<OID_SDA);  
           OID3_Write(0x56); 
           *GINT_FEDGE_0 |=(0x1<<OID_SDA);  
         
        }
        
        if(i >= 5) 
        {
          printk("OID open failed \r\n");
          
          oid_exit_fun();
          
          return -1;    //OID init fail
          
        }
          
 
        printk("OID open OK !!!! \r\n");
	      return 0;
	      
}

static int sn9p702_oid_close(struct inode *inode , struct file *file)
{
	      
        oid_exit_fun();

	      return 0;
}

static ssize_t sn9p702_oid_read(struct file *file, char __user *buffer, size_t len, loff_t *pos)
{
      int ret = 0;
      unsigned long flags;
      
      if(oid_read_flag == 0 ) 
      {
          wait_event_interruptible(oid_read_wq, (oid_read_flag != 0));
      }
      
     spin_lock_irqsave(&oid_lock, flags);
     oid_read_flag = 0;
     copy_to_user(buffer,&oid_read_data,4);
     spin_unlock_irqrestore(&oid_lock, flags); 
     
     ret = 4;
          
      
       
      
        
      return ret;
}

//write
static ssize_t sn9p702_oid_write( struct file *file , const char __user *buffer,
			   size_t len , loff_t *offset )
{
	  int ret = 0;
	  return ret;
}

//unlocked_ioctl
static int sn9p702_oid_ioctl (struct file *filp , unsigned int cmd , unsigned long arg)
{
	int ret = 0;
	
	return ret;
	
}


/***************** œá¹¹Ìå£º file_operations ************************/
//struct
static const struct file_operations sn9p702_oid_fops = {
	.owner   = THIS_MODULE,
	.open	 = sn9p702_oid_open,
	.release = sn9p702_oid_close,	
	.read	 = sn9p702_oid_read,
	.write   = sn9p702_oid_write,
	.unlocked_ioctl	= sn9p702_oid_ioctl,
};



static unsigned char init_flag = 0;



static __init int sn9p702_oid_init(void)
{
  	int ret = 0;
    int intc_result;

  	if(( ret = alloc_chrdev_region(&dev_num, 0, 1, "sn9p702_oid_proc") ) < 0 )
  	{
  		goto dev_reg_error;
  	}
  	init_flag = 1; //±êÊŸÉè±žŽŽœš³É¹Š£»
  
  	printk("SN9P702 Driver :\nmajor: %d\nminor: %d\n", MAJOR(dev_num),MINOR(dev_num));
  
  	cdev_init(&sn9p702_oid_cdev, &sn9p702_oid_fops);
  	
  	
  	if( (ret = cdev_add(&sn9p702_oid_cdev,dev_num,1)) != 0 )
  	{
  		goto cdev_add_error;
  	}
  
  	
  	sn9p702_oid_class = class_create(THIS_MODULE,"sn9p702_oid_class");
  	if( IS_ERR(sn9p702_oid_class) )
  	{
  		goto class_c_error;
  	}
  
  	sn9p702_oid_device = device_create(sn9p702_oid_class, NULL, dev_num, NULL, "sn9p702_oid");
  	if( IS_ERR(sn9p702_oid_device) )
  	{
  		goto device_c_error;
  	}
  	
  	printk("sn9p702 mknod success!\n");
  	
  	memset((INT8S *)&stOIDVar, 0, sizeof(stOIDVar));
	  stOIDVar.module_status = -1;
	
  	
  	
  	oid_task = kthread_run(oid_read_thread, NULL,"oid_thread");



    goto init_success;

dev_reg_error:
	  printk("alloc_chrdev_region failed\n");	
	  return ret;

cdev_add_error:
	  printk("cdev_add failed\n");
 	  unregister_chrdev_region(dev_num, 1);
	  init_flag = 0;
	  return ret;

class_c_error:
	  printk("class_create failed\n");
	  cdev_del(&sn9p702_oid_cdev);
 	  unregister_chrdev_region(dev_num, 1);
	  init_flag = 0;
	  return PTR_ERR(sn9p702_oid_class);

device_c_error:
	  printk("device_create failed\n");
	  cdev_del(&sn9p702_oid_cdev);
 	  unregister_chrdev_region(dev_num, 1);
	  class_destroy(sn9p702_oid_class);
	  init_flag = 0;
	  return PTR_ERR(sn9p702_oid_device);

//------------------ ÇëÔÚŽËÌíŒÓÄúµÄŽíÎóŽŠÀíÄÚÈÝ ----------------//
gpio_int_error:
         		
	  return -1;
  
init_success:
  	printk("SN9P702 init success!\n");
  	return 0;
}

//exit
static __exit void sn9p702_oid_exit(void)
{
  	if(init_flag == 1)
  	{
  		cdev_del(&sn9p702_oid_cdev);
   		unregister_chrdev_region(dev_num, 1);
  		device_unregister(sn9p702_oid_device);
  		class_destroy(sn9p702_oid_class);
  	}
  	
  	 if(oid_task) 
     {
          printk("oid task stop\n");
          kthread_stop(oid_task); 
     }
        
  	
}


/**************** module operations**********************/
//module loading
module_init(sn9p702_oid_init);
module_exit(sn9p702_oid_exit);


MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("from Ken");
MODULE_DESCRIPTION("SN9P702 drive");














