#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/vmalloc.h>

int init_module(void);
void cleanup_module(void);
static int device_open(struct inode *,
	struct file *);
static int device_release(struct inode *,
	struct file *);
static ssize_t device_read(struct file *,
	char *, size_t, loff_t *);
static ssize_t device_write(struct file *,
	const char *, size_t, loff_t *);

#define SUCCESS 0
#define DEVICE_NAME "linet"// Dev name 
#define BUF_LEN 80//Max length of device message 

//---------------------------------------------------------------------------------------------------------
//Things for the GPIO Port 

#define BCM2708_PERI_BASE       0x3F000000 //pi 3B+
#define GPIO_BASE               (BCM2708_PERI_BASE + 0x200000)	// GPIO controller  
#define BLOCK_SIZE 		(4*1024)

#define INP_GPIO(g)   *(gpio.addr + ((g)/10)) &= ~(7<<(((g)%10)*3)) 
#define OUT_GPIO(g)   *(gpio.addr + ((g)/10)) |=  (1<<(((g)%10)*3)) //001
//alternative function
#define SET_GPIO_ALT(g,a) *(gpio.addr + (((g)/10))) |= (((a)<=3?(a) + 4:(a)==4?3:2)<<(((g)%10)*3))
 
#define GPIO_SET  *(gpio.addr + 7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR  *(gpio.addr + 10) // clears bits which are 1 ignores bits which are 0
 
#define GPIO_READ(g)  *(gpio.addr + 13) &= (1<<(g))


//GPIO Clock
#define CLOCK_BASE              (BCM2708_PERI_BASE + 0x00101000)
#define GZ_CLK_BUSY (1 << 7)

//---------------------------------------------------------------------------------------------------------

//How many samples to capture
#define SAMPLE_SIZE 	1000000	//1M

//Define GPIO Pins

//ADC 0-11
#define BIT0_ADC 0
#define BIT1_ADC 1
#define BIT2_ADC 2
#define BIT3_ADC 3
#define BIT4_ADC 4
#define BIT5_ADC 5
#define BIT6_ADC 6
#define BIT7_ADC 7
#define BIT8_ADC 8
#define BIT9_ADC 9
#define BIT10_ADC 10
#define BIT11_ADC 11

//CS 12-15
#define BIT12_CS 12
#define BIT13_CS 13
#define BIT14_CS 14
#define BIT15_CS 15

//---------------------------------------------------------------------------------------------------------

// IO Acces
static struct bcm2837_peripheral {
    unsigned long addr_p;
    int mem_fd;
    void *map;
    volatile unsigned int *addr;
};
 

static int map_peripheral(struct bcm2837_peripheral *p);
static void unmap_peripheral(struct bcm2837_peripheral *p);
static void readScope(void);//Read a specific sample from the scope


static int Major;		/* Major number assigned to our device driver */
static int Device_Open = 0;	/* Is device open?  
				 * Used to prevent multiple access to device */
static char msg[BUF_LEN];	/* The msg the device will give when asked */
static char *msg_Ptr;


static uint16_t *ScopeBuffer_Ptr;
static unsigned char *buf_p;


static struct  fops = {
	.read = device_read,
	.write = device_write,
	.open = device_open,
	.release = device_release
};

//---------------------------------------------------------------------------------------------------------

static struct bcm2837_peripheral myclock = {CLOCK_BASE};

static struct bcm2837_peripheral gpio = {GPIO_BASE};

static int Buffer_index;

static uint16_t Buffer[SAMPLE_SIZE];	//need only 16bit front

static unsigned char *ScopeBufferStart;
static unsigned char *ScopeBufferStop;

//---------------------------------------------------------------------------------------------------------

static int map_peripheral(struct bcm2837_peripheral *p)
{
	p->addr=(uint32_t *)ioremap(GPIO_BASE, BLOCK_SIZE); //41 GPIO register with 32 bit (4*8)
   	return 0;
}
 
static void unmap_peripheral(struct bcm2837_peripheral *p) {
 	iounmap(p->addr);//unmap the address
}
//---------------------------------------------------------------------------------------------------------
static void readScope(void){

	int counter=0;
	int Fail=0;

	//disable IRQ
    local_irq_disable();
    local_fiq_disable();

    //take samples
	while(counter<SAMPLE_SIZE){
		Buffer[counter++]= *(gpio.addr + 13); 
	}

	//enable IRQ
    local_fiq_enable();
    local_irq_enable();

	buf_p=(unsigned char*)Buffer;//cound maybe removed

	ScopeBufferStart=(unsigned char*)Buffer;

	ScopeBufferStop=ScopeBufferStart+sizeof(Buffer);
}

//---------------------------------------------------------------------------------------------------------

/*
 * This function is called when the module is loaded
 */
int __init init_module(void)
{
    Major = register_chrdev(0, DEVICE_NAME, &fops);

	if (Major < 0) {
	  printk(KERN_ALERT "Registering char device failed with %d\n", Major);
	  return Major;
	}

	printk(KERN_EMERG "I was assigned major number %d. To talk to\n", Major);
	printk(KERN_EMERG "the driver, create a dev file with\n");
	printk(KERN_EMERG "'mknod /dev/%s c %d 0'.\n", DEVICE_NAME, Major);
	printk(KERN_EMERG "Remove the device file and module when done.\n");

	//Map GPIO

	if(map_peripheral(&gpio) == -1) 
	{
		printk(KERN_ALERT,"Failed to map the physical GPIO registers into the virtual memory space.\n");
		return -1;
	}

	//Define Scope pins
	// Define as  Input
	INP_GPIO(BIT0_ADC);
	INP_GPIO(BIT1_ADC);
	INP_GPIO(BIT2_ADC);
	INP_GPIO(BIT3_ADC);
	INP_GPIO(BIT4_ADC);
	INP_GPIO(BIT5_ADC);
	INP_GPIO(BIT6_ADC);
	INP_GPIO(BIT7_ADC);
	INP_GPIO(BIT8_ADC);
	INP_GPIO(BIT9_ADC);
	INP_GPIO(BIT10_ADC);
	INP_GPIO(BIT11_ADC);
	INP_GPIO(BIT12_CS);
	INP_GPIO(BIT13_CS);
	INP_GPIO(BIT14_CS);
	INP_GPIO(BIT15_CS);

	return SUCCESS;
}
//---------------------------------------------------------------------------------------------------------
/*
 * This function is called when the module is unloaded
 */
void __exit cleanup_module(void)
{
	unregister_chrdev(Major, DEVICE_NAME);
	unmap_peripheral(&gpio);
	unmap_peripheral(&myclock);
}
//---------------------------------------------------------------------------------------------------------
/* 
 * Called when a process tries to open the device file, like
 * "cat /dev/mycharfile"
 */
static int device_open(struct inode *inode, struct file *file)
{
	static int counter = 0;

	if (Device_Open)
		return -EBUSY;

	Device_Open++;
	sprintf(msg, "I already told you %d times Hello world!\n", counter++);
	msg_Ptr = msg;

	readScope();//Read n Samples into memory

	try_module_get(THIS_MODULE);

	return SUCCESS;
}
//---------------------------------------------------------------------------------------------------------
/* 
 * Called when a process closes the device file.
 */
static int device_release(struct inode *inode, struct file *file)
{
	Device_Open--;		/* We're now ready for our next caller */
	module_put(THIS_MODULE);
	return 0;
}
//---------------------------------------------------------------------------------------------------------
/* 
 * Called when a process, which already opened the dev file, attempts to
 * read from it.
 * Here one could also add a call of the function readScope() in order to do a permanent readout. 
 * As the code is right now one needs to open the device file for each new measurement, read from it and close it. 
 */
static ssize_t device_read(struct file *filp,	
			   char *buffer,	
			   size_t length,
			   loff_t * offset)
{
	
	// Number of bytes actually written to the buffer 
	int bytes_read = 0;

	if (*msg_Ptr == 0)
		return 0;

	//Check that we do not overfill the buffer

	while (length && buf_p<ScopeBufferStop) {

		if(0!=put_user(*(buf_p++), buffer++))
			printk(KERN_INFO "Problem with copy\n");
		length--;
		bytes_read++;
	}

	return bytes_read;
}
//---------------------------------------------------------------------------------------------------------
/*  
 * Called when a process writes to dev file: echo "hi" > /dev/hello 
 */
static ssize_t
device_write(struct file *filp, const char *buff, size_t len, loff_t * off)
{
	printk(KERN_ALERT "Sorry, this operation isn't supported.\n");
	return -EINVAL;
}
