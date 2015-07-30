/*
 * AMLOGIC av_det driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the named License,
 * or any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 *
 *
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/of_gpio.h>
#include <linux/amlogic/cpu_version.h>
#include <linux/amlogic/iomap.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/timer.h>
//#include <linux/sysfs.h>


#define OWNER_NAME   "aml-avdet"
#define TIMER_DET

struct aml_av_det_platform_data {
	const char *name;
	int interrupt_pin;
	struct gpio_desc *interrupt_desc;
	int audio_sel_pin;
	struct gpio_desc *audio_sel_desc;
	struct switch_dev sdev; // for android
#ifdef TIMER_DET
	struct work_struct work;
	struct mutex lock;
#else
	unsigned int irq_in_num;
	unsigned int irq_out_num;
#endif
};

struct aml_av_det {
	struct aml_av_det_platform_data	*pdata;
	struct platform_device			*pdev;	
};

typedef enum audio_select_e {
	I2S_AUDIO_OUTPUT=0,
	COAXIAL_OUTPUT,
	AV_AUDIO_MUTE,
	AV_AUDIO_UNMUTE,
	HDMI_AUDIO_MUTE,
	HDMI_AUDIO_UNMUTE,
}audio_select_t;

typedef enum {
	AV_IN=0,
	AV_OUT,
}av_det_state;

static struct aml_av_det *Myamlav_det = NULL;
unsigned int audio_mode_s = 0;
#ifdef TIMER_DET
static struct timer_list g_timer;
unsigned int  det_delay_10ms = 20;/* av det check delay,unit 10ms*/
static int det_flag = 2; /*0:av in,1,av:out,2:first boot check*/ 
#endif
extern void aml_audio_i2s_mute(void);/* both i2s and spdif mute*/
extern void aml_audio_i2s_unmute(void);/* both i2s and spdif unmute*/
static int audio_control(unsigned int audio_mode )
{
	struct aml_av_det *amlav_det = Myamlav_det;
	switch(audio_mode)
	{
		case I2S_AUDIO_OUTPUT:
					audio_mode_s = 0;
					gpio_direction_output(amlav_det->pdata[0].audio_sel_pin,1);
					printk(KERN_INFO"audio_sel_pin set I2S_AUDIO_OUTPUT!\n");
    		break;
    case COAXIAL_OUTPUT:
					audio_mode_s = 1;
					gpio_direction_output(amlav_det->pdata[0].audio_sel_pin,0);
					printk(KERN_INFO"audio_sel_pin set COAXIAL_OUTPUT!\n");
    		break;
    case AV_AUDIO_MUTE:
    			audio_mode_s = 2;
    			aml_audio_i2s_mute();
    			printk(KERN_INFO"AV_AUDIO_MUTE!\n");
    		break;
    case AV_AUDIO_UNMUTE:
    			audio_mode_s = 3;
    			aml_audio_i2s_unmute();
    			printk(KERN_INFO"AV_AUDIO_UNMUTE!\n");
    		break;
    case HDMI_AUDIO_MUTE:
    			audio_mode_s = 4;
    			printk(KERN_INFO"HDMI_AUDIO_MUTE!\n");
    		break;
    case HDMI_AUDIO_UNMUTE:
    			audio_mode_s = 5;
    			printk(KERN_INFO"HDMI_AUDIO_UNMUTE!\n");
    		break;
    default:
    		break;
  }
  return 0;
}
#ifndef TIMER_DET
void audio_det_av_chaneg(int av_insert)
{
	switch(av_insert)
	{
		case AV_IN:
				audio_control(AV_AUDIO_UNMUTE);
				audio_control(HDMI_AUDIO_MUTE);
			break;
		case AV_OUT:
				audio_control(AV_AUDIO_MUTE);
				audio_control(HDMI_AUDIO_UNMUTE);
			break;
	}
}
#endif
////////////////////////////////////for sysfs///////////////////////////////////////////////////////////////
static ssize_t av_det_val(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int audio_mode=0;

	if (!strcmp(attr->attr.name, "audio_mode")) {        
		printk(KERN_INFO"----%s\n", buf);
		sscanf(buf, "%d", &audio_mode);       
	}
	audio_control(audio_mode);
	return count;
}
static ssize_t get_val(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", audio_mode_s);
}
static DEVICE_ATTR(audio_mode, 0666, get_val, av_det_val);
static int aml_is_av_insert(struct aml_av_det_platform_data *pdata)
{
	int ret = -1;
	//gpio check 
	if (pdata->interrupt_pin)
	{
		ret = gpio_get_value(pdata->interrupt_pin);
		//audio handle
	#ifndef TIMER_DET
		printk(KERN_INFO"AV %s\n", ret?"OUT":"IN");
		audio_det_av_chaneg(ret);/*audio mode change by driver */
	#else/*define TIMER_DET */
		if(det_flag != ret){
			det_flag = ret;
			printk(KERN_INFO"AV %s\n", ret?"OUT":"IN");
			if(ret == 0){
				/* AV in*/
				switch_set_state(&pdata->sdev, 2);  // 1 :have mic ;  2 no mic
				printk(KERN_INFO "switch_set_state: 2 \n");
			}
			else{
				/* AV out*/
				switch_set_state(&pdata->sdev, 0);
				printk(KERN_INFO "switch_set_state: 0 \n");
			}
		}
	#endif/*end TIMER_DET */
	}else{
		printk(KERN_ERR"Miss interrupt_pin\n");
	}
		
	return ret;
}
#ifndef TIMER_DET
/* irq fun start*/
extern void disable_irq_nosync(unsigned int irq);
extern void enable_irq(unsigned int irq);
void av_det_irq_init(struct aml_av_det_platform_data *pdata)
{
	if (pdata->irq_in_num && pdata->irq_out_num) {
		gpio_for_irq(pdata->interrupt_pin,
				AML_GPIO_IRQ(pdata->irq_in_num, FILTER_NUM7,
				GPIO_IRQ_FALLING));
		gpio_for_irq(pdata->interrupt_pin,
				AML_GPIO_IRQ(pdata->irq_out_num, FILTER_NUM7,
				GPIO_IRQ_RISING));
		printk(KERN_INFO"av_det_irq_init in:%d out:%d\n",pdata->irq_in_num,pdata->irq_out_num);
	}
}
irqreturn_t aml_av_det_irq(int irq, void *dev_id)
{
	return IRQ_WAKE_THREAD;
}
irqreturn_t aml_av_det_irq_thread(int irq, void *data)
{
	struct aml_av_det_platform_data *pdata = (struct aml_av_det_platform_data *)data;

	if (!pdata) {
		printk(KERN_ERR "missing platform data\n");
		return IRQ_NONE;
	}	
	aml_is_av_insert(pdata);
	
	return IRQ_HANDLED;
}
#else /* define TIMER_DET  */
static void g_timer_handle(unsigned long data)
{
	struct aml_av_det_platform_data *pdata = (struct aml_av_det_platform_data *)data;

	schedule_work(&pdata->work);	
	mod_timer(&g_timer, jiffies + HZ / 100 * det_delay_10ms);
}
static void av_det_work_func(struct work_struct *work)
{
	struct aml_av_det_platform_data *pdata = NULL;

	pdata = container_of(work, struct aml_av_det_platform_data, work);
	aml_is_av_insert(pdata);
}
static void g_timer_add(unsigned int timer_expires_10ms,struct aml_av_det_platform_data *pdata)
{
    init_timer(&g_timer);
    g_timer.function = &g_timer_handle;
    g_timer.expires = jiffies + HZ / 100 * timer_expires_10ms;
    g_timer.data = (unsigned long)pdata;
    add_timer(&g_timer);
}
#endif/* end define TIMER_DET  */
///////////////////////////////////////////////////////////////////////////////////////////////////////
static const struct of_device_id amlogic_av_det_match[] =
{
	{
		.compatible = "amlogic, aml_av_det",
	},
	{},
};
/*av_det class */
static ssize_t class_av_det_val(struct class *cla, struct class_attribute *attr, const char *buf, size_t count)
{
	unsigned int audio_mode=0;

	if (!strcmp(attr->attr.name, "audio_mode")) {        
		printk(KERN_INFO"----%s\n", buf);
		sscanf(buf, "%d", &audio_mode);       
	}
	audio_control(audio_mode);
	return count;
}
static ssize_t class_get_val(struct class *cla, struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", audio_mode_s);
}
//static CLASS_ATTR(audio_mode, 0644, class_get_val, class_av_det_val);
static struct class_attribute av_det_class_attrs[] = {
	__ATTR(audio_mode, 0666, class_get_val, class_av_det_val),
	__ATTR_NULL
};
static struct class av_det_class = {
	.name = "av_det",
	.class_attrs = av_det_class_attrs,
};
///////////////////////////////////////////////////////////////////////////////////////////////////////
static int aml_av_det_suspend(struct platform_device *pdev,pm_message_t state)
{
	printk(KERN_INFO "enter aml_av_det_suspend\n");
	return 0;
}

static int aml_av_det_resume(struct platform_device *pdev)
{
	printk(KERN_INFO "enter aml_av_det_resume\n");
	return 0;
}

static int aml_av_det_probe(struct platform_device *pdev)
{
	struct aml_av_det_platform_data *pdata;
	struct aml_av_det *amlav_det;
	int retval,ret=-1;
	int ret_dts = -1;
	struct device_node *np = pdev->dev.of_node;
	const char *str;
	struct gpio_desc *desc;
#ifndef TIMER_DET
	int value = -1;
	int irq_in_num = -1;
	int irq_out_num = -1;
#endif

	printk(KERN_INFO "enter aml_av_det_probe\n");
	amlav_det = kzalloc(sizeof(struct aml_av_det), GFP_KERNEL);
	if (!amlav_det)
	{   
		printk(KERN_ERR "kzalloc error\n");
		return -ENOMEM;
	}
	pdata=kzalloc(sizeof(struct aml_av_det_platform_data),GFP_KERNEL);
	if(!pdata){
		goto err;
	}
	memset((void* )pdata,0,sizeof(*pdata));
	amlav_det->pdev = pdev;
    
	//dts init
	if(np == NULL ){
		printk(KERN_ERR "np == null \n");
		goto err;
	}
	printk(KERN_INFO"start read av_det dts \n");
	
	//read dev_name
	ret_dts=of_property_read_string(pdev->dev.of_node,"dev_name",&pdata->name);
	if (ret_dts){
		dev_err(&pdev->dev, "read %s  error\n","dev_name");
		goto err;
	}
	printk(KERN_INFO"av_det pdata->name:%s\n",pdata->name);
	
	//read audio_sel_pin
	ret_dts = of_property_read_string(pdev->dev.of_node, "audio_sel_pin", &str);
	if(ret_dts)
	{
		printk(KERN_ERR"Error: can not get audio_sel_pin------%s %d\n",__func__,__LINE__);
		pdata->audio_sel_pin = 0;
	}else{
			desc = of_get_named_gpiod_flags(pdev->dev.of_node,
			"audio_sel_pin", 0, NULL);
			pdata->audio_sel_desc = desc;
			pdata->audio_sel_pin = desc_to_gpio(desc);
	    printk(KERN_INFO"audio_sel_pin is %d\n",pdata->audio_sel_pin);
	}
	if(pdata->audio_sel_pin > 0)
		gpio_request(pdata->audio_sel_pin,OWNER_NAME);
		
	//read interrupt_pin
	ret_dts = of_property_read_string(pdev->dev.of_node, "interrupt_pin", &str);
	if(ret_dts)
	{
		printk(KERN_ERR"Error: can not get interrupt_pin------%s %d\n",__func__,__LINE__);
		pdata->interrupt_pin = 0;
	}else{
			desc = of_get_named_gpiod_flags(pdev->dev.of_node,
			"interrupt_pin", 0, NULL);
			pdata->interrupt_desc = desc;
			pdata->interrupt_pin = desc_to_gpio(desc);
	    printk(KERN_INFO"interrupt_pin is %d\n",pdata->interrupt_pin);
	}
	if(pdata->interrupt_pin > 0)
		gpio_request(pdata->interrupt_pin,OWNER_NAME);

#ifndef TIMER_DET
	//read irq_num
	ret_dts=of_property_read_u32(pdev->dev.of_node,"irq_in_num",&value);
	if (ret_dts){
		dev_err(&pdev->dev, "read %s  error\n","irq_in_num");
		goto err;
	}
	pdata->irq_in_num = value;
	printk(KERN_INFO"irq_in_num is %d\n",pdata->irq_in_num);
	
	value = -1;
	ret_dts=of_property_read_u32(pdev->dev.of_node,"irq_out_num",&value);
	if (ret_dts){
		dev_err(&pdev->dev, "read %s  error\n","irq_out_num");
		goto err;
	}
	pdata->irq_out_num = value;
	printk(KERN_INFO"irq_out_num is %d\n",pdata->irq_out_num);
#endif
	//end dts init
	if (!pdata) {
		printk(KERN_ERR "missing platform data\n");
		retval = -ENODEV;
		goto err;
	}	
#ifndef TIMER_DET
	/* convert irq num */
	if (pdata->irq_in_num && pdata->irq_out_num)
	{
		av_det_irq_init(pdata);/*irq gpio reg init*/
		irq_in_num = irq_of_parse_and_map(
			pdev->dev.of_node, 0);
		irq_out_num = irq_of_parse_and_map(
			pdev->dev.of_node, 1);
			
		pdata->irq_in_num = irq_in_num;
		pdata->irq_out_num = irq_out_num;
		printk(KERN_INFO"convert irq num in=%d ,out=%d \n",pdata->irq_in_num,pdata->irq_out_num);
	}
	/* store pdata*/
	amlav_det->pdata = pdata;
	platform_set_drvdata(pdev, amlav_det);
	Myamlav_det = amlav_det;

	/*Register av detect irq : plug in & unplug*/
	if (pdata->irq_in_num && pdata->irq_out_num) {
		

		ret = request_threaded_irq(irq_in_num,(irq_handler_t)aml_av_det_irq,
		aml_av_det_irq_thread,IRQF_DISABLED, "av_det_in", (void *)pdata);
		if (ret) {
			printk(KERN_ERR"Failed to request av IN detect.\n");
			goto err;
		}else
			printk(KERN_INFO"secuss to request av IN detect.\n");
			
		ret |= request_threaded_irq(irq_out_num,(irq_handler_t)aml_av_det_irq,
		aml_av_det_irq_thread,IRQF_DISABLED,"av_det_out", (void *)pdata);
		if (ret) {
			printk(KERN_ERR"Failed to request av OUT detect.\n");
			goto err;
		}else
			printk(KERN_INFO"secuss to request av OUT detect.\n");
	}
	/*end irq init*/

	ret = device_create_file(&pdev->dev, &dev_attr_audio_mode);
	if (ret < 0){
		printk(KERN_ERR "asoc: failed to add av_det sysfs files\n");
		goto err;
	}
#else/*define TIMER_DET*/

	pdata->sdev.name = "h2w";//for report headphone to android
	ret = switch_dev_register(&pdata->sdev);
	if (ret < 0){
			printk(KERN_ERR "AV_DET: register switch dev failed\n");
			goto err;
	}
	/*work init*/
	INIT_WORK(&pdata->work, av_det_work_func);
	mutex_init(&pdata->lock);
	mutex_lock(&pdata->lock);
	/* det timer init */
	g_timer_add(det_delay_10ms,pdata);
	if(pdata->interrupt_pin > 0)
		gpio_direction_input(pdata->interrupt_pin);

	mutex_unlock(&pdata->lock);

	/* store pdata*/
	amlav_det->pdata = pdata;
	platform_set_drvdata(pdev, amlav_det);
	Myamlav_det = amlav_det;

	ret = device_create_file(&pdev->dev, &dev_attr_audio_mode);
	if (ret < 0){
		printk(KERN_ERR "asoc: failed to add av_det sysfs files\n");
		goto err;
	}
#endif/*end TIMER_DET*/
	/*class register */
	ret = class_register(&av_det_class);
	if (ret){
		printk(KERN_ERR "class_register av_det_class failed\n");
		goto err;
	}
	return 0;

err:
	kfree(amlav_det);
	kfree(pdata);
	if(pdata->audio_sel_pin > 0)
		gpio_free(pdata->audio_sel_pin);
	if(pdata->interrupt_pin > 0)
		gpio_free(pdata->interrupt_pin);
#ifndef TIMER_DET
	if(irq_in_num > 0)
		free_irq(irq_in_num,pdata);
	if(irq_out_num > 0)
		free_irq(irq_out_num,pdata);
#endif
	return retval;
}

static int __exit aml_av_det_remove(struct platform_device *pdev)
{
	struct aml_av_det *amlav_det = platform_get_drvdata(pdev);

	printk(KERN_INFO "enter aml_av_det_remove\n");
#ifndef TIMER_DET
	if(amlav_det->pdata[0].irq_in_num > 0)
		free_irq(amlav_det->pdata[0].irq_in_num,amlav_det->pdata);
	if(amlav_det->pdata[0].irq_out_num > 0)
		free_irq(amlav_det->pdata[0].irq_out_num,amlav_det->pdata);
#else
	mutex_lock(&amlav_det->pdata[0].lock);
	del_timer(&g_timer);
	cancel_work_sync(&amlav_det->pdata[0].work);
	mutex_unlock(&amlav_det->pdata[0].lock);
	det_flag = 2;
#endif
	audio_mode_s = 0;
	platform_set_drvdata(pdev, NULL);
	class_unregister(&av_det_class);
	kfree(amlav_det);

	return 0;
}

static struct platform_driver aml_av_det_driver = {
	.driver = {
		.name = "aml_av_det",
	.owner = THIS_MODULE,
	.of_match_table = of_match_ptr(amlogic_av_det_match),
	},
	.probe = aml_av_det_probe,
	.remove = __exit_p(aml_av_det_remove),
	.suspend = aml_av_det_suspend,
	.resume  = aml_av_det_resume,
};

static int __init aml_av_det_init(void)
{
	int ret = -1;
  
	printk(KERN_INFO "enter aml_av_det_init\n");
	ret = platform_driver_register(&aml_av_det_driver);
  
	if (ret != 0) {
		printk(KERN_ERR "failed to register av_det driver, error %d\n", ret);
		return -ENODEV;
	}
	return ret;    
}
module_init(aml_av_det_init);

static void __exit aml_av_det_exit(void)
{
	//printk(KERN_INFO "enter aml_av_det_exit\n");
	platform_driver_unregister(&aml_av_det_driver);
}
module_exit(aml_av_det_exit);

MODULE_DESCRIPTION("Amlogic av_det driver");
MODULE_AUTHOR("dianzhong.huo <dianzhong.huo@amlogic.com>");
MODULE_LICENSE("GPL");
