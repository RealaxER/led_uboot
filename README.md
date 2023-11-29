## Environment
#### U-boot tag: v2020.04(Tested)
#### Enable this U-boot configuration (CONFIG_DEMO_LED):
Add these lines into  file 
```
configs/am335x_evm_defconfig
```

``` 
CONFIG_DM_GPIO=y
CONFIG_LED=y  
CONFIG_DEMO_LED=y  
CONFIG_CMD_DEMO_LED=y  
```
#### Connect a led into the GPIO1.14 pin

## Driver gpio
```
cd /bbb/u-boot/drivers
```
### Makefile device driver 
```
obj-$(CONFIG_DEMO_LED) += demo_led/
```
### Kconfig device driver 
```
source "drivers/demo_led/Kconfig"
```
### Create demo_led driver
```
mkdir -p demo_led/
cd demo_led/
```
### Makefile demo_led
```make
obj-$(CONFIG_$(SPL_)_DEMO_LED) += demo_led.o
```
### Kconfig demo_led
```Kconfig
menu "LED DEMO Support"

config DEMO_LED 
        bool "Enable DEMO LED support"
        depends on DM && DM_GPIO
        help
          This is just a demo to show how to simply control a led in u-boot.
endmenu
```

### demo_led.c
```c
#include <common.h>
#include <dm.h>
#include <errno.h>
#include <led.h>
#include <malloc.h>
#include <asm-generic/gpio.h>
#include <dm/lists.h>

struct led_gpio_priv {
	struct gpio_desc gpio;
};

static int gpio_led_set_state(struct udevice *dev, enum led_state_t state)
{
	struct led_gpio_priv *priv = dev_get_priv(dev);
	int ret;

	if (!dm_gpio_is_valid(&priv->gpio))
		return -EREMOTEIO;
	switch (state) {
	case LEDST_OFF:
	case LEDST_ON:
		break;
	case LEDST_TOGGLE:
		ret = dm_gpio_get_value(&priv->gpio);
		if (ret < 0)
			return ret;
		state = !ret;
		break;
	default:
		return -ENOSYS;
	}

	return dm_gpio_set_value(&priv->gpio, state);
}

static enum led_state_t gpio_led_get_state(struct udevice *dev)
{
	struct led_gpio_priv *priv = dev_get_priv(dev);
	int ret;

	if (!dm_gpio_is_valid(&priv->gpio))
		return -EREMOTEIO;
	ret = dm_gpio_get_value(&priv->gpio);
	if (ret < 0)
		return ret;

	return ret ? LEDST_ON : LEDST_OFF;
}

static int led_gpio_probe(struct udevice *dev)
{
	struct led_uc_plat *uc_plat = dev_get_uclass_plat(dev);
	struct led_gpio_priv *priv = dev_get_priv(dev);
	int ret;

	/* Ignore the top-level LED node */
	if (!uc_plat->label)
		return 0;

	ret = gpio_request_by_name(dev, "gpios", 0, &priv->gpio, GPIOD_IS_OUT);
	if (ret)
		return ret;

	return 0;
}

static int led_gpio_remove(struct udevice *dev)
{
	/*
	 * The GPIO driver may have already been removed. We will need to
	 * address this more generally.
	 */
#ifndef CONFIG_SANDBOX
	struct led_gpio_priv *priv = dev_get_priv(dev);

	if (dm_gpio_is_valid(&priv->gpio))
		dm_gpio_free(dev, &priv->gpio);
#endif

	return 0;
}

static int led_gpio_bind(struct udevice *parent)
{
	struct udevice *dev;
	ofnode node;
	int ret;

	dev_for_each_subnode(node, parent) {
		struct led_uc_plat *uc_plat;
		const char *label;

		label = ofnode_read_string(node, "label");
		if (!label) {
			debug("%s: node %s has no label\n", __func__,
			      ofnode_get_name(node));
			return -EINVAL;
		}
		ret = device_bind_driver_to_node(parent, "demo_led",
						 ofnode_get_name(node),
						 node, &dev);
		if (ret)
			return ret;
		uc_plat = dev_get_uclass_plat(dev);
		uc_plat->label = label;
	}

	return 0;
}

static const struct led_ops gpio_led_ops = {
	.set_state	= gpio_led_set_state,
	.get_state	= gpio_led_get_state,
};

static const struct udevice_id led_gpio_ids[] = {
	{ .compatible = "demo-uboot-leds" },
	{ }
};

U_BOOT_DRIVER(demo_led) = {
	.name	= "demo_led",
	.id	= UCLASS_LED,
	.of_match = led_gpio_ids,
	.ops	= &gpio_led_ops,
	.priv_auto = sizeof(struct led_gpio_priv),
	.bind	= led_gpio_bind,
	.probe	= led_gpio_probe,
	.remove	= led_gpio_remove,
};
```

## CMD
```
cd /bbb/u-boot/cmd
```
### Kconfig cmd
```
config CMD_DEMO_LED
    bool "demo_led"
    help
      Enable the 'demo_led' command which allows for control of LEDs. In
	  this case, it is P1.14.
```
### Makfile cmd
```
obj-$(CONFIG_CMD_DEMO_LED) += demo_led.o
```
### demo_led.c
```c
#include <common.h>
#include <command.h>
#include <dm.h>
#include <led.h>
#include <dm/uclass-internal.h>

static const char *const state_label[] = {
	[LEDST_OFF]	= "off",
	[LEDST_ON]	= "on",
	[LEDST_TOGGLE]	= "toggle",
};

enum led_state_t get_demo_led_cmd(char *var)
{
	int i;

	for (i = 0; i < LEDST_COUNT; i++) {
		if (!strncmp(var, state_label[i], strlen(var)))
			return i;
	}

	return -1;
}

static int show_led_state(struct udevice *dev)
{
	int ret;

	ret = led_get_state(dev);
	if (ret >= LEDST_COUNT)
		ret = -EINVAL;
	if (ret >= 0)
		printf("%s\n", state_label[ret]);

	return ret;
}

static int show_label(void)
{
	struct udevice *dev;
	for (uclass_find_first_device(UCLASS_LED, &dev);
		 dev;
		 uclass_find_next_device(&dev)) {
			struct led_uc_plat *plat = dev_get_uclass_plat(dev);

			if (!plat->label)
					continue;
			printf("%s \n", plat->label);
	}

	return 0;
}

int do_demo_led(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	enum led_state_t cmd;
	const char *led_label;
	struct udevice *dev;

	int ret;

	/* Validate arguments */
	if (argc < 2)
		return CMD_RET_USAGE;
	led_label = argv[1];
	if (strncmp(led_label, "show", 4) == 0)
		return show_label();

	cmd = argc > 2 ? get_demo_led_cmd(argv[2]) : LEDST_COUNT;

	ret = led_get_by_label(led_label, &dev);
	if (ret) {
		printf("LED '%s' not found (err=%d)\n", led_label, ret);
		return CMD_RET_FAILURE;
	}
	switch (cmd) {
	case LEDST_OFF:
	case LEDST_ON:
	case LEDST_TOGGLE:
		ret = led_set_state(dev, cmd);
		break;

	case LEDST_COUNT:
		printf("LED '%s': ", led_label);
		ret = show_led_state(dev);
		break;
	}
	if (ret < 0) {
		printf("LED '%s' operation failed (err=%d)\n", led_label, ret);
		return CMD_RET_FAILURE;
	}

	return 0;
}

U_BOOT_CMD(
	demo_led, 4, 1, do_demo_led,
	"manage DEMO LEDs",
	"<led_label> on|off|toggle\tChange LED state\n"
	"demo_led <led_label>\tGet LED state\n"
	"demo_led show\t Show LED label"
);
```


## Device tree
```
bbb/u-boot/arch/arm/dts/am335x-boneblack.dts
```
### Create a led node in device-tree  
```
demo_leds {  
		pinctrl-names = "default";  
		pinctrl-0 = <&demo_leds>;  

		compatible = "demo-uboot-leds";  
  
		used_led@1 {  
				label = "led-1_14";  
				gpios = <&gpio1 14 GPIO_ACTIVE_HIGH>;  
				default-state = "off";  
		};  
};  
```
### Create pinctrl for this node in device-tree  
```
&am33xx_pinmux {
	demo_leds: demo_leds {
		pinctrl-single,pins = <
			0x38 (PIN_OUTPUT_PULLDOWN | MUX_MODE7) /* gpmc_ad14.gpio1_14 */
		>;
	};
};
```

## Step to run demo
### 1. Compile the new u-boot with demo-led driver in it
```
export CC=`pwd`/gcc-11.3.0-nolibc/arm-linux-gnueabi/bin/arm-linux-gnueabi-
make ARCH=arm CROSS_COMPILE=${CC} am335x_evm_defconfig
make ARCH=arm CROSS_COMPILE=${CC}
```
### 2. Copy MLO, u-boot.img, u-boot.bin(optional) to boot parition of SDcard
### 3. Run these command on U-boot commandline  
```
=> demo_led show  
led:P1:14  
=> demo_led led:P1:14  
LED 'led:P1:14': off  
=> demo_led led:P1:14 on  
=> demo_led led:P1:14  
LED 'led:P1:14': on  
=> demo_led led:P1:14 off  
=> demo_led led:P1:14  
LED 'led:P1:14': off  
=> demo_led  
demo_led - manage DEMO LEDs  

Usage:  
demo_led <led_label> on|off|toggle      Change LED state  
demo_led <led_label>    Get LED state  
demo_led show    Show LED label  
```
 
