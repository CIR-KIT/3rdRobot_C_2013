#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
 .name = KBUILD_MODNAME,
 .init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
 .exit = cleanup_module,
#endif
 .arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x568fba06, "module_layout" },
	{ 0x335879f5, "usb_deregister" },
	{ 0x9d13d499, "usb_register_driver" },
	{ 0xb72041c9, "usb_alloc_coherent" },
	{ 0x9f6b4c62, "down_interruptible" },
	{ 0x77e2f33, "_copy_from_user" },
	{ 0x7c3c2550, "usb_register_dev" },
	{ 0x90e78de4, "usb_alloc_urb" },
	{ 0x5a34a45c, "__kmalloc" },
	{ 0xc79b071e, "usb_get_dev" },
	{ 0x6395be94, "__init_waitqueue_head" },
	{ 0x83800bfa, "kref_init" },
	{ 0x7a172b67, "kmem_cache_alloc_trace" },
	{ 0x4d884691, "malloc_sizes" },
	{ 0x17f2f9a6, "usb_free_urb" },
	{ 0x37a0cba, "kfree" },
	{ 0x43596e71, "usb_put_dev" },
	{ 0x6f8423e7, "usb_deregister_dev" },
	{ 0x24cbdf32, "dev_set_drvdata" },
	{ 0x9775cdc, "kref_get" },
	{ 0x64b410e1, "dev_get_drvdata" },
	{ 0xb14b4c3, "usb_find_interface" },
	{ 0x68aca4ad, "down" },
	{ 0xfa66f77c, "finish_wait" },
	{ 0x1000e51, "schedule" },
	{ 0x5c8b5ce8, "prepare_to_wait" },
	{ 0x9021c4eb, "current_task" },
	{ 0xc8b57c27, "autoremove_wake_function" },
	{ 0x7ad99b8f, "usb_bulk_msg" },
	{ 0x17b5605, "usb_free_coherent" },
	{ 0xcf21d241, "__wake_up" },
	{ 0x71e3cecb, "up" },
	{ 0x940602e5, "down_trylock" },
	{ 0x27e1a049, "printk" },
	{ 0x71de9b3f, "_copy_to_user" },
	{ 0x48035c88, "usb_submit_urb" },
	{ 0xa1c76e0a, "_cond_resched" },
	{ 0xd5b037e1, "kref_put" },
	{ 0x14568c91, "usb_kill_urb" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=usbcore";

MODULE_ALIAS("usb:v0FF8p0020d*dc*dsc*dp*ic*isc*ip*");
