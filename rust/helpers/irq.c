// SPDX-License-Identifier: GPL-2.0

#include <linux/interrupt.h>

__rust_helper int rust_helper_request_irq(unsigned int irq,
					  irq_handler_t handler,
					  unsigned long flags, const char *name,
					  void *dev)
{
	return request_irq(irq, handler, flags, name, dev);
}

__rust_helper void rust_helper_irq_wake_thread(unsigned int irq, void *dev_id)
{
	irq_wake_thread(irq, dev_id);
}
