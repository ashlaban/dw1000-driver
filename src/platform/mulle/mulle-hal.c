/**
 * \file mulle-hal.c
 * \author Kim Albertsson
 * \date 2014-Oct-16
 *
 * \brief Implementation of the Hardware Abstraction Layer for the mulle platform.
 *
 * \details The mulle implementation of the HAL covers:
 * 	- Interrupts.
 * 		One pin is used for interrupts. The PE5 (port E, pin 5) pin is used 
 * 		for interrupts and should be connected to GPIO8 of the dw1000.
 * 
 * \note The mulle implementation assumes that contiki is available and exposes
 * a specific callback, \ref dw_interrupt_callback_proc, for use in a contiki 
 * application. If contiki is not available the interrupt 
 *
 * \todo Generalise the interrupt callback so that the system can be used with 
 * and without contiki. This could be done by introducing a contiki specific 
 * HAL implementation, something akin to mulle-contiki-hal.c.
 *
 * \todo Add to documention. Version of the mulle platform and corresponding arm processor.
 */

#include "dw1000-hal.h"

#include "contiki.h"
#include "MK60N512VMD100.h"


/**
 * \ingroup interrupt
 * \addtogroup mulle-contiki-interrupt Specifics for mulle-contiki
 *
 * \brief The interrupt system for mulle using contiki is extented with a
 * callback process that is polled whenever there is an interrupt. This gives
 * you as the programmer total control of the interrupt handler without needing
 * to worry about touching the actual interrupt vector.
 * 
 * @{
 */
/**
 * \brief Contiki process callback for interrupt handling.
 */
extern struct process dw_interrupt_callback_proc;
/** @}*/


/**
 * \brief Arm cortex m4 interrupt vector for port E pins.
 *
 * \todo Takes over interrupt vector completely, can you mix-in the interrupt 
 * functionality?
 */
extern void _isr_gpio_e(void);

/**
 * \brief Internal interrupt callback.
 *
 * \note I actually don't remember why I separated the internal and external 
 * interface. Might only be that I thought it was a good idea to have the 
 * sparatation...
 */
static void _dw_hal_interrupt_callback(void);

void dw_hal_init(void)
{
	SIM_SCGC5  |= SIM_SCGC5_PORTE_MASK; // Enable clock for port E
	PORTE_PCR5  = 0x00000100; // Function sel, alt 1, gpio
	PORTE_PCR5 |= 0x01000000; // Clear interrupt
	PORTE_PCR5 |= 0x00090000; // IRQ enable, on logic rising edge
	// uint32_t irqStatus = PORTE_PCR5 & 0x01000000; // Get interrupt status

	dw_hal_interrupt_handler  = _isr_gpio_e;
	dw_hal_interrupt_callback = _dw_hal_interrupt_callback;
}

void dw_hal_enable_interrupt(void)
{
	PORTE_PCR5 |= 1<<24;
	NVICICPR2   = 1<<27;
	NVICISER2  |= 1<<27;
}

void dw_hal_disable_interrupt(void)
{
	NVICICER2 |= 1<<27;
}

void dw_hal_clear_pending_interrupt(void)
{
	PORTE_PCR5 |= (1<<24);
	NVICICPR2 = 1<<27;
}

static void _dw_hal_interrupt_callback(void)
{
	process_poll(&dw_interrupt_callback_proc);
}

/* Interrupt handler */
void _isr_gpio_e(void)
{
	dw_hal_disable_interrupt();
	
	// Clear interrupt status
	dw_hal_clear_pending_interrupt();
	
	// Put handling on queue
	dw_hal_interrupt_callback();
	
	// Enable interrupt again!
	dw_hal_enable_interrupt();
}