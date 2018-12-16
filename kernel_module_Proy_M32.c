#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include "../address_map_arm.h"
#include "../interrupt_ID.h"

#define HPS_TIMER2_BASE 0XFFD00000
#define HPS_TIMER2_SPAN 0X14
#define HPS_TIMER3_BASE 0XFFD01000
#define HPS_TIMER3_SPAN 0X14
#define HPS_TIMER2_IRQ 201
#define HPS_TIMER3_IRQ 202

void * LW_virtual; // Lightweight bridge base address
volatile int *LEDR_ptr, *KEY_ptr, *HPS_Timer2_ptr, *HPS_Timer3_ptr; // virtual addresses
void * HPS_Timer2_virtual; // Timer2 base address
void * HPS_Timer3_virtual; // Timer3 base address





/*  irq_handler_t irq_handler(int irq, void *dev_id, struct pt_regs *regs) //lo que se ejecuta cada vez que salta la interrupcion
 {
 *LEDR_ptr = *LEDR_ptr + 1;
 // Clear the Edgecapture register (clears current interrupt)
 *(KEY_ptr + 3) = 0xF;
 return (irq_handler_t) IRQ_HANDLED;
 } */
 
 irq_handler_t irq_handler_HPS_timer2(int irq, void *dev_id, struct pt_regs *regs) //lo que se ejecuta cada vez que salta la interrupcion
 {
	 
 int value, value2;
 *LEDR_ptr = *LEDR_ptr ^ 0x1;
//  *LEDR_ptr = *LEDR_ptr + 1;
  value = *(HPS_Timer2_ptr+3); //borrar flag


 return (irq_handler_t) IRQ_HANDLED;
 }
 
  irq_handler_t irq_handler_HPS_timer3(int irq, void *dev_id, struct pt_regs *regs) //lo que se ejecuta cada vez que salta la interrupcion
 {
	 
 int value;
 *LEDR_ptr = *LEDR_ptr ^ 0x2;
 // *LEDR_ptr = *LEDR_ptr + 1;
  value = *(HPS_Timer3_ptr+3); //borrar flag

 return (irq_handler_t) IRQ_HANDLED;
 }
 
 
 
 static int __init initialize_handler(void)	
 {
 int value, value2;
 // generate a virtual address for the FPGA lightweight bridge
 LW_virtual = ioremap_nocache (LW_BRIDGE_BASE, LW_BRIDGE_SPAN);

 LEDR_ptr = LW_virtual + LEDR_BASE; // virtual address for LEDR port
 *LEDR_ptr = 0x001; // turn on the righttmost light
 
 //crea direcciones virtuales de los timers
 HPS_Timer2_virtual = ioremap_nocache (HPS_TIMER2_BASE, HPS_TIMER2_SPAN); 
 HPS_Timer3_virtual = ioremap_nocache (HPS_TIMER3_BASE, HPS_TIMER3_SPAN);
 
 HPS_Timer2_ptr = HPS_Timer2_virtual; //salta cada segundo
 
   *(HPS_Timer2_ptr + 2) = 0x00FE;  
   *(HPS_Timer2_ptr + 3) = 0X00FE;   
   *HPS_Timer2_ptr = 25000000;
   *(HPS_Timer2_ptr + 2) = 0x0003;   
   
   
HPS_Timer3_ptr = HPS_Timer3_virtual; //salta cada dos segundos
 
   *(HPS_Timer3_ptr + 2) = 0x00FE;  
   *(HPS_Timer3_ptr + 3) = 0X00FE;   
   *HPS_Timer3_ptr = 50000000;
   *(HPS_Timer3_ptr + 2) = 0x0003;

/*  KEY_ptr = LW_virtual + KEY_BASE; // virtual address for KEY port
 *(KEY_ptr + 3) = 0xF; // Clear the Edgecapture register
 *(KEY_ptr + 2) = 0xF; // Enable IRQ generation for the 4 buttons */

 // Register the interrupt handler.
 value = request_irq (HPS_TIMER2_IRQ, (irq_handler_t) irq_handler_HPS_timer2, IRQF_SHARED,
 "HPS_Timer2_irq_handler", (void *) (irq_handler_HPS_timer2));
 
  // Register the interrupt handler.
 value2 = request_irq (HPS_TIMER3_IRQ, (irq_handler_t) irq_handler_HPS_timer3, IRQF_SHARED,
 "HPS_Timer3_irq_handler", (void *) (irq_handler_HPS_timer3));
 
 return value;
 }
 
 
 static void __exit cleanup_handler(void)
 {
 *LEDR_ptr = 0; // Turn off LEDs and de-register irq handler
 free_irq (HPS_TIMER2_IRQ, (void*) irq_handler_HPS_timer2);
 free_irq (HPS_TIMER3_IRQ, (void*) irq_handler_HPS_timer3);
 }
 module_init(initialize_handler);
 module_exit(cleanup_handler);