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
#define FPGA_INTERVAL_TIMER_BASE 0xFF202000
#define FPGA_INTERVAL_TIMER_SPAN 0X20
#define HPS_TIMER2_IRQ 201
#define HPS_TIMER3_IRQ 202
#define FPGA_INTERVAL_TIMER_IRQ 72

void * LW_virtual; // Lightweight bridge base address
volatile int *LEDR_ptr, *HPS_Timer2_ptr, *HPS_Timer3_ptr, *FPGA_Timer_Interval_ptr; // virtual addresses
void * HPS_Timer2_virtual; // Timer2 base address
void * HPS_Timer3_virtual; // Timer3 base address
void * FPGA_Timer_Interval_Virtual; // Fpga Timer base address

 
 //rutina de interrupcion de timer2
 irq_handler_t irq_handler_HPS_timer2(int irq, void *dev_id, struct pt_regs *regs) //lo que se ejecuta cada vez que salta la interrupcion
 {
	 
 int value, value2;
 *LEDR_ptr = *LEDR_ptr ^ 0x1;
  value = *(HPS_Timer2_ptr+3); //borrar flag


 return (irq_handler_t) IRQ_HANDLED;
 }
 
 
  //rutina de interrupcion de timer3
  irq_handler_t irq_handler_HPS_timer3(int irq, void *dev_id, struct pt_regs *regs) //lo que se ejecuta cada vez que salta la interrupcion
 {
	 
 int value;
 *LEDR_ptr = *LEDR_ptr ^ 0x2;

 //mover servo
 
 
 
 
 
 
 
  value = *(HPS_Timer3_ptr+3); //borrar flag

 return (irq_handler_t) IRQ_HANDLED;
 }
 
 
 //rutina de interrupcion de FPGA Interval timer
 irq_handler_t irq_handler_FPGA_timer(int irq, void *dev_id, struct pt_regs *regs) //lo que se ejecuta cada vez que salta la interrupcion
 {
	 
 int value, value2;
 *LEDR_ptr = *LEDR_ptr ^ 0x4;
 *FPGA_Timer_Interval_ptr = 0x10; //borrar flag, escribir cualquier valor en registro status


 return (irq_handler_t) IRQ_HANDLED;
 }
 
 
 //inicializacion 
 static int __init initialize_handler(void)	
 {
 int value, value2,value3;
 // generate a virtual address for the FPGA lightweight bridge
 LW_virtual = ioremap_nocache (LW_BRIDGE_BASE, LW_BRIDGE_SPAN);

 LEDR_ptr = LW_virtual + LEDR_BASE; // virtual address for LEDR port
 *LEDR_ptr = 0x001; // turn on the righttmost light
 
 //crea direcciones virtuales de los timers
 HPS_Timer2_virtual = ioremap_nocache (HPS_TIMER2_BASE, HPS_TIMER2_SPAN); 
 HPS_Timer3_virtual = ioremap_nocache (HPS_TIMER3_BASE, HPS_TIMER3_SPAN);
 FPGA_Timer_Interval_Virtual = ioremap_nocache (FPGA_INTERVAL_TIMER_BASE, FPGA_INTERVAL_TIMER_SPAN);
 
 
 //configuracion timer2. reloj de 25MHz
 HPS_Timer2_ptr = HPS_Timer2_virtual; //salta cada segundo
 
   *(HPS_Timer2_ptr + 2) = 0x00FE;  //deshabilita timer
   *(HPS_Timer2_ptr + 3) = 0X00FE;  // borra flag
   *HPS_Timer2_ptr = 25000000;		//Load Value
   *(HPS_Timer2_ptr + 2) = 0x0003;   //Auto Reload & Enable
   
//configuracion timer3. reloj de 25MHz
HPS_Timer3_ptr = HPS_Timer3_virtual; //salta cada dos segundos
 
   *(HPS_Timer3_ptr + 2) = 0x00FE;    //deshabilita timer
   *(HPS_Timer3_ptr + 3) = 0X00FE;    // borra flag
   *HPS_Timer3_ptr = 50000000;		//Load Value
   *(HPS_Timer3_ptr + 2) = 0x0003;  //Auto Reload & Enable

   
   
//configuracion timer FPGA Interval. reloj de 100MHz
FPGA_Timer_Interval_ptr = FPGA_Timer_Interval_Virtual; //salta cada tres segundos
 
   *(FPGA_Timer_Interval_ptr + 1) = *(FPGA_Timer_Interval_ptr + 1) | 0x8; //parar contador
   *(FPGA_Timer_Interval_ptr + 1) = *(FPGA_Timer_Interval_ptr + 1) | 0x2;  //set auto reload (CONT)
   *(FPGA_Timer_Interval_ptr + 2) = 0xA300;   //valor inicial , low
   *(FPGA_Timer_Interval_ptr + 3) = 0x11E1;	  //valor inicial , high
   *(FPGA_Timer_Interval_ptr + 1) = *(FPGA_Timer_Interval_ptr + 1) | 0x1;  //habilitar interrupcion
   *(FPGA_Timer_Interval_ptr + 1) = *(FPGA_Timer_Interval_ptr + 1) | 0x4;  //arrancar timer;
   

 // Register the interrupt handler.
 value = request_irq (HPS_TIMER2_IRQ, (irq_handler_t) irq_handler_HPS_timer2, IRQF_SHARED,
 "HPS_Timer2_irq_handler", (void *) (irq_handler_HPS_timer2));
 
  // Register the interrupt handler.
 value2 = request_irq (HPS_TIMER3_IRQ, (irq_handler_t) irq_handler_HPS_timer3, IRQF_SHARED,
 "HPS_Timer3_irq_handler", (void *) (irq_handler_HPS_timer3));
 
   // Register the interrupt handler.
 value3 = request_irq (FPGA_INTERVAL_TIMER_IRQ, (irq_handler_t) irq_handler_FPGA_timer, IRQF_SHARED,
 "FPGA_Timer_irq_handler", (void *) (irq_handler_FPGA_timer));
 
 return value;
 }
 
 
 static void __exit cleanup_handler(void)
 {
 *LEDR_ptr = 0; // Turn off LEDs and de-register irq handler
 free_irq (HPS_TIMER2_IRQ, (void*) irq_handler_HPS_timer2);
 free_irq (HPS_TIMER3_IRQ, (void*) irq_handler_HPS_timer3);
 free_irq (FPGA_INTERVAL_TIMER_IRQ, (void*) irq_handler_FPGA_timer);
 }
 module_init(initialize_handler);
 module_exit(cleanup_handler);