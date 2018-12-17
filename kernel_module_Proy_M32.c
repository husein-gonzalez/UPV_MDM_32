/********************************************************
Proyecto: Sistema de control de movimiento de una plataforma para una cámara

File: kernel_module_Proy_M32.c

Autor: Husein Gonzalez
*********************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include "../address_map_arm.h"
#include "../interrupt_ID.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Altera University Program");
MODULE_DESCRIPTION("DE1SoC Servo control");

#define HPS_TIMER2_BASE 0XFFD00000
#define HPS_TIMER2_SPAN 0X14
#define HPS_TIMER3_BASE 0XFFD01000
#define HPS_TIMER3_SPAN 0X14
#define HPS_TIMER2_IRQ 201
#define HPS_TIMER3_IRQ 202
#define FPGA_INTERVAL_TIMER_IRQ 72

void * LW_virtual; // Lightweight bridge base address
volatile int *LEDR_ptr, *HPS_Timer2_ptr, *HPS_Timer3_ptr, *FPGA_Timer_Interval_ptr, *JP2_ptr; // virtual addresses
void * HPS_Timer2_virtual; // Timer2 base address
void * HPS_Timer3_virtual; // Timer3 base address
void * FPGA_Timer_Interval_Virtual; // Fpga Timer base address

int servox_time=0,servoX_counter=0,grados_X=0, grados_Y=0;
int servoY_counter=0;
int time_acel_x=0, time_acel_y=0, contador_acel_x=0, contador_acel_y=0, acel_y_F=0, acel_x_F=0, acel_y_Edge=0, acel_x_Edge=0;
int contador_total_acel=0;
int contador_filtro_acel=0;
int filtro_acel_temp_X=0, filtro_acel_temp_Y=0;
int acel_max_x=0, acel_min_x=10000;
int acel_max_y=0, acel_min_y=10000;


 
 //rutina de interrupcion de timer2 (control del servo X)
 irq_handler_t irq_handler_HPS_timer2(int irq, void *dev_id, struct pt_regs *regs) //lo que se ejecuta cada vez que salta la interrupcion
 {
	 
 int value, value2;
 
 //el periodo del PWM es 50Hz->20ms. dividimos esos 20ms en 2000 partes para tener resolucion de 10us, lo que nos permite mover de 1º en 1º
 
 if(servoX_counter>=2000)
	 servoX_counter=0;
 else
	 servoX_counter++; 
 
 // 0º= 600us, 90º=1500us     180 º = 2400us
 
 
 // el aceleromtero nos pasa los grados en datos de 60 a 240
 
if (servoX_counter < grados_X)		
		
	*JP2_ptr = *JP2_ptr | 0x20; //pin 5 a 1

else
	
		*JP2_ptr = *JP2_ptr & 0xDF;; //pin 5 a 0

  value = *(HPS_Timer2_ptr+3); //borrar flag

 return (irq_handler_t) IRQ_HANDLED;
 }
 
 
  //rutina de interrupcion de timer3 (control del servo y)
  irq_handler_t irq_handler_HPS_timer3(int irq, void *dev_id, struct pt_regs *regs) //lo que se ejecuta cada vez que salta la interrupcion
 {
	 
 int value;
 *LEDR_ptr = *LEDR_ptr ^ 0x2;

 //mover servo y
  
 //el periodo del PWM es 50Hz->20ms. dividimos esos 20ms en 2000 partes para tener resolucion de 10us, lo que nos permite mover de 1º en 1º
 
 if(servoY_counter>=2000)
	 servoY_counter=0;
 else
	 servoY_counter++; 
 
 // 0º= 600us, 90º=1500us     180 º = 2400us
 
 
 // el aceleromtero nos pasa los grados en datos de 60 a 240
 
if (servoY_counter < grados_Y)		
		
	*JP2_ptr = *JP2_ptr | 0x80; //pin 7 a 1

else
	
		*JP2_ptr = *JP2_ptr & 0x7F;; //pin 7 a 0 
 
  value = *(HPS_Timer3_ptr+3); //borrar flag

 return (irq_handler_t) IRQ_HANDLED;
 }
 
 
 //rutina de interrupcion de FPGA Interval timer
 irq_handler_t irq_handler_FPGA_timer(int irq, void *dev_id, struct pt_regs *regs) //lo que se ejecuta cada vez que salta la interrupcion
 {
	 
	 int value, acel_x_bit, acel_y_bit;
	 int temp;
	 
	 value= *JP2_ptr;  // leer el valor de los pines
	 
	 //aislar los bits 17 y 19	 
	 acel_x_bit = value & 0x20000 ; //bit17 ->x
	 acel_y_bit = value & 0x80000 ; //bit19 ->y
	 
	 if(contador_total_acel>=10000)	//ha finalizado periodo de 10ms
	 {
		//autocalibrar acelerometro
		//medir maximo y minimo
		//los grados se le dan al servo en decenas de microsegundos: 60 - 2400  == 0º - 180º
		//grados del servo = (tiempo en alta del acelerometro)- timepo minimo en alta / (rango acelerometro/rango servo)
		
		// X limits
		if(contador_acel_x>acel_max_x)  // valor maximo del acelerometro
			acel_max_x=contador_acel_x;
		
		if(contador_acel_x<acel_min_x)	// valor minimo del acelerometro
			acel_min_x=contador_acel_x;	
		// Y limits
		if(contador_acel_y>acel_max_y)  // valor maximo del acelerometro
			acel_max_y=contador_acel_y;
		
		if(contador_acel_y<acel_min_y)	// valor minimo del acelerometro
			acel_min_y=contador_acel_y;			
		
		//transformar de microsegundos a grados X
		contador_acel_x =  contador_acel_x - acel_min_x; //quitar minimo
		temp = (acel_max_x - acel_min_x)/180;		 //escalar		
		temp = (contador_acel_x / temp ) + 60;	// convertir en grados, de 60 a 240 =180º
				
		printk(" minx: %d    maxx: %d, gradosx %d \n\r", acel_min_x,acel_max_x,grados_X );
		 
		 //evitar sobrepasar limites
		 if(temp > 240)
			 grados_X=240;
		 else if(temp<60)
			 grados_X=60;
		 else
			 grados_X = temp;
		 
		//transformar de microsegundos a grados Y
		contador_acel_y =  contador_acel_y - acel_min_y; //quitar minimo
		temp = (acel_max_y - acel_min_y)/180;		 //escalar		
		temp = (contador_acel_y / temp ) + 60;	// convertir en grados, de 60 a 240 =180º	 
		 
		 //evitar sobrepasar limites
		 if(temp > 240)
			 grados_Y=240;
		 else if(temp<60)
			 grados_Y=60;
		 else
			 grados_Y = temp;
		 
		 //reiniciar variables
		contador_total_acel=0;
		contador_acel_x=0;
		contador_acel_y=0;		
	 }
	else					//esta dentro del periodo de 10ms
	{
		if(acel_x_bit)
			contador_acel_x++;
		
		if(acel_y_bit)
			contador_acel_y++;
		
		contador_total_acel++;
	
	}
	    
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
 
 JP2_ptr = LW_virtual + JP2_BASE; // virtual address for GPIO1 port
 
 //crea direcciones virtuales de los timers
 HPS_Timer2_virtual = ioremap_nocache (HPS_TIMER2_BASE, HPS_TIMER2_SPAN); 
 HPS_Timer3_virtual = ioremap_nocache (HPS_TIMER3_BASE, HPS_TIMER3_SPAN);
 //FPGA_Timer_Interval_Virtual = ioremap_nocache (FPGA_INTERVAL_TIMER_BASE, FPGA_INTERVAL_TIMER_SPAN);
 
 
 //configura GPIOS JP2
 
 *(JP2_ptr + 1) = 0xA0;   // 5 y 7 son salidas. pin 5 = servo x, pin7 = servo y
 
 
 
 //configuracion timer2. reloj de 25MHz
 HPS_Timer2_ptr = HPS_Timer2_virtual; //salta cada 10 microsegundos (1800us == 180º)
 
   *(HPS_Timer2_ptr + 2) = 0x00FE;  //deshabilita timer
   *(HPS_Timer2_ptr + 3) = 0X00FE;  // borra flag
   *HPS_Timer2_ptr = 250;		//Load Value, salta cada 10us
   *(HPS_Timer2_ptr + 2) = 0x0003;   //Auto Reload & Enable
   
//configuracion timer3. reloj de 25MHz
HPS_Timer3_ptr = HPS_Timer3_virtual; //salta cada dos segundos
 
   *(HPS_Timer3_ptr + 2) = 0x00FE;    //deshabilita timer
   *(HPS_Timer3_ptr + 3) = 0X00FE;    // borra flag
   *HPS_Timer3_ptr = 250;		//Load Value, salta cada 10us
   *(HPS_Timer3_ptr + 2) = 0x0003;  //Auto Reload & Enable

   
   
//configuracion timer FPGA Interval. reloj de 100MHz
//FPGA_Timer_Interval_ptr = FPGA_Timer_Interval_Virtual; //salta cada microsegundo
 
 FPGA_Timer_Interval_ptr = LW_virtual + TIMER_BASE;
 
   *(FPGA_Timer_Interval_ptr + 1) = *(FPGA_Timer_Interval_ptr + 1) | 0x8; //parar contador
   *(FPGA_Timer_Interval_ptr + 1) = *(FPGA_Timer_Interval_ptr + 1) | 0x2;  //set auto reload (CONT)
   *(FPGA_Timer_Interval_ptr + 2) = 0x0064;   //valor inicial , low , salta cada microsegundo
   *(FPGA_Timer_Interval_ptr + 3) = 0x0000;	  //valor inicial , high
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