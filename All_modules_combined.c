#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#define AUDIO_BASE			0xff203040
#define max_sample_size 160000

/* Struct definitions*/

struct MouseInfo{
    char byte1, byte2, byte3;
    int mouse_coords[2];
    int byte_counter;
    bool clicked;
};

struct SamplesLR {
  int Left[max_sample_size];
  int Right[max_sample_size];
  bool input_clear;
  int sample_length;
  int sample_start;
  int sample_end;
};

/* Function prototypes */

void clear_screen();
void plot_pixel(int x, int y, short int line_color);
void draw_box(int x, int y, short int color);
void wait_for_vsync();
void setup_frame_buffer(volatile int* pixel_ctrl_ptr);
void initialize_pixel_ctrl_ptr(volatile int** pixel_ctrl_ptr);
void swap_buffers(volatile int* pixel_ctrl_ptr);

void HEX_PS2(char, char, char);

void setup_interupts();
void interrupt_handler();
void key_ISR();
void PS2_ISR();
void the_exception(void) __attribute__((section(".exceptions")));

bool populate_sample(struct SamplesLR *sample);
void play_sample(struct SamplesLR *sample);
void clear_sample(struct SamplesLR *sample); 
void move_sample(struct SamplesLR *sample, int left_or_right);
void play_all_samples_ordered();
void output_from_sample(struct SamplesLR *sample, int counter, int *left_value, int *right_value); 

/* Globals declarations*/ 

volatile int pixel_buffer_start;  
short int Buffer1[240][512];            // 240 rows, 512 (320 + padding) columns
short int Buffer2[240][512];
int* PS2_BASE = 0xFF200100;
struct MouseInfo mouse_info = {.byte_counter = -2, .clicked = false, .mouse_coords = {170, 120}};
struct SamplesLR sample1 = {.Left = {0}, .Right = {0}, .input_clear = true, .sample_length = 0, .sample_start = 0, .sample_end = 0};
struct SamplesLR sample2 = {.Left = {0}, .Right = {0}, .input_clear = true, .sample_length = 0, .sample_start = 0, .sample_end = 0};
struct SamplesLR sample3 = {.Left = {0}, .Right = {0}, .input_clear = true, .sample_length = 0, .sample_start = 0, .sample_end = 0};

/* Order of setup:
        Create volatile int* pixel_ctrl_ptr
        call initialize_pixel_ctrl_ptr(volatile int* pixel_ctrl_ptr)
        call setup_frame_buffer(volatile int* pixel_ctrl_ptr)
        setup first two frames without erasing, then erase before each draw


        After drawing frame:
                call swap_buffers(volatile int* pixel_ctrl_ptr);

*/

int main() {
  setup_interupts();
  volatile int *pixel_ctrl_ptr = (int*)0xFF203020;
  // initialize_pixel_ctrl_ptr(pixel_ctrl_ptr);

  setup_frame_buffer(pixel_ctrl_ptr);

  int offset = 0;
  int previous[2] = {170, 120};
  int current[2] = {170, 120};

  while (1) {
	  
    if (offset >= 3) {
      plot_pixel(previous[0], previous[1], 0x0000);
	
      previous[0] = mouse_info.mouse_coords[0];
      previous[1] = mouse_info.mouse_coords[1];
    } 
	else 
	{
      offset++;
    }

    plot_pixel(mouse_info.mouse_coords[0], mouse_info.mouse_coords[1], 0xFFFF);
    swap_buffers(pixel_ctrl_ptr);
    
  }
}

/* Functions related to interrupt handlers and ISR */

void setup_interupts() {
  __builtin_wrctl(0, 1);

  // Turns on buttons AND PS2 I THINK!!!
  __builtin_wrctl(3, 130);

  // Turns on interrupt mask register
  volatile int* KEYs = 0xFF200050;
  *(KEYs + 2) = 15;

  // Turn on interrupts for PS2 status register
  volatile int* PS2_CONTROLLER = 0xFF200104;
  *PS2_CONTROLLER = 1;

  return;
}

void interrupt_handler() {
  // Check which interupt, and run its ISR
  int ipending_reg = __builtin_rdctl(4);

  if ((ipending_reg & 0x1) == 1) {
    key_ISR();
  } else if ((ipending_reg & 0x80) == 128) {
    PS2_ISR();
  }
}

void key_ISR() {
  printf("Key interupts");
  volatile int* KEYs = 0xFF200050;
  *(KEYs + 3) = 15;

  return;
}

void PS2_setup() {
  volatile int* PS2_ptr = (int*)PS2_BASE;
  *(PS2_ptr) = 0xFF;
}

void PS2_ISR(){
	volatile int * PS2_ptr = (int *) PS2_BASE;
	int PS2_data = *(PS2_ptr);
	
	mouse_info.byte1 = mouse_info.byte2;
	mouse_info.byte2 = mouse_info.byte3;
	mouse_info.byte3 = PS2_data & 0xFF;
	mouse_info.byte_counter++;
	
	if(mouse_info.byte_counter >= 3){
		mouse_info.byte_counter = 0;
		int dx = mouse_info.byte1;
		int dy = mouse_info.byte3;

		if((mouse_info.mouse_coords[0] + dy) <= 318 && mouse_info.mouse_coords[0] + dy >= 1)
		{
			// printf("dy is : %d", dy);
			mouse_info.mouse_coords[0] += dy;
		}

		if((mouse_info.mouse_coords[1] + dx) <= 238 && mouse_info.mouse_coords[1] + dx >= 1)
		{
			// printf("dx is : %d", dx);
			mouse_info.mouse_coords[1] += dx;
		}		
	}
	
	if ( (mouse_info.byte2 == 0xAA) && (mouse_info.byte3 == 0x00) )
		{
			// mouse inserted; initialize sending of data
			*(PS2_ptr) = 0xF4;
		}
	
	return;
}

void the_exception(void)
/*******************************************************************************
 * Exceptions code. By giving the code a section attribute with the name
 * ".exceptions" we allow the linker program to locate this code at the proper
 * exceptions vector address.
 * This code calls the interrupt handler and later returns from the exception.
 ******************************************************************************/
{
  asm("subi sp, sp, 128");
  asm("stw et, 96(sp)");
  asm("rdctl et, ctl4");
  asm("beq et, r0, SKIP_EA_DEC");  // Interrupt is not external
  asm("subi ea, ea, 4");           /* Must decrement ea by one instruction
                                    * for external interupts, so that the
                                    * interrupted instruction will be run */
  asm("SKIP_EA_DEC:");
  asm("stw r1, 4(sp)");  // Save all registers
  asm("stw r2, 8(sp)");
  asm("stw r3, 12(sp)");
  asm("stw r4, 16(sp)");
  asm("stw r5, 20(sp)");
  asm("stw r6, 24(sp)");
  asm("stw r7, 28(sp)");
  asm("stw r8, 32(sp)");
  asm("stw r9, 36(sp)");
  asm("stw r10, 40(sp)");
  asm("stw r11, 44(sp)");
  asm("stw r12, 48(sp)");
  asm("stw r13, 52(sp)");
  asm("stw r14, 56(sp)");
  asm("stw r15, 60(sp)");
  asm("stw r16, 64(sp)");
  asm("stw r17, 68(sp)");
  asm("stw r18, 72(sp)");
  asm("stw r19, 76(sp)");
  asm("stw r20, 80(sp)");
  asm("stw r21, 84(sp)");
  asm("stw r22, 88(sp)");
  asm("stw r23, 92(sp)");
  asm("stw r25, 100(sp)");  // r25 = bt (skip r24 = et, because it is saved
                            // above)
  asm("stw r26, 104(sp)");  // r26 = gp
  // skip r27 because it is sp, and there is no point in saving this
  asm("stw r28, 112(sp)");  // r28 = fp
  asm("stw r29, 116(sp)");  // r29 = ea
  asm("stw r30, 120(sp)");  // r30 = ba
  asm("stw r31, 124(sp)");  // r31 = ra
  asm("addi fp, sp, 128");
  asm("call interrupt_handler");  // Call the C language interrupt handler
  asm("ldw r1, 4(sp)");           // Restore all registers
  asm("ldw r2, 8(sp)");
  asm("ldw r3, 12(sp)");
  asm("ldw r4, 16(sp)");
  asm("ldw r5, 20(sp)");
  asm("ldw r6, 24(sp)");
  asm("ldw r7, 28(sp)");
  asm("ldw r8, 32(sp)");
  asm("ldw r9, 36(sp)");
  asm("ldw r10, 40(sp)");
  asm("ldw r11, 44(sp)");
  asm("ldw r12, 48(sp)");
  asm("ldw r13, 52(sp)");
  asm("ldw r14, 56(sp)");
  asm("ldw r15, 60(sp)");
  asm("ldw r16, 64(sp)");
  asm("ldw r17, 68(sp)");
  asm("ldw r18, 72(sp)");
  asm("ldw r19, 76(sp)");
  asm("ldw r20, 80(sp)");
  asm("ldw r21, 84(sp)");
  asm("ldw r22, 88(sp)");
  asm("ldw r23, 92(sp)");
  asm("ldw r24, 96(sp)");
  asm("ldw r25, 100(sp)");  // r25 = bt
  asm("ldw r26, 104(sp)");  // r26 = gp
  // skip r27 because it is sp, and we did not save this on the stack
  asm("ldw r28, 112(sp)");  // r28 = fp
  asm("ldw r29, 116(sp)");  // r29 = ea
  asm("ldw r30, 120(sp)");  // r30 = ba
  asm("ldw r31, 124(sp)");  // r31 = ra
  asm("addi sp, sp, 128");
  asm("eret");
}

/* Functions related to VGA */

void swap_buffers(volatile int* pixel_ctrl_ptr) {
  wait_for_vsync();  // swap front and back buffers on VGA vertical sync
  pixel_buffer_start = *(pixel_ctrl_ptr + 1);  // new back buffer
}

void initialize_pixel_ctrl_ptr(volatile int** pixel_ctrl_ptr) {
  *pixel_ctrl_ptr = (int*)0xFF203020;
}

void setup_frame_buffer(volatile int* pixel_ctrl_ptr) {
  // declare other variables(not shown)
  // initialize location and direction of rectangles(not shown)

  /* set front pixel buffer to Buffer 1 */
  *(pixel_ctrl_ptr + 1) = (int)&Buffer1;  // first store the address in the back buffer
  /* now, swap the front/back buffers, to set the front buffer location */
  wait_for_vsync();
  /* initialize a pointer to the pixel buffer, used by drawing functions */
  pixel_buffer_start = *pixel_ctrl_ptr;
  clear_screen();  // pixel_buffer_start points to the pixel buffer

  /* set back pixel buffer to Buffer 2 */
  *(pixel_ctrl_ptr + 1) = (int)&Buffer2;
  pixel_buffer_start = *(pixel_ctrl_ptr + 1);  // we draw on the back buffer
  clear_screen();  // pixel_buffer_start points to the pixel buffer
}

void plot_pixel(int x, int y, short int line_color) {
  volatile short int* one_pixel_address;

  one_pixel_address = pixel_buffer_start + (y << 10) + (x << 1);

  *one_pixel_address = line_color;
}

void clear_screen() {
  int y, x;
  for (x = 0; x < 320; x++)
    for (y = 0; y < 240; y++) plot_pixel(x, y, 0);
}

void draw_box(int x, int y, short int color) {
  for (int i = x - 1; i <= x + 1; i++) {
    for (int j = y - 1; j <= y + 1; j++) {
      plot_pixel(i, j, color);
    }
  }
}

void wait_for_vsync() {
  volatile int* buffer_reg = (int*)0xFF203020;
  *buffer_reg = 1;
  volatile int* status_reg = (int*)0xFF20302C;

  while (1) {
    int status_bit = (*status_reg) & 1;
    if (status_bit == 0) {
      return;
    }
  }
}

/* Audio functions */

void play_sample(struct SamplesLR *sample) {
  volatile int *audioPtr = (int *)AUDIO_BASE;
  int counter = 0;
  
  while (1) {
    int fifoSpace = *(audioPtr + 1);
    fifoSpace = fifoSpace & 0xff000000;

    if (fifoSpace != 0) {
      counter++;

      if (counter >= sample->sample_length ||
          counter >= max_sample_size - 1) {
        break;
      }
		
      *(audioPtr + 2) = sample->Left[counter];
      *(audioPtr + 3) = sample->Right[counter];
    }
  }
}

bool populate_sample(struct SamplesLR *sample) {
  // This can be done before calling
  if (!sample->input_clear) {
    return false;
  }

  sample->input_clear = false;

  volatile int *audioPtr = (int *)AUDIO_BASE;
  int counter = 0;
  
  // Clear input FIFO
  *audioPtr = 0x4;
  while (1) {
    // Check if input FIFO (RALC) != 0
    int fifoSpace = *(audioPtr + 1);
    fifoSpace = fifoSpace & 0x000000ff;

    /*
    if(clicked)
    {
        if (clicked is on button by checking mouse coords)
        {
            break;
        }
    }
    */

    if (fifoSpace != 0) {
      int leftInputFifo, rightInputFifo;

      sample->Left[counter] = *(audioPtr + 2);
      sample->Right[counter] = *(audioPtr + 3);
      counter++;

      if (counter >= max_sample_size - 1) {
        break;
      }
    }
  }
  sample->sample_length = counter - 1;
  sample->sample_end = counter - 1;
}

void clear_sample(struct SamplesLR *sample){
    memset(sample->Left, 0, sizeof sample->Left);
    memset(sample->Right, 0, sizeof sample->Right);
    sample->input_clear = true;
    sample->sample_length = 0;
    sample->sample_start = 0;
    sample->sample_end = 0;
}

void move_sample(struct SamplesLR *sample, int left_or_right){
    int change /* = decide value for change*/;

    if(left_or_right == 2){
        change *= -1;
    }

    // Check if adding values goes out of bound
    if(sample->sample_start + change < max_sample_size && sample->sample_start + change > 0)
    {
        if(sample->sample_end + change < max_sample_size && sample->sample_end + change > 0)
        {
            sample->sample_start += change;
            sample->sample_end += change;
        }
    }
}

void play_all_samples_ordered(){
  volatile int *audioPtr = (int *)AUDIO_BASE;
  int counter = 0;
  
  while (1) {
    int fifoSpace = *(audioPtr + 1);
    fifoSpace = fifoSpace & 0xff000000;
    
    if (fifoSpace != 0) {
      counter++;

      if (counter >= max_sample_size - 1)
      {
        break;
      }

      int left_output = 0;
      int right_output = 0;

      // Check if each sample should be added to total output by checking their start and end locations

      int left_sample_value = 0;
      int right_sample_value = 0;  

      output_from_sample(&sample1, counter, left_sample_value, right_sample_value);
      left_output += left_sample_value;
      right_output += right_sample_value;

      output_from_sample(&sample2, counter, left_sample_value, right_sample_value);
      left_output += left_sample_value;
      right_output += right_sample_value;

      output_from_sample(&sample3, counter, left_sample_value, right_sample_value);
      left_output += left_sample_value;
      right_output += right_sample_value;

      
      *(audioPtr + 2) = left_output;
      *(audioPtr + 3) = right_output;
    }
  }
}

void output_from_sample(struct SamplesLR *sample, int counter, int *left_value, int *right_value){
    
    // Counter is within range of sample output, calculate its offset and change left and right values to the samples L and R values at that counter
    if(counter > sample->sample_start && counter < sample->sample_end)
    {
        *left_value = sample->Left[counter - sample->sample_start];
        *right_value = sample->Right[counter - sample->sample_start];
        return;
    }
    else
    {
        *left_value = 0;
        *right_value = 0;
        return;
    }
}