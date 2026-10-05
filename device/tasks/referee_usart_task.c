#include "referee_usart_task.h"

#include "cmsis_os.h"
#include "referee.h"

/*
 * Generic referee transport state.  The concrete project owns the UART/DMA
 * receive hook; this task only turns received bytes in the FIFO into the
 * protocol-independent referee structures.
 */
uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];
fifo_s_t Referee_FIFO;
uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t Referee_Unpack_OBJ;

void referee_usart_task(void const *argument)
{
    (void)argument;

    (void)fifo_s_init(&Referee_FIFO, Referee_FIFO_Buffer,
                      REFEREE_FIFO_BUF_LENGTH);
    Referee_StructInit();

    for (;;) {
        Referee_UnpackFifoData(&Referee_Unpack_OBJ, &Referee_FIFO);
        osDelay(10);
    }
}
