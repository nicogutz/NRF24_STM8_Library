# NRF24 Library Usage Guide
Check mirf.c for more details.

## Prerequisites

Before using this library, ensure that you have the following:

- STM8 microcontroller development board.
- PlatformIO with STLink driver support
- NRF24 module connected to the STM8 board with appropriate wiring.

## Configuration

### Pins Configuration

Make sure to define the pins connected to the NRF24 module in the library using the following macros:

```c
#define CE_PIN_LETTER GPIOC
#define CE_PIN_NUMBER GPIO_PIN_4

#define CSN_PIN_LETTER GPIOC
#define CSN_PIN_NUMBER GPIO_PIN_3
```

Adjust these macros based on your actual hardware configuration.

### NRF Configuration

Set the NRF channel number and payload size using the following macros:

```c
#define NRF_CHANNEL_NR 114
#define NRF_PAYLOAD_SIZE 32
```

## Initialization

To initialize the NRF24 module, use the `Nrf24_init()` function. This function configures the CE/CS pins and SPI.

```c
Nrf24_init();
```

## Sending Data

To send data, use the `Nrf24_send` function. Provide the data array and a pointer to a boolean indicating the RX/TX state of the NRF module.

```c
uint8_t data[NRF_PAYLOAD_SIZE] = { /* your data here */ };
bool PTX = /* true if in TX mode, false if in RX mode */;
Nrf24_send(data, &PTX);- STM8 libraries installed.
```

- `Nrf24_setTADDR`: Set the transmitting device address.
- `Nrf24_addRADDR`: Add more device addresses to receive.
- `Nrf24_SetOutputRF_PWR`: Set the transmission power.
- `Nrf24_SetSpeedDataRates`: Select between high-speed data rates.
- `Nrf24_setRetransmitDelay`: Set the auto retransmit delay.

Refer to the function descriptions in the code for more details on each configuration function.

## Power Management

Control the power state of the NRF24 module using the following functions:

- `Nrf24_powerUpRx`: Power up the NRF module for receiving.
- `Nrf24_powerUpTx`: Power up the NRF module for transmitting.
- `Nrf24_powerDown`: Power down the NRF module.

## Status and Checking

Use the following functions to check the status and monitor the communication:

- `Nrf24_getStatus`: Get the status register value.
- `Nrf24_isSending`: Check if the module is still sending.
- `Nrf24_isSend`: Check if sending has finished or retries are over.

## Usage Example

The main.cpp file comes with an example on how to set up the NRF.

### 1. GPIO Configuration

First, configure the necessary GPIO pins for debugging LEDs and the button. Additionally, set up an interrupt for the button.

```c
// Debuging LEDs
GPIO_Init(GPIOB, GPIO_PIN_1, GPIO_MODE_OUT_OD_LOW_FAST);
GPIO_Init(GPIOB, GPIO_PIN_2, GPIO_MODE_OUT_OD_LOW_FAST);

// Button
GPIO_Init(GPIOB, GPIO_PIN_0, GPIO_MODE_IN_PU_IT);

// Interrupt for button
EXTI_DeInit();
EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_FALL_ONLY);
EXTI_SetTLISensitivity(EXTI_TLISENSITIVITY_FALL_ONLY);

enableInterrupts();

// Turn off LEDs
GPIO_WriteHigh(GPIOB, GPIO_PIN_1);
GPIO_WriteHigh(GPIOB, GPIO_PIN_2);
```

### 2. NRF24 Initialization

Initialize the NRF24 module and configure it based on your application requirements.

```c
Nrf24_init();
bool PTX = 0;

Nrf24_config(&PTX);
```

### 3. Set Own and Receiver Addresses

Set the own address and the receiver address using 5 characters. Ensure the addresses are unique and match between the transmitter and receiver.

```c
int ret;

ret = Nrf24_setRADDR((uint8_t *)"ABCDE");
if (ret != SUCCESS)
{
    while (1)
    {
        delay_ms(1);
    }
}

ret = Nrf24_setTADDR((uint8_t *)"FGHIJ");
if (ret != SUCCESS)
{
    while (1)
    {
        delay_ms(1);
    }
}
```

### 4. Configure Communication Parameters

Set the communication parameters such as data rate and retransmit delay.

```c
Nrf24_SetSpeedDataRates(RF24_1MBPS);
Nrf24_setRetransmitDelay(0);
```

### 5. Main Loop

Implement the main loop to handle communication and LED control based on button presses.

```c
uint8_t buf[32];

// Clear RX FiFo
while (1)
{
    if (Nrf24_dataReady() == FALSE)
    {
        break;
    }
    Nrf24_getData(buf);
}

while (1)
{
    // When the program is received, the received data is output from the serial port
    if (Nrf24_dataReady())
    {
        // Turns the led on when a packet is received if the button was clicked
        if (bIntFlag == 1)
        {
            bIntFlag = 0;
            GPIO_WriteLow(GPIOB, GPIO_PIN_1);
        }
        else
        {
            GPIO_WriteHigh(GPIOB, GPIO_PIN_1);
        }

        Nrf24_getData(buf);

        // The ESP is printing stuff so it takes a while to switch to RX mode.
        delay_ms(10);

        Nrf24_send(buf, &PTX);

        // Same here, delay a bit.
        delay_ms(10);

        while (!Nrf24_isSend(50, &PTX))
        {
            // If no ACK after the preconfigured retries, send again.
            Nrf24_send(buf, &PTX);
            delay_ms(10);
        }
    }
    // Wait a bit to check if the NRF received data.
    delay_ms(1);
}
```

Adjust the code as needed for your specific application logic and use case. Ensure that the button press detection and LED control match your hardware setup.