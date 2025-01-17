#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <time.h>

/* Define macros and global variables */
#define NUM_DEV 4 // Number of MAX7219 devices
#define BOARD_WIDTH (NUM_DEV * 8)
#define BOARD_HEIGHT 8

/* Tetris piece definitions */
const uint8_t tetrisPieces[7][4] = {
    {0b1111, 0, 0, 0}, // I piece
    {0b1100, 0b1100, 0, 0}, // Square piece
    {0b0110, 0b1100, 0, 0}, // S piece
    {0b1100, 0b0110, 0, 0}, // Z piece
    {0b1000, 0b1110, 0, 0}, // L piece
    {0b0010, 0b1110, 0, 0}, // J piece
    {0b0100, 0b1110, 0, 0}  // T piece
};

uint8_t bufferCol[NUM_DEV * 8]; // Buffer for column data
uint8_t gameBoard[BOARD_HEIGHT][BOARD_WIDTH] = {0}; // Game board
SPI_HandleTypeDef hspi1; // SPI handle

// Tetris piece state
struct TetrisPiece {
    int type;
    int x;
    int y;
};

struct TetrisPiece currentPiece;

/* Function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
void max_write(int row, uint8_t data);
void flushBuffer(void);
void max7219_cmd(uint8_t Addr, uint8_t data);
void matrixInit(void);
void clearDisplay(void);
void Error_Handler(void);
void spawnNewPiece(void);
void drawPiece(void);
int checkCollision(void);
void lockPiece(void);
void clearFullRows(void);

/* Main function */
int main(void)
{
    /* Initialization code */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();

    /* Initialize random seed */
    srand(HAL_GetTick());

    /* Initialize the matrix display */
    matrixInit();
    clearDisplay();

    /* Spawn first piece */
    spawnNewPiece();

    while (1)
    {
        /* Clear the display buffer */
        for (int i = 0; i < NUM_DEV * 8; i++)
        {
            bufferCol[i] = 0;
        }

        /* Draw current piece */
        drawPiece();

        /* Refresh the display */
        flushBuffer();

        /* Wait for 500 milliseconds */
        HAL_Delay(500);

        /* Move the Tetris piece to the left */
        currentPiece.x--;

        /* Check for collision */
        if (checkCollision())
        {
            /* Move back and lock the piece */
            currentPiece.x++;
            lockPiece();
            clearFullRows();
            spawnNewPiece();
        }
    }
}

/* Spawn a new random Tetris piece */
void spawnNewPiece(void)
{
    currentPiece.type = rand() % 7; // Random piece type
    currentPiece.x = BOARD_WIDTH; // Start off-screen to the right
    currentPiece.y = 3; // Start at top
}

/* Draw the current Tetris piece */
void drawPiece(void)
{
    const uint8_t *piece = tetrisPieces[currentPiece.type];

    for (int row = 0; row < 4; row++)
    {
        if (piece[row] == 0) continue;  // Skip empty rows

        for (int col = 0; col < 4; col++)
        {
            if (piece[row] & (1 << (3 - col)))
            {
                int bufferIndex = (currentPiece.x + col);
                int bufferRow = currentPiece.y + row;

                if (bufferIndex >= 0 && bufferIndex < BOARD_WIDTH &&
                    bufferRow >= 0 && bufferRow < BOARD_HEIGHT)
                {
                    bufferCol[bufferIndex] |= (1 << bufferRow);
                }
            }
        }
    }

    /* Draw the game board */
    for (int y = 0; y < BOARD_HEIGHT; y++)
    {
        for (int x = 0; x < BOARD_WIDTH; x++)
        {
            if (gameBoard[y][x])
            {
                bufferCol[x] |= (1 << y);
            }
        }
    }
}

/* Check for collision with the board or other pieces */
int checkCollision(void)
{
    const uint8_t *piece = tetrisPieces[currentPiece.type];

    for (int row = 0; row < 4; row++)
    {
        if (piece[row] == 0) continue;  // Skip empty rows

        for (int col = 0; col < 4; col++)
        {
            if (piece[row] & (1 << (3 - col)))
            {
                int x = currentPiece.x + col;
                int y = currentPiece.y + row;

                if (x < 0 || (x < BOARD_WIDTH && gameBoard[y][x]))
                {
                    return 1; // Collision detected
                }
            }
        }
    }
    return 0; // No collision
}

/* Lock the current piece into the game board */
void lockPiece(void)
{
    const uint8_t *piece = tetrisPieces[currentPiece.type];

    for (int row = 0; row < 4; row++)
    {
        if (piece[row] == 0) continue;  // Skip empty rows

        for (int col = 0; col < 4; col++)
        {
            if (piece[row] & (1 << (3 - col)))
            {
                int x = currentPiece.x + col;
                int y = currentPiece.y + row;

                if (x >= 0 && x < BOARD_WIDTH && y >= 0 && y < BOARD_HEIGHT)
                {
                    gameBoard[y][x] = 1; // Lock piece
                }
            }
        }
    }
}

/* Clear full rows and shift everything above down */
void clearFullRows(void)
{
    for (int y = 0; y < BOARD_HEIGHT; y++)
    {
        int full = 1;
        for (int x = 0; x < BOARD_WIDTH; x++)
        {
            if (!gameBoard[y][x])
            {
                full = 0;
                break;
            }
        }

        if (full)
        {
            // Clear the row and shift everything above down
            for (int row = y; row > 0; row--)
            {
                for (int x = 0; x < BOARD_WIDTH; x++)
                {
                    gameBoard[row][x] = gameBoard[row - 1][x];
                }
            }

            // Clear the top row
            for (int x = 0; x < BOARD_WIDTH; x++)
            {
                gameBoard[0][x] = 0;
            }
        }
    }
}

/* Function to send data to a specific row of the MAX7219 */
void max_write(int row, uint8_t data)
{
    int devTarget = (row - 1) / 8; // Determine which MAX7219 device
    int offset = devTarget * 8; // Offset for the row within the device
    uint16_t writeData = 0;

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);  // Select the slave
    for (int dev = 0; dev < NUM_DEV; dev++)
    {
        if (dev == devTarget)
        {
            writeData = ((row - offset) << 8) | data;  // Send the row number and data byte
            HAL_SPI_Transmit(&hspi1, (uint8_t *)&writeData, 1, 1000);
        }
        else
        {
            writeData = 0x0000;  // Send NOOP
            HAL_SPI_Transmit(&hspi1, (uint8_t *)&writeData, 1, 1000);
        }
    }
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);  // Deselect the slave
}

/* Function to refresh the display with the buffer content */
void flushBuffer(void)
{
    uint8_t bufferRow[NUM_DEV * 8] = {0}; // Buffer to store data row-wise

    /* Convert columns to rows */
    for (int i = 0; i < NUM_DEV * 8; i++)
    {
        int dev = i / 8; // Device index
        for (int j = 0; j < 8; j++)
        {
            if ((bufferCol[i]) & (1 << j))
            {
                bufferRow[j + (8 * dev)] |= (1 << (7 - (i - (8 * dev))));
            }
        }
    }

    /* Write the row data to the display */
    for (int row = 1; row <= (NUM_DEV * 8); row++)
    {
        max_write(row, bufferRow[row - 1]);
    }
}

/* Function to send a command to all MAX7219 devices */
void max7219_cmd(uint8_t Addr, uint8_t data)
{
    uint16_t writeData = (Addr << 8) | data;
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // Select the slave
    for (int i = 0; i < NUM_DEV; i++)
    {
        HAL_SPI_Transmit(&hspi1, (uint8_t *)&writeData, 1, 100);
    }
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // Deselect the slave
}

/* Function to initialize the MAX7219 devices */
void matrixInit(void)
{
    max7219_cmd(0x09, 0x00); // No decoding
    max7219_cmd(0x0A, 0x01); // Intensity
    max7219_cmd(0x0B, 0x07); // Scan limit (all digits)
    max7219_cmd(0x0C, 0x01); // Shutdown register (normal operation)
    max7219_cmd(0x0F, 0x00); // Display test (off)
}

/* Function to clear the display */
void clearDisplay(void)
{
    for (int i = 0; i < NUM_DEV * 8; i++)
    {
        bufferCol[i] = 0;
    }
    flushBuffer();
}

/* System Clock Configuration function */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* SPI1 Initialization function */
static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* GPIO Initialization function */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin : CS_Pin */
    GPIO_InitStruct.Pin = CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);
}

/* Error Handler function */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        // Stay here in case of error
    }
}
