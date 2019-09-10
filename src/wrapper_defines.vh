/*
* Align the uart clock both in simulation and synthesis with the one used for the board
*/
// Set in the uart ctrl at init stage: clk/(16*115200) = 26@48MHz or 27@50MHz or 54@100MHz
//`define UART_CLK_DIV_DEF  8'd27       // @50MHz
//`define UART_CLK_DIV_DEF  8'd26       // @48MHz
//`define UART_CLK_DIV_DEF  8'd54       // @100MHz
`define UART_CLK_DIV_DEF    8'd123      // @227.273MHz

// Set in the testbench depending on the running frequency of the simulation  
//`define SIM_HALF_CLK_PERIOD_DEF 		10.412        //10.412@48MHz
//`define SIM_HALF_CLK_PERIOD_DEF 		10            // 10@50MHz
//`define SIM_HALF_CLK_PERIOD_DEF 		5             // 5@100MHz
`define SIM_HALF_CLK_PERIOD_DEF 		2.2           // 2.2@227.273MHz

// Set in the testbench depending on the running frequency of the simulation CLK_MHz / (baud_rate) 
//`define SIM_UART_NUM_CLK_TICKS_BIT 	417           // 48MHz/115200 = 417
//`define SIM_UART_NUM_CLK_TICKS_BIT    868           // 100MHz/115200 = 868
`define SIM_UART_NUM_CLK_TICKS_BIT 		1973          // 227.273MHz/115200 = 1973