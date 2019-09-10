`include "wrapper_defines.vh"

module fpga_top
#(	parameter CNT_NBIT		= 24,
	parameter UART_AW		= 3,
	parameter UART_DW		= 8,
	parameter CNT_BIT		= 16,
	parameter RECV_NUM_BYTE	= 16,
    //key width for the aes core (bits)
	parameter AES_KW 		= 256,
	//plain/encrypted width for the aes core (bits)
	parameter AES_DW 		= 128)
(
	input CLK_PIN,
	input RST_PIN,

	input RX_PIN,
	output TX_PIN,
	output [9:0] M_LED,
	output TRIGGER_PIN,
	output UART_INT_PIN
);

wire clk;


  // ------------------------------------------------------------------------------
  // Clock input driver
  // ------------------------------------------------------------------------------
  IBUFG clkdrv (.I( CLK_PIN ), .O( clk ));   // 48MHz input

	reg [CNT_NBIT-1:0] cnt;

	always@(posedge clk, posedge RST_PIN)
	begin
		if(~RST_PIN)
			cnt<=0;
		else
			cnt<=cnt+1;

	end

	//assign M_LED[3:0] = 4'hf;
	assign M_LED[9:8]	= cnt[CNT_NBIT-1:CNT_NBIT-2];


// wires for the UART - SAKURAGHOST
wire [UART_AW-1:0]	wb_adr_host_uart;
wire [UART_DW-1:0]	wb_dat_host_uart;
wire				wb_cyc_host_uart;
wire				wb_stb_host_uart;
wire [3:0]			wb_sel_host_uart;
wire 				wb_we_host_uart;
wire [UART_DW-1:0]	wb_dat_uart_host;
wire				wb_ack_uart_host;

// wires for the AESES core
wire [0:AES_KW-1]	host_aes_key;
wire [0:AES_DW-1]	host_aes_block;
wire [0:AES_DW-1]	aes_host_result;

wire				aes_host_result_valid;
wire				aes_host_ready;

wire                host_aes_schedule;
wire                host_aes_enable;

/*	AES core. Config: 128 plain 128 key, ECB */

	aeses_lite aes0(
	       // clk and rst
	       .clk(clk),
	       .rst(~RST_PIN),
	       // data inputs
	       .aes_key_i(host_aes_key),
	       .aes_blk_i(host_aes_block),
	       .aes_blk_o(aes_host_result),
	       // control outputs
	       .valid_o(aes_host_result_valid),
	       .ready_o(aes_host_ready),
	       // control inputs
	       .enable_key_schedule_i(host_aes_schedule),
	       .enable_op_i(host_aes_enable)
	       //.key_mode_i(host_aes_key_mode),
	       //.enc_decneg_i(host_aes_enc_decneg)
	);



/*	UART controller for the communication with the host-PC. Compliant with the
 *	uart16550*/
	uart_top uart0(
		  // system clock and reset PINS from the FPGA
		  .wb_clk_i(clk),
		  .wb_rst_i(~RST_PIN),
		  // Wishbone signals
		  .wb_adr_i(wb_adr_host_uart),
		  .wb_dat_i(wb_dat_host_uart),
		  .wb_dat_o(wb_dat_uart_host),
		  .wb_we_i(wb_we_host_uart),
		  .wb_stb_i(wb_stb_host_uart),
		  .wb_cyc_i(wb_cyc_host_uart),
		  .wb_ack_o(wb_ack_uart_host),
		  .wb_sel_i(wb_sel_host_uart),
		  .int_o(UART_INT_PIN), // interrupt request
		  // UART RX and TX PINS from the FPGA
		  .stx_pad_o(TX_PIN),
		  .srx_pad_i(RX_PIN),

		// modem signals
		  //.rts_pad_o(rts_o),
		  .cts_pad_i(1'b1),
		  //.dtr_pad_o(),
		  .dsr_pad_i(1'b1),
		  .ri_pad_i(1'b1),
		  .dcd_pad_i(1'b1)

	  );

/* This is the main host controller for the evaluated system connected to the
*  UART and the DUT (e.g., simon, AES and so on). It gets data from the serial
*  port, prepare them for the DUT that computes and return the result that is
*  propagated back through the UART to the PC for validation

*/
	host_ctrl	#(
			.UART_AW(UART_AW),
			.UART_DW(UART_DW),
			.CNT_BIT(CNT_BIT))

			host0(
				.wb_clk(clk),
				.wb_rst(~RST_PIN),
			   	// WB iface	to uart16550
				.wb_adr_uart_o(wb_adr_host_uart),
				.wb_dat_uart_o(wb_dat_host_uart),
				.wb_cyc_uart_o(wb_cyc_host_uart),
				.wb_stb_uart_o(wb_stb_host_uart),
				.wb_sel_uart_o(wb_sel_host_uart),
				.wb_we_uart_o(wb_we_host_uart),
				.wb_dat_uart_i(wb_dat_uart_host),
				.wb_ack_uart_i(wb_ack_uart_host),
				// hardware led
				.recv_byte_led_o(M_LED[7:0]),
				// iface to aes_core
				.host_aes_key_o(host_aes_key),
                .host_aes_blk_o(host_aes_block),
                .host_aes_enable_key_schedule_o(host_aes_schedule),
                .host_aes_enable_op_o(host_aes_enable),
				.aes_host_result_i(aes_host_result),
				.aes_host_result_valid_i(aes_host_result_valid),
				.aes_host_ready_i(aes_host_ready),
				// trigger pin for the oscilloscope
				.trigger_o(TRIGGER_PIN)
			);

endmodule