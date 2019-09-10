`include "./uart/uart_defines.v"
`include "wrapper_defines.vh"
`include "aeses_defines.vh"

module host_ctrl(
					wb_clk,wb_rst,
					//interface uart
					wb_adr_uart_o,
					wb_dat_uart_o,
					wb_cyc_uart_o,
					wb_stb_uart_o,
					wb_sel_uart_o,
					wb_we_uart_o,
					wb_dat_uart_i,
					wb_ack_uart_i,
					// interface aes core
					host_aes_key_o,
					host_aes_blk_o,
					host_aes_enable_key_schedule_o,
					host_aes_enable_op_o,
					aes_host_result_i,
					aes_host_result_valid_i,
					aes_host_ready_i,
					//signals to the FPGA primary outputs
					trigger_o,			// trigger for the oscilloscope
					recv_byte_led_o 	// FPGA led check fsm ss
			);

parameter UART_AW	=	3;
parameter UART_DW	=	8;
parameter CNT_BIT	=	16;
parameter NUM_BYTE_BLK	=	16;
parameter NUM_BYTE_KEY  =   32;

parameter AES_KW = 256;	//key width for the aes core (bits)
parameter AES_DW = 128;	//plain/encrypted width for the aes core (bits)

parameter TRIGGER_BIT_W 		= 18; 		// number of bits for the trigger counter watchdog
parameter TRIGGER_MAX			= 100000; 	// spinlock wait for number of cycles
parameter AES_PRE_POST_DELAY 	= 20; 		// isolate aes computation from the trigger set and clear operations

parameter NUM_CNT_REUSE_TIMES 	= 0;	// number of times we use the encrypted output as input of the subsequent computation
parameter NUM_CNT_REUSE_BIT 	= 16;	// number of bits for the counter of such reencoding

//////////////////////////////////////
// wb clock and reset signals		//
//////////////////////////////////////
input wb_clk;
input wb_rst;

////////////////////////////////////////
// WISHBONE IFACE to the UART16550	////
////////////////////////////////////////
output	reg [UART_AW-1:0]	wb_adr_uart_o;
output	reg [UART_DW-1:0]	wb_dat_uart_o;
output 	reg					wb_cyc_uart_o;
output 	reg					wb_stb_uart_o;
output	reg [3:0]			wb_sel_uart_o;
output	reg 				wb_we_uart_o;

input	[UART_DW-1:0]	wb_dat_uart_i;
input 					wb_ack_uart_i;
////////////////////////////////////////
// IFACE to the AES		 			////
////////////////////////////////////////
output      [0:AES_KW-1]    host_aes_key_o;
output      [0:AES_DW-1]    host_aes_blk_o;
output  reg                 host_aes_enable_key_schedule_o;
output  reg                 host_aes_enable_op_o;

input		[0:AES_DW-1]	aes_host_result_i;
input	 					aes_host_result_valid_i;
input 						aes_host_ready_i;

///////////////////////////////////////
output [7:0]	recv_byte_led_o;
output reg		trigger_o;


function integer clog2(input integer value);
begin
		if(value==0)
			clog2=0;
		else
		begin
			while(value>0)
			begin
				clog2=clog2+1;
				value=value>>1;
			end
		end
end
endfunction

// this controller implements the loopback of the uart data
localparam [4:0]	CONFIGURE					= 5'd0,
					WE_UART_DIV_REG				= 5'd1,
					SET_UART_DIV_REG			= 5'd2,
					END_CONFIGURE				= 5'd3,

					CHECK_UART_RXFIFO			= 5'd6,
					READ_BYTE_UART         		= 5'd7,
					
					CHECK_UART_TXFIFO			= 5'd8,
					WRITE_BYTE_UART    			= 5'd9,
// compute aes
					AES_WAIT_READY_INIT			= 5'd18,
					AES_INIT_KEY_1				= 5'd19,
					AES_INIT_KEY_2				= 5'd20,

					AES_WAIT_READY_COMP			= 5'd21,
					AES_START_COMPUTATION		= 5'd22,
					AES_WAIT_COMP_DONE			= 5'd23,
// delay to allow trigger reset
					TRIGGER_DELAY				= 5'd24,	// wait before set the trigger
// delay to correctly align trigger before and after computation
					TRIGGER_DELAY_PRE_COMP		= 5'd25,	// set the trigger SOMETIME before starting the actual computation
					TRIGGER_DELAY_POST_COMP		= 5'd26;	// clear the trigger SOMETIME after the end of the actual computation

// count the 8bit words to receive before moving to the next state

reg [4:0] ss, ss_next;
reg [4:0] ss_after_read, ss_after_read_next;

reg trigger_o_next;
reg [TRIGGER_BIT_W-1:0] trigger_cnt, trigger_cnt_next; // keep delay before set the trigger again

reg trigger_cnt_inc;
reg trigger_cnt_rst;
reg trigger_cnt_we;

// delay before and after the aes to allow proper trigger setup time
reg [TRIGGER_BIT_W-1:0] cnt_aes_delay, cnt_aes_delay_next;

reg cnt_aes_delay_inc;
reg cnt_aes_delay_rst;
reg cnt_aes_delay_we;

//////////////////////////////////////////////////
// registers to store key and plain to feed  	//
// the DUT, e.g. AES module						//
//////////////////////////////////////////////////
reg [UART_DW-1:0] key_r 	[0:NUM_BYTE_KEY-1];
reg key_r_we_next;
wire [0:255] key_wire; // bitvector wire


reg [UART_DW-1:0] plain_r 	[0:NUM_BYTE_BLK-1];
reg plain_r_we_next;
reg plainFlush_r_we_next;
wire [0:127] plain_wire; // bitvector wire

reg [0:127] enc_r, enc_r_next; 			// store the aes_core output at once, i.e., 128bit

reg [UART_DW-1:0] 		data, data_next;

localparam CTR_WIDTH = $clog2(NUM_BYTE_KEY);
//count the next byte to set in the key and plain bitvectors from the uart byte-wise iface
reg [CTR_WIDTH:0] cnt_recvbyte, cnt_recvbyte_next, cnt_recvbyte_rst_value;
reg	cnt_recvbyte_dec;
reg	cnt_recvbyte_rst;
reg	cnt_recvbyte_we;

// update signals for the wb interface to uart
reg [UART_AW-1:0]	wb_adr_uart_o_next;
reg [UART_DW-1:0]	wb_dat_uart_o_next;
reg					wb_cyc_uart_o_next;
reg					wb_stb_uart_o_next;
reg [3:0]			wb_sel_uart_o_next;
reg 				wb_we_uart_o_next;

// update signals for the aes_core iface
reg         host_aes_enable_key_schedule_next;
reg         host_aes_enable_op_next;

//////////////////////////////////////////////////
//	HELPER TASKS TO ACCESS THE UART				//
//  set_uart_op: 								//
//		performs a Read or write op to a 		//
//		specific address of the uart			//
//////////////////////////////////////////////////
localparam 	READ_OP		= 1'b0;
localparam	WRITE_OP	= 1'b1;

task set_uart_op;
input rw;							// 0 is read 1 is write
input [UART_AW-1:0] addr; 			// address to perform the op
input [3:0] sel;					// select bytes in the 32bit word
input [UART_DW-1:0] wdata;			// data to write eventually
begin
	wb_cyc_uart_o_next	= 1;		// WB cyc
	wb_stb_uart_o_next	= 1;		// WB stb
	wb_adr_uart_o_next	= addr;		// WB addr
	wb_sel_uart_o_next	= sel;		// WB sel
	wb_we_uart_o_next	= rw; 		// r/w
	wb_dat_uart_o_next	= wdata;	// drive the dat_i to uart when write
end
endtask

//////////////////////////////////////////////////

assign host_aes_blk_o 	= {
                        plain_r[15],	plain_r[14], 	plain_r[13], 	plain_r[12],
						plain_r[11],	plain_r[10],	plain_r[9],		plain_r[8],
						plain_r[7],		plain_r[6],		plain_r[5],		plain_r[4],
						plain_r[3],		plain_r[2],		plain_r[1], 	plain_r[0] };

assign host_aes_key_o 	= {
                        key_r[31],		key_r[30],		key_r[29],		key_r[28],
                        key_r[27],		key_r[26],		key_r[25],		key_r[24],
                        key_r[23],		key_r[22],		key_r[21],		key_r[20],
                        key_r[19],		key_r[18],		key_r[17],		key_r[16],
                        key_r[15],		key_r[14],		key_r[13],		key_r[12],
						key_r[11],		key_r[10],		key_r[9],		key_r[8],
						key_r[7],		key_r[6],		key_r[5],		key_r[4],
						key_r[3],		key_r[2],		key_r[1], 		key_r[0] };

assign recv_byte_led_o[4:0] = ss; // set ss to led status
assign recv_byte_led_o[5] = aes_host_ready_i;
assign recv_byte_led_o[6] = host_aes_enable_key_schedule_o | host_aes_enable_op_o;
assign recv_byte_led_o[7] = aes_host_result_valid_i;

//////////////////////////////////////////////////////////
// SEQUENTIAL LOGIC										//
//////////////////////////////////////////////////////////

always@(posedge wb_clk, posedge wb_rst)
begin:seq_host
	integer i;
	if(wb_rst)
	begin
		ss 				<=	CONFIGURE;
		ss_after_read   <=  AES_WAIT_READY_INIT;
		trigger_o 		<=	0;
		trigger_cnt		<=	0;
		cnt_aes_delay	<=	0;
		// wb signals to uart
		wb_adr_uart_o	<=	{UART_AW{1'b0}};
		wb_cyc_uart_o	<=	0;
		wb_stb_uart_o	<=	0;
		wb_sel_uart_o	<=	0;
		wb_dat_uart_o	<=	0;
		wb_we_uart_o	<=	0;
		data			<=	0; // last data received from the uart

		for(i=0;i<NUM_BYTE_KEY;i=i+1)
			key_r[i]	<=	0;

		for(i=0;i<NUM_BYTE_BLK;i=i+1)
			plain_r[i]	<=	0;

		cnt_recvbyte	<=	0;
		enc_r			<=	0;

		// AES_CORE signals
        host_aes_enable_key_schedule_o  <= 1'b0;
        host_aes_enable_op_o            <= 1'b0;
	end
	else
	begin
		ss				<=	ss_next;
		trigger_o 		<=	trigger_o_next;

		if(trigger_cnt_we)
			trigger_cnt		<=	trigger_cnt_next;

		if(cnt_aes_delay_we)
			cnt_aes_delay	<=	cnt_aes_delay_next;

	//	uart signals
		wb_adr_uart_o	<= wb_adr_uart_o_next;
		wb_cyc_uart_o	<= wb_cyc_uart_o_next;
		wb_stb_uart_o	<= wb_stb_uart_o_next;
		wb_sel_uart_o	<= wb_sel_uart_o_next;
		wb_dat_uart_o	<= wb_dat_uart_o_next;
		wb_we_uart_o	<= wb_we_uart_o_next;
		data			<= data_next; // last data received from the uart

	// internal variables to feed the aes dut
		if(cnt_recvbyte_we)
			cnt_recvbyte		<=	cnt_recvbyte_next;

		if(key_r_we_next)
			key_r[cnt_recvbyte]	<=	data_next;

		if(plain_r_we_next)
		begin
			plain_r[cnt_recvbyte]	<=	data_next;
		end
		enc_r		<= enc_r_next;

	//keep the AES_CORE signals
        host_aes_enable_key_schedule_o  <= host_aes_enable_key_schedule_next;
        host_aes_enable_op_o            <= host_aes_enable_op_next;
	
	// my internal updates...
	    ss_after_read       <=  ss_after_read_next;
	end
end


//////////////////////////////////////////////////////////
// COMBINATORIAL LOGIC									//
// to update all theh signals of the host_controller	//
//////////////////////////////////////////////////////////
always@(*)
begin
	// internal state variables
	ss_next 			= ss;
	trigger_o_next		= trigger_o;

	// keep the wb interface signals
	wb_adr_uart_o_next	= wb_adr_uart_o;
	wb_cyc_uart_o_next	= wb_cyc_uart_o;
	wb_stb_uart_o_next	= wb_stb_uart_o;
	wb_sel_uart_o_next	= wb_sel_uart_o;
	wb_we_uart_o_next	= wb_we_uart_o;
	wb_dat_uart_o_next	= wb_dat_uart_o;
	data_next			= data;

	// key plain and recv_byte_cnt update
	plainFlush_r_we_next= 0;
	enc_r_next			= enc_r;

	//keep the aes_core signals
    host_aes_enable_key_schedule_next = 1'b0;
    host_aes_enable_op_next = 1'b0;

	trigger_cnt_inc = 0;
    trigger_cnt_rst = 0;

	cnt_recvbyte_dec = 0;
	cnt_recvbyte_rst = 0;

	cnt_aes_delay_inc=0;
	cnt_aes_delay_rst=0;
	
	// manage next state in a single UART read scenario (instead of having OP1 and OP2)
	ss_after_read_next = ss_after_read;
	cnt_recvbyte_rst_value = 0;

	case(ss)
///// start uart config ////
		CONFIGURE:
			begin
			if(wb_rst==0)
				ss_next = WE_UART_DIV_REG;
			else
				ss_next = CONFIGURE;
			end
		WE_UART_DIV_REG:
			begin
				set_uart_op(WRITE_OP,`UART_REG_LC,4'b1,8'b10000011);
				if(wb_ack_uart_i)
				begin
					ss_next = SET_UART_DIV_REG;
					wb_cyc_uart_o_next	= 0;
					wb_stb_uart_o_next	= 0;
					wb_we_uart_o_next	= 0;
				end
				else //keep waiting
					ss_next = WE_UART_DIV_REG;
			end
		SET_UART_DIV_REG:
			begin
				set_uart_op(WRITE_OP,`UART_REG_DL1,4'd1,`UART_CLK_DIV_DEF);     // clk/(16*115200) = 26@48MHz or 27@50MHz or 54@100MHz
				if(wb_ack_uart_i)
				begin
					ss_next = END_CONFIGURE;
					wb_cyc_uart_o_next	= 0;
					wb_stb_uart_o_next	= 0;
					wb_we_uart_o_next	= 0;
					end
				else //keep waiting
					ss_next = SET_UART_DIV_REG;
			end
		END_CONFIGURE:
			begin
				set_uart_op(WRITE_OP,`UART_REG_LC,4'b1,8'b00000011);
				if(wb_ack_uart_i)
				begin
					ss_next = CHECK_UART_RXFIFO;
					ss_after_read_next = AES_WAIT_READY_INIT;
					cnt_recvbyte_rst = 1;
					cnt_recvbyte_rst_value = 31;

					wb_cyc_uart_o_next	= 0;
					wb_stb_uart_o_next	= 0;
					wb_we_uart_o_next	= 0;
					end
				else //keep waiting
					ss_next = END_CONFIGURE;
			end
			
// GET_AESES_CONTROL_BYTE -- this is to get the control byte that is needed to drive the operation of the AESES core
// It basically mimicks the behaviour of the RESET_OP2 and RESET_OP1 states (in the original files...)
        CHECK_UART_RXFIFO:
            begin
                set_uart_op(READ_OP,`UART_REG_LS,1,0);
                if(wb_ack_uart_i && wb_dat_uart_i[`UART_LS_DR]==1) //lineStatus_reg.data_ready
                begin
                    ss_next = READ_BYTE_UART;

                    wb_cyc_uart_o_next    = 0;
                    wb_stb_uart_o_next    = 0;
                    wb_we_uart_o_next    = 0;
                    $strobe("@%0t: Host0 - UART_FIFO_RX_BUF_HAS_BYTE",$time);
                end
            end
        READ_BYTE_UART:
            begin
                set_uart_op(READ_OP,`UART_REG_RB,1,0);
                if(wb_ack_uart_i)
                begin
                    if(cnt_recvbyte==0)
                    begin
                        // if I ended reading, then I go in the after-read state!
                        ss_next = ss_after_read;
                        cnt_recvbyte_rst = 1;
                        cnt_recvbyte_rst_value = 15;
                    end
                    else
                    begin
                        ss_next = CHECK_UART_RXFIFO;
                        cnt_recvbyte_dec = 1;
                    end
    
                    // get data and close WB transaction to uart
                    wb_cyc_uart_o_next    = 0;
                    wb_stb_uart_o_next    = 0;
                    wb_we_uart_o_next    = 0;
                    data_next = wb_dat_uart_i;
                    $strobe("@%0t: Host0 - END_READ_UART",$time);
                end
                else
                begin
                    ss_next = READ_BYTE_UART;
                    $strobe("@%0t: Host0 - TRY_READ_UART",$time);
                end
            end

/// check uart txfifo and then send a byte ////
		CHECK_UART_TXFIFO:
			begin
				set_uart_op(READ_OP,`UART_REG_LS,1,0);
				if(wb_ack_uart_i && wb_dat_uart_i[`UART_LS_TE]==1)
				begin
					ss_next = WRITE_BYTE_UART;

					wb_cyc_uart_o_next	= 0;
					wb_stb_uart_o_next	= 0;
					wb_we_uart_o_next	= 0;
					
					$strobe("@%0t: Host0 - UART_FIFO_TX_BUF_EMPTY",$time);
				end
			end
		WRITE_BYTE_UART:
			begin
				set_uart_op(WRITE_OP,`UART_REG_TR,1, enc_r[ (UART_DW * (15-cnt_recvbyte)) +: 8] );
				if(wb_ack_uart_i)
				begin
					if(cnt_recvbyte==0)
					begin
						ss_next = CHECK_UART_RXFIFO;
						cnt_recvbyte_rst = 1;
						cnt_recvbyte_rst_value = 15;
					end
					else
					begin
						ss_next = CHECK_UART_TXFIFO;
						cnt_recvbyte_dec = 1;
					end

					// close WB transaction to uart
					wb_cyc_uart_o_next	= 0;
					wb_stb_uart_o_next	= 0;
					wb_we_uart_o_next	= 0;
					$strobe("@%0t: Host0 - END_WRITE_UART",$time);
				end
				else
				begin
					ss_next = WRITE_BYTE_UART;
					$strobe("@%0t: Host0 - TRY_WRITE_UART",$time);
				end
			end

///// aes_core: KEYMEM init ////
		AES_WAIT_READY_INIT: // wait aes to be ready again. it exits reset with ready=1
			begin
				if(aes_host_ready_i)
				begin
					ss_next = AES_INIT_KEY_1;
					// TODO: check correctness
					// start keyschedule when the aeses_core is ready
					host_aes_enable_key_schedule_next = 1'b1;
					$strobe("@%0t: Host0 - AES_CORE is ready",$time);
				end
			end
		AES_INIT_KEY_1: // set key and block and wait for keyschedule
			begin
				if(~aes_host_ready_i) // wait for ready to be LOW
				begin
					ss_next = AES_INIT_KEY_2;
					$strobe("@%0t: Host0 - key schedule STARTED",$time);
				end
			end
		AES_INIT_KEY_2: // set key and block and wait for keyschedule
			begin
				if(aes_host_ready_i)
				begin
					ss_next = CHECK_UART_RXFIFO;
					ss_after_read_next = AES_WAIT_READY_COMP;
					$strobe("@%0t: Host0 - key scheduled DONE, waiting for blocks to encrypt/decrypt",$time);
				end
			end

///// aes_core: COMPUTATION (ENC/DEC) ////
		AES_WAIT_READY_COMP: // eventually wait aes core is ready for new computation
				begin
				trigger_o_next	=	1;
				if(aes_host_ready_i)
				begin
					ss_next = TRIGGER_DELAY_PRE_COMP;
					$strobe("@%0t: Host0 - AES_CORE is ready",$time);
				end
				end
		TRIGGER_DELAY_PRE_COMP: //avoid ringing effects: delay aes computation from trigger set
			begin

				if(cnt_aes_delay == AES_PRE_POST_DELAY)
				begin
					cnt_aes_delay_rst = 1'b1;
					ss_next = AES_START_COMPUTATION; // start the computation
					// TODO: check correctness!
					host_aes_enable_op_next = 1'b1;
				end
				else
				begin
					cnt_aes_delay_inc=1'b1;
				end
			end
		AES_START_COMPUTATION:	// run the encryption: set host_aes_next_o = 1 and wait for result
			begin
				//set_aes_op(AES_ENC_DEC_OP,IS_AES_ENC,plain_wire,key_wire);
				ss_next = AES_START_COMPUTATION;
				if(~aes_host_ready_i)// wait for ready to be LOW
				begin
					ss_next = AES_WAIT_COMP_DONE;
					$strobe("@%0t: Host0 - AES computation STARTED",$time);
				end
			end
		AES_WAIT_COMP_DONE:	// run the encryption: set host_aes_next_o = 1 and wait for result
			begin
				ss_next = AES_WAIT_COMP_DONE;
				//if(aes_host_ready_i) //FIXME possible to have ready signal without validity because of an error?
				if(aes_host_result_valid_i)
				begin
					enc_r_next = aes_host_result_i;
					ss_next = TRIGGER_DELAY_POST_COMP;
					//$strobe("@%0t: Host0 - AES computation input: %h",$time,host_aes_blk_o);
					$strobe("@%0t: Host0 - AES computation DONE with output: %h",$time,aes_host_result_i);
				end
			end
		TRIGGER_DELAY_POST_COMP: //wait for the oscilloscope before starting the new computation
			begin
				if(cnt_aes_delay == AES_PRE_POST_DELAY)
				begin
    				trigger_o_next	=	0;
					cnt_aes_delay_rst = 1'b1;
					ss_next = TRIGGER_DELAY; // start the computation
				end
				else
				begin
					cnt_aes_delay_inc = 1'b1;
				end
			end

// OSCILLOSCOPE Delay: before resetting the trigger
		TRIGGER_DELAY: //wait for the oscilloscope before starting the new computation
			begin
				if(trigger_cnt == TRIGGER_MAX)
				begin
					trigger_cnt_rst = 1'b1;
					ss_next = CHECK_UART_TXFIFO; // restart the computation
				end
				else
				begin
					trigger_cnt_inc=1'b1;
				end
			end
	endcase
end


// counter for trigger delay
always@(*)
begin

		cnt_aes_delay_next	= 16'b0;
		cnt_aes_delay_we		= 1'b0;

		if(cnt_aes_delay_inc)
		begin
			cnt_aes_delay_next	=	cnt_aes_delay + 1'b1;
			cnt_aes_delay_we 	=	1'b1;
		end
		else if(cnt_aes_delay_rst)
		begin
			cnt_aes_delay_next	= 0;
			cnt_aes_delay_we 	= 1'b1;
		end
end


// counter for trigger delay
always@(*)
begin
		trigger_cnt_next	= 16'b0;
		trigger_cnt_we		= 1'b0;

		if(trigger_cnt_inc)
		begin
			trigger_cnt_next	=	trigger_cnt + 1'b1;
			trigger_cnt_we 		=	1'b1;
		end
		else if(trigger_cnt_rst)
		begin
			trigger_cnt_next	= 0;
			trigger_cnt_we 		= 1'b1;
		end
end


// counter for rx/tx bytes from/to uart for plain, key and enc
always@(*)
begin
		cnt_recvbyte_next	= 'b0;
		cnt_recvbyte_we		= 1'b0;

		if(cnt_recvbyte_dec)
		begin
			cnt_recvbyte_next	=	cnt_recvbyte - 1'b1;
			cnt_recvbyte_we 	=	1'b1;
		end
		else if(cnt_recvbyte_rst)
		begin
		    // set the proper reset value according to what you need to read next from UART
		    // CONTROL_BYTE = 1
		    // AES_BLOCK = 15
		    // AES_KEY = 31
			cnt_recvbyte_next	= cnt_recvbyte_rst_value;
			cnt_recvbyte_we 	= 1'b1;
		end
end


// fix all the write enables that were scrambled by my different state machine
always@(*)
begin
    key_r_we_next = wb_ack_uart_i && ss_after_read == AES_WAIT_READY_INIT;
    plain_r_we_next = plainFlush_r_we_next || (wb_ack_uart_i && ss_after_read == AES_WAIT_READY_COMP);
end

endmodule