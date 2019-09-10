`timescale 1ns/1ps

`include "wrapper_defines.vh"
`include "aeses_defines.vh"
`include "uart/uart_defines.v"

module fpga_top_tb ();

parameter HALF_CLK_PERIOD 				=`SIM_HALF_CLK_PERIOD_DEF;
parameter CLK_PERIOD 					=HALF_CLK_PERIOD * 2;
parameter UART_NUM_CLK_TICKS_BIT 		=`SIM_UART_NUM_CLK_TICKS_BIT; // CLK_MHz / (baud_rate)
parameter UART_NUM_DWORD_BITS			= 8;
parameter UART_NUM_STOP_BITS			= 1;

reg		clk;
reg		rst;
reg		pad_tx_tb_uart;
wire	pad_rx_uart_tb;

reg [UART_NUM_DWORD_BITS-1:0] recv_data;
//integer cnt;

wire 	[9:0] LED;

fpga_top
top0(	
	.CLK_PIN(clk),
	.RST_PIN(rst),
	.RX_PIN(pad_tx_tb_uart),
	.TX_PIN(pad_rx_uart_tb),
	.M_LED(LED)
	);

//////////////////////////////
/// UART SEND RECEIVE TASKS	//
//////////////////////////////
task sendByteUart;
		input [7:0] wdata;
		integer i;
	begin
		$display("@%0t: START tb.sendByteUart(%b)",$time,wdata);
	//start bit
		@(posedge clk);
		pad_tx_tb_uart <= 0;
		i<=0;
		repeat(UART_NUM_CLK_TICKS_BIT) @(posedge clk);
	// data bit from the LSB
		repeat(UART_NUM_DWORD_BITS)
		begin
			i<=i+1;
			pad_tx_tb_uart <= wdata[i];
			repeat(UART_NUM_CLK_TICKS_BIT) @(posedge clk);
		end
	//stop bit(s)
		repeat(UART_NUM_STOP_BITS)
		begin
			pad_tx_tb_uart <= 1;
			repeat(UART_NUM_CLK_TICKS_BIT) @(posedge clk);
		end
		$display("@%0t: END tb.sendByteUart(%b)",$time,wdata);
	end
endtask


task recvByteUart;
	output reg [7:0] rdata;
		begin
		$display("@%0t: START tb.recvByteUart()",$time);

	// looking for start bit
	while(pad_rx_uart_tb!=0) // can be X !!!
		@(negedge pad_rx_uart_tb);
		
	$display("@%0t: uart.recv() startbit: %b",$time,pad_rx_uart_tb);
	// wait to be in the middle of the sent star start bit
	repeat(UART_NUM_CLK_TICKS_BIT/2) @(posedge clk);

	// receive all the data bits from the LSB
		repeat(UART_NUM_DWORD_BITS)
		begin
			repeat(UART_NUM_CLK_TICKS_BIT) @(posedge clk);
			rdata <= {pad_rx_uart_tb , rdata[UART_NUM_DWORD_BITS-1:1]};
		end

	// receive the stop bit(s)
		repeat(UART_NUM_STOP_BITS)
		begin
			repeat(UART_NUM_CLK_TICKS_BIT) @(posedge clk);
			if(pad_rx_uart_tb!=1)
				$display("@%0t: ERROR tb.recvByteUart() WRONG STOP BIT",$time);
			//assert(pad_rx_uart_tb==1);
		end

	$display("@%0t: END tb.recvByteUart(%b)",$time,rdata);
end
endtask

//////////////////////////////
/// AESES TESTERS			//
//////////////////////////////
wire [1:0] mode = `KEY_192; // test mode selection
wire encryption = 1'b1;
 
wire [0:255] key_128_bit = 256'h000102030405060708090a0b0c0d0e0f00000000000000000000000000000000;
wire [0:255] key_192_bit = 256'h000102030405060708090a0b0c0d0e0f10111213141516170000000000000000;
wire [0:255] key_256_bit = 256'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f;

wire [0:127] test_ptx = 128'h00112233445566778899aabbccddeeff;

wire [0:127] ctx_128 = 128'h69c4e0d86a7b0430d8cdb78070b4c55a;
wire [0:127] ctx_192 = 128'hdda97ca4864cdfe06eaf70a0ec0d7191;
wire [0:127] ctx_256 = 128'h8ea2b7ca516745bfeafc49904b496089;

reg [0:255] input_key;
wire [7:0] input_key_bytes[0:31];

reg [0:127] start_value;
wire [7:0] start_value_bytes[0:15];

reg [0:127] final_reference;
reg [7:0] result[0:15];

always@(*)
begin
    case(mode)
        `KEY_128:
            begin
            input_key = key_128_bit;
            start_value = ctx_128;
            //final_value = ctx_128;
            final_reference = ctx_128;
            end
        `KEY_192:
            begin
            input_key = key_192_bit;
            start_value = ctx_192;
            //final_value = ctx_192;
            final_reference = ctx_192;
            end
        `KEY_256:
            begin
            input_key = key_256_bit;
            start_value = ctx_256;
            //final_value = ctx_256;
            final_reference = ctx_256;
            end
        default:
            begin
            input_key = 256'h0;
            start_value = {64{2'b10}};
            //final_value = {64{2'b10}};
            final_reference = {64{2'b10}};
            end
    endcase
    
    if(encryption)
        start_value = test_ptx;
    else
        final_reference = test_ptx;
end

genvar k;
generate
    for(k=0; k<32; k=k+1)
    begin: split_key_bytes
        assign input_key_bytes[k] = input_key[8*k:8*(k+1)-1]; 
    end
    
    for(k=0; k<16; k=k+1)
    begin: split_ptx_ctx_i
        assign start_value_bytes[k] = start_value[8*k:8*(k+1)-1];
    end
endgenerate

//////////////////////////////
/// CLOCK GENERATOR			//
//////////////////////////////
always #HALF_CLK_PERIOD clk = ~clk;

/*initial
begin
		$dumpvars(0,fpga_top_tb );
		$dumpfile("out.vcd");
		// cadence stuff
		//$shm_open("waves");
		//$shm_probe("ASM");
end*/

// The test sequence
initial
begin:initial_tb
    automatic integer i;
    integer cnt;
	//cnt=0;
	
	clk <= 0;
  	rst = 0;
	pad_tx_tb_uart = 1;
	recv_data =0;
	repeat(2) @(posedge clk);
  	rst = 1;
	
	repeat(2) @(posedge clk);
	
	// send key-schedule control byte
	sendByteUart({1'b1, mode, 5'd0});
	// send key...
	cnt = 0;
	repeat(32)
	begin
	   sendByteUart(input_key_bytes[cnt]);
	   cnt=cnt+1;
    end
    
    repeat(3)
    begin
        // send operation control byte
        sendByteUart({1'b0, mode, 1'b1, encryption, 3'd0});
        // send ptx/ctx...
        cnt = 0;
        repeat(16)
        begin
            sendByteUart(start_value_bytes[cnt]);
            cnt=cnt+1;
        end
        
        // read the result!!
        cnt = 0;
        repeat(16)	
        begin
            recvByteUart(recv_data);
            result[cnt]=recv_data;
            cnt=cnt+1;
        end
        
        $write("#################\n");
        for(i=0;i<16;i=i+1)
           $write("%h",result[i]);
        $write("\n%h\n", final_reference);
        $write("#################\n");
    end
	
	#20 $finish;
end

endmodule