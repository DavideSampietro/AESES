`include "aeses_defines.vh"

// only supporting AES-256

module aeses_lite(
        clk, rst,
        enable_key_schedule_i, aes_key_i,
        enable_op_i, aes_blk_i,
        valid_o, aes_blk_o, ready_o);

input clk, rst;

// these are the inputs to the key scheduler
input enable_key_schedule_i;
input [0:255] aes_key_i;

// these are the inputs to the encryption module
input enable_op_i;
input [0:127] aes_blk_i;

// this is the output of the module
output valid_o;
output [0:127] aes_blk_o;
output ready_o;

/****************************
   ENCODING OF PARAMETERS
****************************/
localparam NEW_SCHEDULE = 2'b01;
localparam READY = 2'b10;
localparam OPERATION = 2'b11;

/****************************
       INTERNAL STATE
****************************/
reg [1:0] state;
reg [4:0] round_count;

// next state
reg [1:0] next_state;
reg [4:0] next_round_count;

/****************************
      KEY SCHEDULER MOD
****************************/
reg start_scheduling, next_start_scheduling;

wire valid_key_out;
wire [0:127] round_key_output;

key_sched_256 sched0(
    .clk(clk), .rst(rst),
    .new_key_schedule(start_scheduling),
    .aes_key_input(aes_key_i),
    .valid_key_out(valid_key_out), .round_key_material(round_key_output));

/****************************
     SHIFT REGISTER MOD
****************************/
reg shift_keystore, shift_reg_new_input;
wire [0:127] round_key;

shift_register #(.WIDTH(128), .DEPTH(15)) key_store(
    .clk(clk), .rst(rst),
    .din(round_key_output), .dout(round_key),
    .shift(shift_keystore), .new_in(valid_key_out));

/****************************
       ENCRYPTION MOD
****************************/
reg start_operation, next_start_operation;

aes_cipher encryption_core(
    .clk(clk), .rst(rst),
    .ptx(aes_blk_i), .round_key(round_key),
    .start_operation(start_operation),
    .ctx(aes_blk_o), .valid_ctx(valid_o));

// I place here the very last output!!!
assign ready_o = state == READY && ~rst;

/****************************
       INTERNAL STATE
****************************/
always@(posedge clk)
begin
    if(rst)
        begin
        state <= READY;
        end
    else
        begin
        state <= next_state;
        round_count <= next_round_count;
        end
end

always@(*)
begin
    next_state = state;
    
    next_start_scheduling = 1'b0;
    next_start_operation = 1'b0;
    
    next_round_count = 4'd0;

    case(state)
        NEW_SCHEDULE:
            begin
            next_round_count = round_count + valid_key_out;
            if(round_count == 5'd15)
                begin
                next_state = READY;
                end
            end
        READY:
            begin
            if(enable_key_schedule_i)
                begin
                next_state = NEW_SCHEDULE;
                next_start_scheduling = 1'b1;
                end
            else if(enable_op_i)
                begin
                next_state = OPERATION;
                next_start_operation = 1'b1;
                next_round_count = 4'd0;
                end
            end
        OPERATION:
            begin
            next_round_count = round_count + 1;
            if(round_count == 4'd15)
                next_state = READY;
            end
        default:
            next_state = READY;
    endcase
end

/****************************
         MISCELLANEA
****************************/
always@(posedge clk)
begin
    start_scheduling <= next_start_scheduling;
    start_operation <= next_start_operation;
end

always@(*)
begin
    shift_keystore = valid_key_out | state == OPERATION && !valid_o;
end

endmodule