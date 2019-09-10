`include "aeses_defines.vh"

module aeses_core(
        clk, rst,
        enable_key_schedule_i, key_mode_i, aes_key_i,
        enable_op_i, enc_decneg_i, aes_blk_i,
        valid_o, aes_blk_o, ready_o);

input clk, rst;

// these are the inputs to the key scheduler
input enable_key_schedule_i;
input [1:0] key_mode_i;
input [0:255] aes_key_i;

// these are the inputs to the encryption module
input enable_op_i, enc_decneg_i;
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
reg [1:0] aes_key_mode;
reg [4:0] round_count;
reg enc_decneg;

// next state
reg [1:0] next_state;
reg [1:0] next_aes_key_mode;
reg [4:0] next_round_count;
reg next_enc_decneg;

/****************************
      KEY SCHEDULER MOD
****************************/
reg start_scheduling, next_start_scheduling;

wire valid_key_out;
wire [0:127] round_key_output;
wire [0:127] inv_round_key_output;

double_scheduler aes_key_scheduler(
    .clk(clk), .rst(rst),
    .new_key_schedule(start_scheduling), .input_key_length(aes_key_mode),
    .aes_key_input(aes_key_i),
    .valid_key_out(valid_key_out),
    .round_key_material(round_key_output),
    .inv_round_key_material(inv_round_key_output)
);

/****************************
     SHIFT REGISTER MOD
****************************/
reg shift_keystore, shift_reg_new_input;
wire [0:127] round_key;

shift_register #(.WIDTH(128), .DEPTH(15)) key_store(
    .clk(clk), .rst(rst),
    .din(round_key_output), .dout(round_key),
    .shift(shift_keystore), .new_in(shift_reg_new_input));

/****************************
       LIFO BUFFER MOD
****************************/
reg lifo_en, lifo_rw;
wire [0:127] inv_round_key;

circular_lifo #(.WIDTH(128), .DEPTH(15)) inv_key_store(
    .clk(clk), .rst(rst),
    .din(inv_round_key_output), .dout(inv_round_key),
    .en(lifo_en), .r_wneg(lifo_rw));

/****************************
       ENCRYPTION MOD
****************************/
reg start_operation, next_start_operation;

aes_cipher encryption_core(
    .clk(clk), .rst(rst),
    .pctx(aes_blk_i), .e_dneg(enc_decneg),
    .round_key(round_key), .inv_round_key(inv_round_key),
    .mode(aes_key_mode), .start_operation(start_operation),
    .cptx(aes_blk_o), .valid_cptx(valid_o));

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
        aes_key_mode <= next_aes_key_mode;
        round_count <= next_round_count;
        enc_decneg <= next_enc_decneg;
        end
end

always@(*)
begin
    next_state = state;
    next_aes_key_mode = aes_key_mode;
    
    next_start_scheduling = 1'b0;
    next_start_operation = 1'b0;
    
    next_round_count = 4'd0;
    next_enc_decneg = enc_decneg;

    case(state)
        NEW_SCHEDULE:
            begin
            next_round_count = round_count + 1;
            if(round_count == 5'd16)
                begin
                next_state = READY;
                end
            end
        READY:
            begin
            if(enable_key_schedule_i && key_mode_i != `KEY_INVALID)
                begin
                next_state = NEW_SCHEDULE;
                next_aes_key_mode = key_mode_i;
                next_start_scheduling = 1'b1;
                end
            else if(enable_op_i)
                begin
                next_state = OPERATION;
                next_start_operation = 1'b1;
                next_round_count = 4'd0;
                next_enc_decneg = enc_decneg_i;
                end
            end
        OPERATION:
            begin
            next_round_count = round_count + 1;
            if(round_count == 4'd14)
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
    shift_keystore = state == NEW_SCHEDULE || state == OPERATION;
    shift_reg_new_input = state == NEW_SCHEDULE;
    
    lifo_en = (state == NEW_SCHEDULE && valid_key_out) || state == OPERATION;
    lifo_rw = state == NEW_SCHEDULE;
end

endmodule