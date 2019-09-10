/*
    AES key scheduler supporting the 3 different key sizes (128, 192, 256-bit keys).
    This module is meant to output 128 bits of the key schedule every clock cycle.
    This is exactly the amount of key material needed to perform a single round of
    the AES algorithm.
    "hp" stands for "high-performance".
    It is designed to abuse any kind of resource, especially data width!
*/
`include "aeses_defines.vh"

module key_sched_256(
        clk, rst,
        new_key_schedule, aes_key_input,
        valid_key_out, round_key_material);

input clk, rst;

input new_key_schedule;
input [0:255] aes_key_input;

output valid_key_out;
output [0:127] round_key_material;

/****************************
   ENCODING OF PARAMETERS
****************************/
localparam IDLE = 1'b0;
localparam SCHED = 1'b1;

/****************************
       INTERNAL STATE
****************************/
reg state;
reg [3:0] round_key_count;
reg [1:0] phase_count;

/****************************
   INTERNAL STATE UPDATES
****************************/
reg next_state;
reg [3:0] next_round_key_count;
reg [1:0] next_phase_count;

/****************************
   INTERNAL DATAPATH REGS
****************************/
reg [31:0] key_word_register[0:7];
reg [31:0] sbox_mapping;
reg [7:0] rc;

/****************************
        DATAPATH WIRES
****************************/
wire [31:0] next_key_word_register[7:0];
reg [31:0] next_g_feed_in;
wire [31:0] next_sbox_mapping;
wire [31:0] twisted_sbox_mapping;

// just split the input AES key in 32-bit words
// that fit the internal datapath registers
wire [31:0] input_key_words[0:7];

genvar i;
generate
    for(i = 0; i < 8; i++)
    begin: split_input_key
        assign input_key_words[i] = aes_key_input[i*32:32*(i+1)-1];
    end
endgenerate

/****************************
   SYNCHRONOUS STATE UPDATE
****************************/
always@(posedge clk)
begin
    if(rst)
    begin
        state <= IDLE;
        round_key_count <= 4'd0;
        phase_count <= 2'd0;
    end
    else begin
        state <= next_state;
        round_key_count <= next_round_key_count;
        phase_count <= next_phase_count;
    end
end

/****************************
   NEXT STATE EVALUATION
****************************/
always@(*)
begin
    next_state = state;
    next_phase_count = 2'd0;
    next_round_key_count = 4'd0;
    
    case(state)
        IDLE:
            begin
            next_state = new_key_schedule;
            if(new_key_schedule)
                next_phase_count = 2'd1;
            end
        SCHED:
            begin
            if(round_key_count == 4'd15)
                begin
                next_state = IDLE;
                next_phase_count = 4'd0;
                end
            else
                begin
                next_round_key_count = round_key_count + valid_key_out;
                next_phase_count = phase_count + 1;
                end
            end
    endcase
end

/****************************
  DATAPATH REGISTER UPDATE
****************************/

// g_feed_in always holds the value which is fed to the sboxes
// it is used both for the 128/192-bit schedule and
// for the first and second half of the 256-bit schedule round (in the latter case,
// it is used as is without twisting)
wire [7:0] rc_times_2;
reg [7:0] next_rc;
reg next_rc_we;

reg new_input_we;
reg key_reg_128_we, key_reg_256_we;

// times 0x02 multiplier in the AES field
aes_const_mult_02 next_rc_calculator(rc, rc_times_2);

always@(posedge clk)
begin
    if(key_reg_128_we)
        begin
        key_word_register[0] <= next_key_word_register[0];
        key_word_register[1] <= next_key_word_register[1];
        key_word_register[2] <= next_key_word_register[2];
        key_word_register[3] <= next_key_word_register[3];
        end
    if(key_reg_256_we)
        begin
        key_word_register[6] <= next_key_word_register[6];
        key_word_register[7] <= next_key_word_register[7];
        key_word_register[4] <= next_key_word_register[4];
        key_word_register[5] <= next_key_word_register[5];
        end
    
    
    if(state == SCHED & phase_count[0])
        sbox_mapping <= next_sbox_mapping;
    
    if(next_rc_we)
        rc <= next_rc;
end

always@(*)
begin
    // temporary condition to enable the write of datapath registers
    // when a new key schedule comes in
    new_input_we = (state == IDLE) & new_key_schedule;
    
    //next_g_feed_in = state == IDLE | phase_count ? next_key_word_register[7] : next_key_word_register[3];
    next_g_feed_in = ~phase_count[1] ? key_word_register[7] : key_word_register[3];    

    // skip the evaluation of the next rc in the second portion of the
    // 256-bit key scheduling round
    next_rc_we = (state == IDLE) | (state == SCHED & phase_count == 2'd3);
    next_rc = state == IDLE ? 8'h01 : rc_times_2;
    
    key_reg_128_we = new_input_we | state == SCHED & phase_count == 2'd2;
    key_reg_256_we = new_input_we | (state == SCHED & phase_count == 2'd0 & round_key_count != 4'd0);
end

/****************************
   DATAPATH IMPLEMENTATION
****************************/
// custom, hardwired sbox
hardwired_sbox sbox(next_g_feed_in, next_sbox_mapping);
assign twisted_sbox_mapping = {sbox_mapping[23:16] ^ rc, sbox_mapping[15:0], sbox_mapping[31:24]};

// these assignments are the same for all the key lengths!!
assign next_key_word_register[0] = state == IDLE ? input_key_words[0] : twisted_sbox_mapping ^ key_word_register[0];
assign next_key_word_register[1] = state == IDLE ? input_key_words[1] : next_key_word_register[0] ^ key_word_register[1];
assign next_key_word_register[2] = state == IDLE ? input_key_words[2] : next_key_word_register[1] ^ key_word_register[2];
assign next_key_word_register[3] = state == IDLE ? input_key_words[3] : next_key_word_register[2] ^ key_word_register[3];

// this assignments are handled differently for 192- and 256-bit keys
assign next_key_word_register[4] = state == IDLE ? input_key_words[4] : sbox_mapping ^ key_word_register[4];
assign next_key_word_register[5] = state == IDLE ? input_key_words[5] : next_key_word_register[4] ^ key_word_register[5];
assign next_key_word_register[6] = state == IDLE ? input_key_words[6] : next_key_word_register[5] ^ key_word_register[6];
assign next_key_word_register[7] = state == IDLE ? input_key_words[7] : next_key_word_register[6] ^ key_word_register[7];

/****************************
       OUTPUT SIGNALS
****************************/
reg [31:0] next_key_material[0:3];
wire [0:127] pack_next_key;

always@(*)
begin
    case(phase_count[1])
        1'b0:
            begin
            next_key_material[0] = key_word_register[0];
            next_key_material[1] = key_word_register[1];
            next_key_material[2] = key_word_register[2];
            next_key_material[3] = key_word_register[3];
            end
        1'b1:
            begin
            next_key_material[0] = key_word_register[4];
            next_key_material[1] = key_word_register[5];
            next_key_material[2] = key_word_register[6];
            next_key_material[3] = key_word_register[7];
            end
    endcase
end

assign pack_next_key = {next_key_material[0], next_key_material[1], next_key_material[2], next_key_material[3]};
assign round_key_material = state == IDLE ? {64{2'b10}} : pack_next_key;

assign valid_key_out = state == SCHED & phase_count[0];

endmodule