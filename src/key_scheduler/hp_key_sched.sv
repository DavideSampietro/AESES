/*
    AES key scheduler supporting the 3 different key sizes (128, 192, 256-bit keys).
    This module is meant to output 128 bits of the key schedule every clock cycle.
    This is exactly the amount of key material needed to perform a single round of
    the AES algorithm.
    "hp" stands for "high-performance".
    It is designed to abuse any kind of resource, especially data width!
*/
`include "aeses_defines.vh"

module hp_key_sched(
        clk, rst,
        new_key_schedule, input_key_length,
        aes_key_input,
        valid_key_out, round_key_material);

input clk, rst;

input new_key_schedule;
input [1:0] input_key_length;
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
reg [1:0] key_length;
reg [3:0] round_key_count;
reg [1:0] phase_count;

/****************************
   INTERNAL STATE UPDATES
****************************/
reg next_state;
reg [1:0] next_key_length;
reg [3:0] next_round_key_count;
reg [1:0] next_phase_count;

/****************************
   INTERNAL DATAPATH REGS
****************************/
reg [31:0] key_word_register[0:7];
reg [31:0] alternate_buffer[0:1];
reg [31:0] g_feed_in;
reg [7:0] rc;

/****************************
        DATAPATH WIRES
****************************/
wire [31:0] sbox_mapping;
wire [31:0] twisted_sbox_mapping;
wire [31:0] next_key_word_register[7:0];

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
        key_length <= `KEY_128;
        round_key_count <= 4'd0;
        phase_count <= 2'd0;
    end
    else begin
        state <= next_state;
        key_length <= next_key_length;
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
    next_key_length = key_length;
    next_phase_count = 2'd0;
    
    if(input_key_length == `KEY_256)
        next_round_key_count = 4'd0;
    else if(input_key_length == `KEY_192)
        next_round_key_count = 4'd2;
    else
        next_round_key_count = 4'd4;
    
    case(state)
        IDLE:
            begin
            if(input_key_length != `KEY_INVALID)
                begin
                next_key_length = input_key_length;
                next_state = new_key_schedule;
                end
            end
        SCHED:
            begin
            // is that legit??
            //if(round_key_count == 4'd15)
            if(round_key_count == 4'd14)
                next_state = IDLE;
            else
                begin
                next_round_key_count = round_key_count + 1;
                next_phase_count = key_length == `KEY_192 & phase_count == 2'd2 ? 2'd0 : phase_count + 1;
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
reg [1:0] g_feed_in_casez_selector;
reg [31:0] next_g_feed_in;
reg g_feed_in_we;
reg alternate_buffer_we;
wire [7:0] rc_times_2;
reg [7:0] next_rc;
reg next_rc_we;

reg new_input_we;
reg key_reg_128_we, key_reg_192_we, key_reg_256_we;

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
    if(key_reg_192_we)
        begin
        key_word_register[4] <= next_key_word_register[4];
        key_word_register[5] <= next_key_word_register[5];
        end
    if(key_reg_256_we)
        begin
        key_word_register[6] <= next_key_word_register[6];
        key_word_register[7] <= next_key_word_register[7];
        end
    
    if(g_feed_in_we)
        g_feed_in <= next_g_feed_in;
    
    if(next_rc_we)
        rc <= next_rc;
    
    if(alternate_buffer_we)
        begin
        alternate_buffer[0] <= key_word_register[4];
        alternate_buffer[1] <= key_word_register[5];
        end
end

always@(*)
begin
    // next_g_feed_in assignment
    g_feed_in_casez_selector = state == IDLE ? input_key_length : key_length;
    
    case(g_feed_in_casez_selector)
        `KEY_192:
            next_g_feed_in = next_key_word_register[5];
        `KEY_256:
            next_g_feed_in = state == IDLE | phase_count[0] ? next_key_word_register[7] : next_key_word_register[3];
        default: // also accounts for KEY_128
            next_g_feed_in = next_key_word_register[3];
    endcase
    
    // alternate_buffer write enable
    // alternate_buffer is used to store the two exceeding 32-bit words
    // produced by a single 192-bit key scheduling round for further use
    // in subsequent steps
    alternate_buffer_we = (key_length == `KEY_192); // && (state == SCHED);

    // skip the evaluation of the next rc in the second portion of the
    // 256-bit key scheduling round
    next_rc_we = (state == IDLE) | (state == SCHED &
        (key_length == `KEY_128 |
        key_length == `KEY_192 & phase_count != 2'd1 |
        key_length == `KEY_256 & ~phase_count[0]));
    next_rc = state == IDLE ? 8'h01 : rc_times_2;
    
    // temporary condition to enable the write of datapath registers
    // when a new key schedule comes in
    new_input_we = (state == IDLE) & new_key_schedule;
    
    key_reg_128_we = new_input_we | state == SCHED &
        (key_length == `KEY_128 |
        key_length == `KEY_192 & phase_count != 2'd1 |
        key_length == `KEY_256 & ~phase_count[0]);

    key_reg_192_we = new_input_we | state == SCHED &
        (key_length == `KEY_192 & phase_count != 2'd1 |
        key_length == `KEY_256 & phase_count[0]);

    key_reg_256_we = new_input_we | state == SCHED &
        (key_length == `KEY_256 & phase_count[0]);
    
    g_feed_in_we = new_input_we | state == SCHED &
        ~(key_length == `KEY_192 & phase_count == 2'd1);
        
end

/****************************
   DATAPATH IMPLEMENTATION
****************************/
// custom, hardwired sbox
hardwired_sbox sbox(g_feed_in, sbox_mapping);
assign twisted_sbox_mapping = {sbox_mapping[23:16] ^ rc, sbox_mapping[15:0], sbox_mapping[31:24]};

// these assignments are the same for all the key lengths!!
assign next_key_word_register[0] = state == IDLE ? input_key_words[0] : twisted_sbox_mapping ^ key_word_register[0];
assign next_key_word_register[1] = state == IDLE ? input_key_words[1] : next_key_word_register[0] ^ key_word_register[1];
assign next_key_word_register[2] = state == IDLE ? input_key_words[2] : next_key_word_register[1] ^ key_word_register[2];
assign next_key_word_register[3] = state == IDLE ? input_key_words[3] : next_key_word_register[2] ^ key_word_register[3];

// this assignments are handled differently for 192- and 256-bit keys
assign next_key_word_register[4] = state == IDLE ? input_key_words[4] :
    key_word_register[4] ^ (key_length == `KEY_256 ?
    sbox_mapping :
    next_key_word_register[3]);

assign next_key_word_register[5] = state == IDLE ? input_key_words[5] : next_key_word_register[4] ^ key_word_register[5];
assign next_key_word_register[6] = state == IDLE ? input_key_words[6] : next_key_word_register[5] ^ key_word_register[6];
assign next_key_word_register[7] = state == IDLE ? input_key_words[7] : next_key_word_register[6] ^ key_word_register[7];

/****************************
       OUTPUT SIGNALS
****************************/
reg [1:0] output_key_selector;
reg [31:0] next_key_material[0:3];
wire [0:127] pack_next_key;

always@(*)
begin
    output_key_selector[1] = (key_length == `KEY_192) & (phase_count != 2'd0);
    output_key_selector[0] = (key_length != `KEY_128) & phase_count[0];
    
    case(output_key_selector)
        // this condition always happens during a 128-bit schedule
        // this condition also happens on even phases of 256-bit keys
        // this condition happens during the first phase of 192-bit keys
        2'b00:
            begin
            next_key_material[0] = key_word_register[0];
            next_key_material[1] = key_word_register[1];
            next_key_material[2] = key_word_register[2];
            next_key_material[3] = key_word_register[3];
            end
        // this condition only happens in odd phases of 256-bit schedules
        2'b01:
            begin
            next_key_material[0] = key_word_register[4];
            next_key_material[1] = key_word_register[5];
            next_key_material[2] = key_word_register[6];
            next_key_material[3] = key_word_register[7];
            end
        // this condition only happens on the second phase of 192-bit schedules
        // alternate buffer hold the last two words of the previous sched. round
        2'b11:
            begin
            next_key_material[0] = alternate_buffer[0];
            next_key_material[1] = alternate_buffer[1];
            next_key_material[2] = key_word_register[0];
            next_key_material[3] = key_word_register[1];
            end
        // this condition is the third phase of 192-bit schedules
        // the scheduler doesn't write datapath regs in this phase!
        2'b10:
            begin
            next_key_material[0] = key_word_register[2];
            next_key_material[1] = key_word_register[3];
            next_key_material[2] = key_word_register[4];
            next_key_material[3] = key_word_register[5];
            end
    endcase
end

assign pack_next_key = {next_key_material[0], next_key_material[1], next_key_material[2], next_key_material[3]};
assign round_key_material = state == IDLE ? 128'd0 : pack_next_key;

assign valid_key_out = state;

endmodule