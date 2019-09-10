`include "aeses_defines.vh"

module aes_cipher(
        clk, rst,
        pctx, e_dneg,
        round_key, inv_round_key,
        mode, start_operation,
        cptx, valid_cptx);

input clk, rst;

input [0:127] pctx;
input e_dneg;
input [0:127] round_key, inv_round_key;

input [1:0] mode;
input start_operation;

output [0:127] cptx;
output reg valid_cptx;

/****************************
   ENCODING OF PARAMETERS
****************************/
localparam IDLE = 1'b0;
localparam OPERATION = 1'b1;

/****************************
       INTERNAL STATE
****************************/
reg state;
reg [0:127] aes_state;
reg [3:0] round_count;

/****************************
   DATAPATH FINAL VARIABLES
****************************/
wire [0:127] xored_next_state;
wire [0:127] no_mix_col_next_state;
wire [0:127] r_key;

/****************************
   INTERNAL STATE UPDATES
****************************/
reg [3:0] next_round_count;
reg next_state;
reg next_valid_cptx;

reg [0:127] next_aes_state;
reg next_aes_state_we;

/****************************
   SYNCHRONOUS STATE UPDATE
****************************/
always@(posedge clk)
begin
    if(rst)
    begin
        state <= IDLE;
        round_count <= 4'd0;
        valid_cptx <= 1'b0;
    end
    else
    begin
        state <= next_state;
        round_count <= next_round_count;
        valid_cptx <= next_valid_cptx;
    end
end

always@(posedge clk)
begin
    if(!rst && next_aes_state_we)
        aes_state <= next_aes_state;
end

/****************************
   NEXT STATE EVALUATION
****************************/

/*
    AES-256 is the one involving more rounds - they are 14 + initial key add.
    AES-192 and AES-128 are 12 and 10 respectively.
    For AES-256, the round_count is in the range [0, 13], while for the other
    two, [2, 13] and [4, 13].
    The 13th round (counting from 0) will be the one not performing the MixColumns
    both in the encryption and decryption flow.
*/
always@(*)
begin
    next_round_count = round_count;
    next_state = state;
    next_valid_cptx = 1'b0;
    
    case(state)
        IDLE:
            begin
            if(start_operation)
                begin
                next_state = OPERATION;
                
                if(mode == `KEY_256)
                    next_round_count = 4'd0;
                else if(mode == `KEY_192)
                    next_round_count = 4'd2;
                else
                    next_round_count = 4'd4;
                
                end
            end
        OPERATION:
            begin
            next_round_count = round_count + 1;
            if(round_count == 4'd13)
                begin
                next_state = IDLE;
                next_valid_cptx = 1'b1;
                end
            end
    endcase
end

// aes_state logic
reg [0:127] pre_keyaddition_1, pre_keyaddition;
reg no_mix_col_round;

always@(*)
begin
    // in this case, the outcome of the sboxes needs to be taken
    // rather than the result of xoring together all the tboxes 32-bit wise
    no_mix_col_round = round_count == 4'd13;
    
    // if IDLE, the initial round-key addition is performed againt the fed
    // ptx (encryption) or ctx (decryption)
    pre_keyaddition_1 = no_mix_col_round ? no_mix_col_next_state : pctx;
    pre_keyaddition = ~no_mix_col_round && state != IDLE ? xored_next_state : pre_keyaddition_1;
    
    next_aes_state = pre_keyaddition ^ r_key;
    
    next_aes_state_we = state != IDLE | start_operation;
end

/****************************
   DATAPATH IMPLEMENTATION
****************************/
wire [31:0] aes_state_words[0:3];
wire [0:79] sbox_tbox_mapping[0:15];
// encryption/decryption selection
wire [0:7] sbox_mapping [0:15];
wire [0:31] tbox_mapping [0:15];

// simply split the 128-bit state into 32-bit words
// this will serve as inputs for the tboxes!
genvar i;
generate
    for(i = 0; i < 4; i++)
        begin: split_aes_state
        assign aes_state_words[i] = aes_state[i*32:(i+1)*32-1];
        end
endgenerate

// map current state to sbox-tbox
direct_sbox_tbox st0(aes_state_words[0], sbox_tbox_mapping[0:3]);
direct_sbox_tbox st1(aes_state_words[1], sbox_tbox_mapping[4:7]);
direct_sbox_tbox st2(aes_state_words[2], sbox_tbox_mapping[8:11]);
direct_sbox_tbox st3(aes_state_words[3], sbox_tbox_mapping[12:15]);

// select between encryption and decryption tables basing on e_dneg
genvar j;
generate
    for(j = 0; j < 16; j=j+1)
    begin: encryption_decryption_xor_mux
        assign tbox_mapping[j] = e_dneg ? sbox_tbox_mapping[(5*j)%16][8:39]:
            sbox_tbox_mapping[(13*j)%16][48:79];
        assign sbox_mapping[j] = e_dneg ? sbox_tbox_mapping[(5*j)%16][0:7]:
            sbox_tbox_mapping[(13*j)%16][40:47];
    end
endgenerate

// select between direct and inverse keyschedule
assign r_key = e_dneg ? round_key : inv_round_key;

// xor together all the tbox outcomes
// assemble the 128-bit sbox results on the last round (no MixColumns stage)

assign no_mix_col_next_state =
    {sbox_mapping[0], sbox_mapping[1], sbox_mapping[2], sbox_mapping[3],
     sbox_mapping[4], sbox_mapping[5], sbox_mapping[6], sbox_mapping[7],
     sbox_mapping[8], sbox_mapping[9], sbox_mapping[10], sbox_mapping[11],
     sbox_mapping[12], sbox_mapping[13], sbox_mapping[14], sbox_mapping[15]};

assign xored_next_state =
    {tbox_mapping[0] ^ tbox_mapping[1] ^ tbox_mapping[2] ^ tbox_mapping[3],
     tbox_mapping[4] ^ tbox_mapping[5] ^ tbox_mapping[6] ^ tbox_mapping[7],
     tbox_mapping[8] ^ tbox_mapping[9] ^ tbox_mapping[10] ^ tbox_mapping[11],
     tbox_mapping[12] ^ tbox_mapping[13] ^ tbox_mapping[14] ^ tbox_mapping[15]};

/****************************
       OUTPUT SIGNALS
****************************/
assign cptx = valid_cptx ? aes_state : {64{2'b10}};

endmodule