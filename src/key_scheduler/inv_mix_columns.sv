module inv_mix_columns(in_key, out_key);

input [0:127] in_key;
output [0:127] out_key;

column_slice c0(in_key[0:31], out_key[0:31]);
column_slice c1(in_key[32:63], out_key[32:63]);
column_slice c2(in_key[64:95], out_key[64:95]);
column_slice c3(in_key[96:127], out_key[96:127]);

endmodule

module column_slice(in_word, out_word);

input [0:31] in_word;
output [0:31] out_word;

wire [7:0] in_bytes[0:3];
wire [7:0] in_bytes_0e[0:3];
wire [7:0] in_bytes_0b[0:3];
wire [7:0] in_bytes_0d[0:3];
wire [7:0] in_bytes_09[0:3];

wire [7:0] out_bytes[0:3];

assign in_bytes[0] = in_word[0:7];
assign in_bytes[1] = in_word[8:15];
assign in_bytes[2] = in_word[16:23];
assign in_bytes[3] = in_word[24:31];

genvar i;
generate
    for(i = 0; i < 4; i=i+1)
        begin: multiplier_instances
        aes_const_mult_0e m_0e(in_bytes[i], in_bytes_0e[i]);
        aes_const_mult_0b m_0b(in_bytes[i], in_bytes_0b[i]);
        aes_const_mult_0d m_0d(in_bytes[i], in_bytes_0d[i]);
        aes_const_mult_09 m_09(in_bytes[i], in_bytes_09[i]);
        end
endgenerate

assign out_bytes[0] = in_bytes_0e[0] ^ in_bytes_0b[1] ^ in_bytes_0d[2] ^ in_bytes_09[3];
assign out_bytes[1] = in_bytes_09[0] ^ in_bytes_0e[1] ^ in_bytes_0b[2] ^ in_bytes_0d[3];
assign out_bytes[2] = in_bytes_0d[0] ^ in_bytes_09[1] ^ in_bytes_0e[2] ^ in_bytes_0b[3];
assign out_bytes[3] = in_bytes_0b[0] ^ in_bytes_0d[1] ^ in_bytes_09[2] ^ in_bytes_0e[3];

assign out_word = {out_bytes[0], out_bytes[1], out_bytes[2], out_bytes[3]};

endmodule

/* 
    we need to implement custom multipliers for AES-field
    we need 0e, 0b, 0d, 09
    0e = 00001110
    0b = 00001011
    0d = 00001101
    09 = 00001001
*/
module aes_const_mult_0e(in_byte, out_byte);

input [7:0] in_byte;
output [7:0] out_byte;

wire [7:0] temp[0:2];
aes_const_mult_02 times_x_1(in_byte, temp[0]);
aes_const_mult_02 times_x_2(temp[0], temp[1]);
aes_const_mult_02 times_x_3(temp[1], temp[2]);

assign out_byte = temp[2] ^ temp[1] ^ temp[0];

endmodule

module aes_const_mult_0b(in_byte, out_byte);

input [7:0] in_byte;
output [7:0] out_byte;

wire [7:0] temp[0:2];

aes_const_mult_02 times_x_1(in_byte, temp[0]);
aes_const_mult_02 times_x_2(temp[0], temp[1]);
aes_const_mult_02 times_x_3(temp[1], temp[2]);

assign out_byte = temp[2] ^ temp[0] ^ in_byte;

endmodule

module aes_const_mult_0d(in_byte, out_byte);

input [7:0] in_byte;
output [7:0] out_byte;

wire [7:0] temp[0:2];

aes_const_mult_02 times_x_1(in_byte, temp[0]);
aes_const_mult_02 times_x_2(temp[0], temp[1]);
aes_const_mult_02 times_x_3(temp[1], temp[2]);

assign out_byte = temp[2] ^ temp[1] ^ in_byte;

endmodule

module aes_const_mult_09(in_byte, out_byte);

input [7:0] in_byte;
output [7:0] out_byte;

wire [7:0] temp[0:2];

aes_const_mult_02 times_x_1(in_byte, temp[0]);
aes_const_mult_02 times_x_2(temp[0], temp[1]);
aes_const_mult_02 times_x_3(temp[1], temp[2]);

assign out_byte = temp[2] ^ in_byte;

endmodule