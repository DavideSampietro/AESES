module aes_const_mult_02(in_byte, out_byte);

input [7:0] in_byte;
output [7:0] out_byte;

parameter rest = 8'h1b;

wire [7:0] in_byte_x;
wire carry;

// AES polynomial: x^8 + x^4 + x^3 + x + 1 = 0
// which means:    x^8 = x^4 + x^3 + x + 1

assign {carry, in_byte_x} = {in_byte, 1'b0};
assign out_byte = carry ? in_byte_x ^ rest : in_byte_x;

endmodule