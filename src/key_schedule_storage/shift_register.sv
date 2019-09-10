module shift_register
#(parameter WIDTH=32, parameter DEPTH=8)
    (
        clk, rst,
        din, dout,
        shift, new_in
    );

input clk, rst;

// those are data inputs/outputs
input [0:WIDTH-1] din;
output [0:WIDTH-1] dout;
// those inputs rule the behaviour of the shift register
input shift, new_in;

// this represents the internal state
reg [0:WIDTH-1] internal_regs[0:DEPTH-1];

// this variables are for the implementation of the shift register
reg [0:WIDTH-1] next_internal_regs[0:DEPTH-1];
/*
    the shift register shifts forward in two scenarios:
    - when a new word comes in
    - when "shift" is asserted and "forward_mode" is set to 1
    This means that data may also be fetched in reverse order, but
    can only be loaded in one direction, which is exactly what we need
    in order to generate a key schedule and fetch either the keys in the
    right order for encryption or decryption
*/

integer j;
always@(posedge clk)
begin
    for(j = 0; j < DEPTH; j++)
    begin
        if(rst)
            internal_regs[j] <= {WIDTH{1'b0}};
        else if(shift)
            internal_regs[j] <= next_internal_regs[j];
    end
end

integer i;
always@(*)
begin
    next_internal_regs[0] = new_in ? din : internal_regs[DEPTH-1];   
    
    for(i = 1; i < DEPTH; i++)
        next_internal_regs[i] = internal_regs[i-1];
end

assign dout = internal_regs[DEPTH-1];

endmodule