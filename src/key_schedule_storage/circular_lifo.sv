module circular_lifo
#(parameter WIDTH=32, parameter DEPTH=8)
    (
        clk, rst,
        din, dout,
        en, r_wneg
    );

input clk, rst;
input [0:WIDTH-1] din;
input en, r_wneg;

output [0:WIDTH-1] dout;

reg [0:WIDTH-1] internal_regs[0:DEPTH-1];
reg [0:WIDTH-1] next_internal_regs[0:DEPTH-1];

/*
    if "en" is set, the internal state of the LIFO is shifted.
    Otherwise is left as is.
    If rw_neg=1, then a read operation is performed and the
    lifo behaves like a circular buffer. The topmost entry is popped
    and enqueued back.
    If rw_neg=0, then a new topmost entry is pushed.
*/

integer j;
always@(posedge clk)
begin
    for(j=0; j < DEPTH; j++)
    begin
        if(rst)
            internal_regs[j] <= {WIDTH{1'b0}};
        else if(en)
            internal_regs[j] <= next_internal_regs[j];
    end
end

integer i;
always@(*)
begin
    next_internal_regs[0] = r_wneg ? din : internal_regs[1];
    next_internal_regs[DEPTH-1] = r_wneg ? internal_regs[DEPTH-2] : internal_regs[0];
    
    for(i=1; i < DEPTH-1; i++)
        next_internal_regs[i] = r_wneg ? internal_regs[i-1] : internal_regs[i+1];
end

assign dout = internal_regs[0];

endmodule