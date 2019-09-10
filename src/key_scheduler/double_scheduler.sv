module double_scheduler(
        clk, rst,
        new_key_schedule, input_key_length,
        aes_key_input,
        valid_key_out, round_key_material, inv_round_key_material);

input clk, rst;

input new_key_schedule;
input [1:0] input_key_length;
input [0:255] aes_key_input;

output reg valid_key_out;
output reg [0:127] round_key_material;
output [0:127] inv_round_key_material;

/****************************
       INTERNAL STATE
****************************/
wire [0:127] next_direct_round_key, next_inverse_round_key;
wire next_valid_key_out;
reg first_last_selector;

always@(posedge clk)
begin
    round_key_material <= next_direct_round_key;
    first_last_selector <= valid_key_out & next_valid_key_out;

    if(rst)
        valid_key_out <= 1'b0;
    else
        valid_key_out <= next_valid_key_out;
end

hp_key_sched aes_key_scheduler(
    .clk(clk), .rst(rst),
    .new_key_schedule(new_key_schedule), .input_key_length(input_key_length),
    .aes_key_input(aes_key_input),
    .valid_key_out(next_valid_key_out), .round_key_material(next_direct_round_key));

inv_mix_columns inverse_scheduler(round_key_material, next_inverse_round_key);

assign inv_round_key_material = first_last_selector & next_valid_key_out ? next_inverse_round_key : round_key_material;

endmodule