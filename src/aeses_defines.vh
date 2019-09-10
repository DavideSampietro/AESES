// encoding of the different key modes
`define KEY_128 2'b00
`define KEY_192 2'b01
`define KEY_256 2'b11
`define KEY_INVALID 2'b10

// AESES_CONTROL_BYTE fields
`define AESES_RESET_BIT 7
`define AESES_KEY_256_BIT 6
`define AESES_KEY_FIELD 6:5
`define AESES_EN_BIT 4
`define AESES_ENC_DEC 3