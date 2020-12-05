module MUX32(
    input [31 : 0] input0, input1,
    input ctrl_signal,
    output [31 : 0] out
);

assign out = (ctrl_signal == 0)? input0 : input1;

endmodule