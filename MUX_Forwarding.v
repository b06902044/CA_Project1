module MUX_Forwarding(
    input [31 : 0] ID_EX_RS, EX_MEM_ALUout, WB_WBdata,
    input [1 : 0]  ctrl,
    output [31 : 0] forword_out
);

assign forword_out = (ctrl == 2'b00)? ID_EX_RS :
                     (ctrl == 2'b01)? WB_WBdata : EX_MEM_ALUout;

endmodule