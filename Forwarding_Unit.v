module Forwarding_Unit(
    input [4 : 0]  ID_EX_RS1, ID_EX_RS2, EX_MEM_RD, MEM_WB_RD,
    input          EX_MEM_RegWrite, MEM_WB_RegWrite,
    output [1 : 0] Forward_Control1, Forward_Control2 
);

assign Forward_Control1 = (EX_MEM_RegWrite === 1 && EX_MEM_RD !== 0 && EX_MEM_RD === ID_EX_RS1)? 2'b10 : 
                          (MEM_WB_RegWrite === 1 && MEM_WB_RD !== 0 && MEM_WB_RD === ID_EX_RS1)? 2'b01 : 2'b00;
assign Forward_Control2 = (EX_MEM_RegWrite === 1 && EX_MEM_RD !== 0 && EX_MEM_RD === ID_EX_RS2)? 2'b10 : 
                          (MEM_WB_RegWrite === 1 && MEM_WB_RD !== 0 && MEM_WB_RD === ID_EX_RS2)? 2'b01 : 2'b00;


endmodule