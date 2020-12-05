module Hazard_Detection (
    input [4 : 0] rs1_addr, rs2_addr, ID_EX_RD,
    input         ID_EX_MemRead,
    output        PcWrite_o, Stall_o, NoOp_o 
);

assign PcWrite_o =  (ID_EX_MemRead == 0)? 1 : ((rs1_addr == ID_EX_RD ||  rs2_addr == ID_EX_RD))? 0 : 1;
assign Stall_o =    (ID_EX_MemRead == 0)? 0 : ((rs1_addr == ID_EX_RD ||  rs2_addr == ID_EX_RD))? 1 : 0;
assign NoOp_o =     (ID_EX_MemRead == 0)? 0 : ((rs1_addr == ID_EX_RD ||  rs2_addr == ID_EX_RD))? 1 : 0;

endmodule