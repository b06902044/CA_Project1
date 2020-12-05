module Hazard_Detection (
    input [4 : 0] rs1_addr, rs2_addr, ID_EX_RD,
    input         ID_EX_MemRead,
    output  reg   PcWrite_o, Stall_o, NoOp_o 
);

//assign PcWrite_o = (ID_EX_MemRead == 0)? 1 : ((rs1_addr == ID_EX_RD ||  rs2_addr == ID_EX_RD))? 0 : 1;
//assign Stall_o = (ID_EX_MemRead == 0)? 0 : ((rs1_addr == ID_EX_RD ||  rs2_addr == ID_EX_RD))? 1 : 0;
//assign NoOp_o = (ID_EX_MemRead == 0)? 0 : ((rs1_addr == ID_EX_RD ||  rs2_addr == ID_EX_RD))? 1 : 0;

always @(rs1_addr or rs2_addr or ID_EX_RD or ID_EX_MemRead) begin
    if(ID_EX_MemRead == 0) begin
        PcWrite_o = 1;
        Stall_o = 0;
        NoOp_o = 0;
    end
    else if(rs1_addr == ID_EX_RD ||  rs2_addr == ID_EX_RD) begin
        PcWrite_o = 0;
        Stall_o = 1;
        NoOp_o = 1;
    end
    else begin
        PcWrite_o = 1;
        Stall_o = 0;
        NoOp_o = 0;
    end
end


endmodule