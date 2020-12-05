module Forwarding_Unit(
    input [4 : 0]  ID_EX_RS1, ID_EX_RS2, EX_MEM_RD, MEM_WB_RD,
    input          EX_MEM_RegWrite, MEM_WB_RegWrite,
    output [1 : 0] Forward_Control1, Forward_Control2 
);

assign Forward_Control1 = (EX_MEM_RegWrite === 1 && EX_MEM_RD !== 0 && EX_MEM_RD === ID_EX_RS1)? 2'b10 : 
                          (MEM_WB_RegWrite === 1 && MEM_WB_RD !== 0 && MEM_WB_RD === ID_EX_RS1)? 2'b01 : 2'b00;
assign Forward_Control2 = (EX_MEM_RegWrite === 1 && EX_MEM_RD !== 0 && EX_MEM_RD === ID_EX_RS2)? 2'b10 : 
                          (MEM_WB_RegWrite === 1 && MEM_WB_RD !== 0 && MEM_WB_RD === ID_EX_RS2)? 2'b01 : 2'b00;

always @(ID_EX_RS1 or ID_EX_RS2 or EX_MEM_RD or MEM_WB_RD or EX_MEM_RegWrite or MEM_WB_RegWrite or Forward_Control1 or Forward_Control2 )  begin
    //$display("rs1 = %d rs2 = %d ex_mem = %d mem_wb = %d ex_mem_reg = %b mem_wb_reg = %b ctrl1 = %b ctrl2 = %b\n", ID_EX_RS1, ID_EX_RS2, EX_MEM_RD, MEM_WB_RD, EX_MEM_RegWrite, MEM_WB_RegWrite, Forward_Control1, Forward_Control2);
end

endmodule