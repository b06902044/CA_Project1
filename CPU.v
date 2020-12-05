`define lw 7'b0000011
`define sw 7'b0100011
`define beq 7'b1100011
`define rtype 7'b0110011
`define imm 7'b0010011

module CPU(
    input clk_i, rst_i, start_i
);

// IF stage

wire [31 : 0] pc_i, pc_o;
wire PcWrite;
PC PC(
    .clk_i(clk_i),
    .rst_i(rst_i),
    .start_i(start_i),
    .pc_i(pc_i),
    .pc_o(pc_o),
    .PCWrite_i(PcWrite)
);

wire [31 : 0] pc_add4;
Adder Adder(
    .pc_i(pc_o),
    .pc_o(pc_add4)
);

MUX32 PC_Mux (
    .input0(pc_add4),
    .input1(),
    .ctrl_signal(1'b0),
    .out(pc_i)
);

wire [31: 0] instr;
Instruction_Memory Instruction_Memory
(
    .addr_i(pc_o), 
    .instr_o(instr)
);

// IF / ID pipeline register

reg [31 : 0] IF_ID_IR, IF_ID_PC;

always @(posedge clk_i) begin
    if(Stall == 0) begin
        if(instr) begin
            IF_ID_IR <=  instr;
            IF_ID_PC <= pc_o;
        end     
    end
end

// ID stage

wire [4: 0] rs1_addr, rs2_addr, rd_addr;
wire [6 : 0] opcode;
wire [2 : 0] func3;
wire [6 : 0] func7;
wire [11 : 0] imm_i;

assign opcode = IF_ID_IR[6 : 0];
assign rd_addr = (opcode == `sw || opcode == `beq)? 5'b0 : IF_ID_IR[11 : 7];
assign func3 = IF_ID_IR[14 : 12];
assign rs1_addr = IF_ID_IR[19 : 15];
assign rs2_addr = (opcode == `lw || opcode == `imm)? 5'b0 : IF_ID_IR[24 : 20];
assign func7 =  IF_ID_IR[31 : 25];
assign imm_i =  (opcode == `lw)? IF_ID_IR[31 : 20] : 
                (opcode == `sw)? {IF_ID_IR[31 : 25], IF_ID_IR[11 : 7]} : 
                (opcode == `beq)? {IF_ID_IR[31], IF_ID_IR[7], IF_ID_IR[30 : 25], IF_ID_IR[11 : 8]} : 
                (func3 == 3'b000)?  IF_ID_IR[31 : 20] : IF_ID_IR[24 : 20];

wire Stall, NoOp;
Hazard_Detection Hazard_Detection (
    .rs1_addr(rs1_addr),
    .rs2_addr(rs2_addr),
    .ID_EX_RD(ID_EX_RD),
    .ID_EX_MemRead(ID_EX_MemRead),
    .PcWrite_o(PcWrite), 
    .Stall_o(Stall), 
    .NoOp_o(NoOp) 
);

always @(posedge clk_i) begin
    $display("if_id_ir = %b opcode = %b rd = %d", IF_ID_IR, opcode, rd_addr);
    $display("rs1 = %d rs2 = %d rd = %d memread = %d pcWrite = %d stall = %d noop = %d", 
              rs1_addr, rs2_addr, ID_EX_RD, ID_EX_MemRead, PcWrite, Stall, NoOp);
end


wire [31: 0] rs1_data, rs2_data;
Registers Registers
(
    .clk_i(clk_i),
    .RS1addr_i(rs1_addr),
    .RS2addr_i(rs2_addr),
    .RDaddr_i(MEM_WB_RD), 
    .RDdata_i(wb_data),
    .RegWrite_i(MEM_WB_RegWrite), 
    .RS1data_o(rs1_data), 
    .RS2data_o(rs2_data) 
);

wire [1 : 0] AluOp;
wire AluSrc, RegWrite, MemtoReg, MemRead, MemWrite, Branch;
Control Control(
    .NoOp(NoOp),
    .opcode(opcode),
    .AluOp(AluOp),
    .AluSrc(AluSrc),
    .RegWrite(RegWrite),
    .MemtoReg(MemtoReg),
    .MemRead(MemRead),
    .MemWrite(MemWrite),
    .Branch(Branch)
);

wire [31: 0] imm_o;
Sign_Extend Sign_Extend(
    .imm_i(imm_i),
    .imm_o(imm_o)
);

// ID / EX pipeline register

reg [31 : 0] ID_EX_RS1data, ID_EX_RS2data, ID_EX_IMM, ID_EX_PC;
reg [4 : 0]  ID_EX_RD, ID_EX_RS1, ID_EX_RS2;
reg [6 : 0]  ID_EX_func7;
reg [2 : 0]  ID_EX_func3;
reg [1 : 0]  ID_EX_AluOp;
reg          ID_EX_AluSrc, ID_EX_RegWrite, ID_EX_MemtoReg, ID_EX_MemRead, ID_EX_MemWrite, ID_EX_Branch;


always @(posedge clk_i) begin
    if(rd_addr)     ID_EX_RD <= rd_addr;
    else            ID_EX_RD <= 0;
    if(rs1_addr)    ID_EX_RS1       <= rs1_addr;
    else            ID_EX_RS1       <= 0;
    if(rs2_addr)    ID_EX_RS2       <= rs2_addr;
    else            ID_EX_RS2       <= 0;
    if (rs1_data)   ID_EX_RS1data   <= rs1_data;
    else            ID_EX_RS1data   <= 0;
    if(rs2_data)    ID_EX_RS2data   <= rs2_data;
    else            ID_EX_RS2data   <= 0;
    if(imm_o)       ID_EX_IMM       <= imm_o;
    else            ID_EX_IMM       <= 0;
    if(IF_ID_PC)    ID_EX_PC        <= IF_ID_PC;
    else            ID_EX_PC        <= 0;
    if(func7)       ID_EX_func7     <= func7;
    else            ID_EX_func7     <= 0;
    if(func3)       ID_EX_func3     <= func3;
    else            ID_EX_func3     <= 0;
    if(AluOp)       ID_EX_AluOp     <= AluOp;
    else            ID_EX_AluOp     <= 0;
    if(AluSrc)      ID_EX_AluSrc    <= AluSrc;
    else            ID_EX_AluSrc    <= 0;
    if(RegWrite)    ID_EX_RegWrite  <= RegWrite;
    else            ID_EX_RegWrite  <= 0;
    if(MemtoReg)    ID_EX_MemtoReg  <= MemtoReg;
    else            ID_EX_MemtoReg  <= 0;
    if(MemRead)     ID_EX_MemRead   <= MemRead;
    else            ID_EX_MemRead   <= 0;
    if(MemWrite)    ID_EX_MemWrite  <= MemWrite;
    else            ID_EX_MemWrite  <= 0;
    if(Branch)      ID_EX_Branch    <= Branch;
    else            ID_EX_Branch    <= 0;
end

always @(posedge clk_i) begin
    $display("rd = %d rs1 = %d rs2 = %d rs1data = %d rs2data = %d aluop = %d alusrc = %d regwr = %d memtoreg = %d memread = %d memwrite = %d\n", 
    ID_EX_RD, ID_EX_RS1, ID_EX_RS2 ,ID_EX_RS1data , ID_EX_RS2data  ,ID_EX_AluOp    ,ID_EX_AluSrc   ,ID_EX_RegWrite ,ID_EX_MemtoReg ,ID_EX_MemRead  ,ID_EX_MemWrite);
end

// EX stage

wire [1 : 0] rs1_ctrl, rs2_ctrl;
Forwarding_Unit Forwarding_Unit (
    .ID_EX_RS1(ID_EX_RS1), 
    .ID_EX_RS2(ID_EX_RS2), 
    .EX_MEM_RD(EX_MEM_RD), 
    .MEM_WB_RD(MEM_WB_RD),
    .EX_MEM_RegWrite(EX_MEM_RegWrite), 
    .MEM_WB_RegWrite(MEM_WB_RegWrite),
    .Forward_Control1(rs1_ctrl), 
    .Forward_Control2(rs2_ctrl) 
);

wire [31 : 0] ALU_RS1, ALU_RS2;
MUX_Forwarding RS1_Forwarding (
    .ID_EX_RS(ID_EX_RS1data), 
    .EX_MEM_ALUout(EX_MEM_ALUout), 
    .WB_WBdata(wb_data),
    .ctrl(rs1_ctrl),
    .forword_out(ALU_RS1)
);
MUX_Forwarding RS2_Forwarding (
    .ID_EX_RS(ID_EX_RS2data), 
    .EX_MEM_ALUout(EX_MEM_ALUout), 
    .WB_WBdata(wb_data),
    .ctrl(rs2_ctrl),
    .forword_out(ALU_RS2)
);

wire [31 : 0] mux_o;
MUX32 ALU_Src_Mux (
    .input0(ALU_RS2),
    .input1(ID_EX_IMM),
    .ctrl_signal(ID_EX_AluSrc),
    .out(mux_o)
);

wire [2 : 0] AluControl_o;
ALU_Control ALU_Control(
    .func3(ID_EX_func3),
    .func7(ID_EX_func7),
    .AluOp(ID_EX_AluOp),
    .AluControl_o(AluControl_o)
);

wire [31 : 0] ALU_o;
ALU ALU(
    .rs1_data(ALU_RS1), 
    .mux_o(mux_o),
    .AluControl(AluControl_o),
    .ALU_o(ALU_o)
);

// EX / MEM pipeline register

reg [31 : 0] EX_MEM_ALUout, EX_MEM_WriteData;
reg [4 : 0]  EX_MEM_RD;
reg          EX_MEM_RegWrite, EX_MEM_MemtoReg, EX_MEM_MemRead, EX_MEM_MemWrite, EX_MEM_Branch;

always @ (posedge clk_i) begin
    if(ALU_o)           EX_MEM_ALUout       <= ALU_o;
    else                EX_MEM_ALUout       <= 0;
    if(ALU_RS2)         EX_MEM_WriteData    <= ALU_RS2;
    else                EX_MEM_WriteData    <= 0;
    if(ID_EX_RD)        EX_MEM_RD           <= ID_EX_RD;
    else                EX_MEM_RD           <= 0;
    if(ID_EX_RegWrite)  EX_MEM_RegWrite     <= ID_EX_RegWrite;
    else                EX_MEM_RegWrite     <= 0;
    if(ID_EX_MemtoReg)  EX_MEM_MemtoReg     <= ID_EX_MemtoReg;
    else                EX_MEM_MemtoReg     <= 0;
    if(ID_EX_MemRead)    EX_MEM_MemRead      <= ID_EX_MemRead;
    else                EX_MEM_MemRead      <= 0;
    if(ID_EX_MemWrite)  EX_MEM_MemWrite     <= ID_EX_MemWrite;
    else                EX_MEM_MemWrite     <= 0;
    if(ID_EX_Branch)    EX_MEM_Branch       <= ID_EX_Branch;
    else                EX_MEM_Branch       <= 0;
end


//MEM stage

wire [31 : 0] MemData_o;
Data_Memory Data_Memory(
    .clk_i(clk_i), 
    .addr_i(EX_MEM_ALUout), 
    .MemRead_i(EX_MEM_MemRead),
    .MemWrite_i(EX_MEM_MemWrite),
    .data_i(EX_MEM_WriteData),
    .data_o(MemData_o)
);

// MEM / WB pipeline register 

reg [31 : 0] MEM_WB_MEMout, MEM_WB_ALUout;
reg [4 : 0]  MEM_WB_RD;
reg          MEM_WB_MemtoReg, MEM_WB_RegWrite;

always @ (posedge clk_i) begin
    if(MemData_o)       MEM_WB_MEMout   <= MemData_o;
    else                MEM_WB_MEMout   <= 0;
    if(EX_MEM_ALUout)   MEM_WB_ALUout   <= EX_MEM_ALUout;
    else                MEM_WB_ALUout   <= 0;
    if(EX_MEM_RD)       MEM_WB_RD       <= EX_MEM_RD;
    else                MEM_WB_RD       <= 0;
    if(EX_MEM_RegWrite) MEM_WB_RegWrite <= EX_MEM_RegWrite;
    else                MEM_WB_RegWrite <= 0;
    if(EX_MEM_MemtoReg) MEM_WB_MemtoReg <= EX_MEM_MemtoReg;
    else                MEM_WB_MemtoReg <= 0;
end

//WB stage

wire [31 : 0] wb_data;
MUX32 WB_Mux(
    .input0(MEM_WB_ALUout),
    .input1(MEM_WB_MEMout),
    .ctrl_signal(MEM_WB_MemtoReg),
    .out(wb_data)
);

always @ (posedge clk_i) begin
    //$display("cpu regwrite = %b aluout = %d memwr = %b memwrdata = %d\n", EX_MEM_RegWrite, EX_MEM_ALUout, EX_MEM_MemWrite, EX_MEM_WriteData);
end


endmodule
