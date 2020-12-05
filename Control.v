`define lw 7'b0000011
`define sw 7'b0100011
`define beq 7'b1100011
`define rtype 7'b0110011
`define imm 7'b0010011

module Control(
    input           NoOp,
    input [6 : 0]   opcode,
    output [1 : 0]  AluOp,
    output          AluSrc, RegWrite, MemtoReg, MemRead, MemWrite, Branch
);

assign Branch   = NoOp ? 0 : (opcode == `beq)? 1 : 0;
assign MemRead  = NoOp ? 0 : (opcode == `lw)? 1 : 0;
assign MemWrite = NoOp ? 0 : (opcode == `sw)? 1 : 0;
assign MemtoReg = NoOp ? 0 : (opcode == `lw)? 1 : 0;
assign RegWrite = NoOp ? 0 : ((opcode == `rtype) || (opcode == `lw) || (opcode == `imm))? 1 : 0;
assign AluSrc   = NoOp ? 0 : ((opcode == `rtype) || (opcode == `beq))? 0 : 1;
assign AluOp    = NoOp ? 0 : (opcode == `rtype)? 2 : (opcode == `beq)? 1 : 0;


endmodule