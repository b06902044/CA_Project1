`define ADD 3'b000
`define SUB 3'b001
`define MUL 3'b010
`define AND 3'b011
`define XOR 3'b100
`define SL  3'b101
`define SR  3'b110
`define WRONG 3'b111

module ALU(
    input signed [31 : 0] rs1_data, mux_o,
    input [2 : 0] AluControl,
    output reg [31 : 0] ALU_o
);

always @(*) begin
    case(AluControl)
        `ADD: ALU_o = rs1_data + mux_o;
        `SUB: ALU_o = rs1_data - mux_o;
        `MUL: ALU_o = rs1_data * mux_o;
        `AND: ALU_o = rs1_data & mux_o;
        `XOR: ALU_o = rs1_data ^ mux_o;
        `SL: ALU_o = rs1_data << mux_o;
        `SR: ALU_o = rs1_data >>> mux_o;
    endcase
end


endmodule
