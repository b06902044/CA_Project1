`define ADD 3'b000
`define SUB 3'b001
`define MUL 3'b010
`define AND 3'b011
`define XOR 3'b100
`define SL  3'b101
`define SR  3'b110
`define WRONG 3'b111

module ALU_Control(
    input [2 : 0] func3,
    input [6 : 0] func7,
    input [1 : 0] AluOp,
    output reg [2 : 0] AluControl_o
);

always @ (func3 or func7 or AluOp) begin
    case (AluOp)
        0 : AluControl_o =  (func3 == 3'b101)? `SR : `ADD;
        1 : AluControl_o = `SUB;
        2 : begin
            case (func7)
                7'b0100000 : AluControl_o = `SUB;
                7'b0000001 : AluControl_o = `MUL;
                7'b0000000 : AluControl_o = (func3 == 3'b111) ? `AND : 
                                            (func3 == 3'b100) ? `XOR : 
                                            (func3 == 3'b001) ? `SL : `ADD ;
            endcase
        end
    endcase
end

endmodule
