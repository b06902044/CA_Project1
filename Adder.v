module Adder (
    input [31 : 0] IF_ID_PC, IF_ID_Imm,
    output [31 : 0] PC_Branch
);

assign PC_Branch = IF_ID_PC +  (IF_ID_Imm << 1);

endmodule