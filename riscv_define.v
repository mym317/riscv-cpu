`define INST_ADDR_WIDTH 32    // Instruction address width(aka PC width)
`define INST_WIDTH 32         // Instruction width, same as DATA_WIDTH
`define DATA_WIDTH 32       // Data width
`define DOUBLE_DATA_WIDTH 64 // Double data width (for mul/div)
`define REG_ADDR_WIDTH 5    // Register address width (32 logical registers)

`define INST_INIT     `INST_ADDR_WIDTH'h1000 // Initial PC value
`define INST_ADD_STEP 4    // PC increment step for each instruction

`define IF_BATCH_SIZE 2    // Number of instructions fetched in one batch. Check out_inst_{0-1} in IF module

// ALU ops
`define ALU_OP_ADD 4'b0000 // sub can be achieved by add with c_in=1 and rs2 negated (2's complement)
                           // ISA cover: ADD SUB ADDI
`define ALU_OP_CMP 4'b0001 // ISA cover: SLT SLTU SLTI SLTIU
`define ALU_OP_AND 4'b0010 // ISA cover: AND ANDI
`define ALU_OP_OR  4'b0011 // ISA cover: OR ORI
`define ALU_OP_XOR 4'b0100 // ISA cover: XOR XORI
`define ALU_OP_SLL 4'b0101 // ISA cover: SLL SLLI
`define ALU_OP_SRL 4'b0110 // ISA cover: SRL SRLI
`define ALU_OP_SRA 4'b0111 // ISA cover: SRA SRAI

// ALU mul/div ops
`define ALU_OP_MUL 4'b1000    // ISA cover: MUL MULH MULHSU MULHU
`define ALU_OP_DIV 4'b1001    // ISA cover: DIV DIVU REM REMU

// CHANGEME: when add new operations
`define ALU_OP_WIDTH `ALU_OP_DIV+ 1

`define ALU_TYPE_INT 2'b00 // Integer ALU
`define ALU_TYPE_MUL 2'b01 // Integer Multiplier/Divider ALU
`define ALU_TYPE_FO  2'b10 // Floating-point ALU (not implemented yet)
`define ALU_TYPE_MEM 2'b11 // Memory access (not implemented yet)

// Extended decode opcodes beyond integer ALU for other functional units.
`define DECODE_OP_LOAD   `ALU_OP_WIDTH     // memory load
`define DECODE_OP_STORE  `ALU_OP_WIDTH+1   // memory store
`define DECODE_OP_BRANCH `ALU_OP_WIDTH+2   // branch (compare + PC add)
`define DECODE_OP_JUMP   `ALU_OP_WIDTH+3   // jal / jalr
`define DECODE_OP_LUI    `ALU_OP_WIDTH+4   // load upper immediate
`define DECODE_OP_AUIPC  `ALU_OP_WIDTH+5   // add upper immediate to PC
`define DECODE_OP_SYSTEM `ALU_OP_WIDTH+6   // fence / ecall / ebreak / csr
`define DECODE_OP_FP     `ALU_OP_WIDTH+7   // floating-point operation placeholder
`define DECODE_OP_DUMMY  `ALU_OP_WIDTH+8   // unknown / unsupported opcode
`define DECODE_OP_WIDTH  `ALU_OP_WIDTH+9

// ROB parameters
`define ROB_SIZE      32
`define ROB_IDX_WIDTH 5   // log2(ROB_SIZE)

// Physical register file parameters (can exceed architectural count)
`define INT_PREG_NUM   64
`define FP_PREG_NUM    64
`define PREG_IDX_WIDTH 6   // log2(max(INT_PREG_NUM, FP_PREG_NUM))
