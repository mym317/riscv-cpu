`include "riscv_define.v"

// Simple top-level that stitches IF -> PreDecode -> RegRename -> PostDecode.
// IF currently only produces PCs/valid; instructions are stubbed as NOPs.
module Top (
    input clk,
    input rst_n
);

// ----------------------------
// IF stage (PC generation)
// ----------------------------
wire [`INST_WIDTH-1:0] if_inst_addr_0;
wire [`INST_WIDTH-1:0] if_inst_addr_1;
wire [`IF_BATCH_SIZE-1:0] if_inst_valid;

IF u_if (
    .clk(clk),
    .rst_n(rst_n),
    .out_inst_addr_0(if_inst_addr_0),
    .out_inst_addr_1(if_inst_addr_1),
    .out_inst_valid(if_inst_valid)
);

// ----------------------------
// Instruction fetch stub (NOPs)
// ----------------------------
wire [`INST_WIDTH-1:0] fetch_inst_0 = 32'h0000_0013; // ADDI x0, x0, 0
wire [`INST_WIDTH-1:0] fetch_inst_1 = 32'h0000_0013;

// ----------------------------
// Pre-decode
// ----------------------------
wire [`IF_BATCH_SIZE-1:0] pre_valid;
wire [1:0] pre_fu_type_0;
wire [1:0] pre_fu_type_1;
wire [`REG_ADDR_WIDTH-1:0] pre_rs1_0;
wire [`REG_ADDR_WIDTH-1:0] pre_rs2_0;
wire [`REG_ADDR_WIDTH-1:0] pre_rd_0;
wire [`DATA_WIDTH-1:0] pre_imm_0;
wire pre_use_imm_0;
wire pre_rs1_is_fp_0;
wire pre_rs2_is_fp_0;
wire pre_rd_is_fp_0;

wire [`REG_ADDR_WIDTH-1:0] pre_rs1_1;
wire [`REG_ADDR_WIDTH-1:0] pre_rs2_1;
wire [`REG_ADDR_WIDTH-1:0] pre_rd_1;
wire [`DATA_WIDTH-1:0] pre_imm_1;
wire pre_use_imm_1;
wire pre_rs1_is_fp_1;
wire pre_rs2_is_fp_1;
wire pre_rd_is_fp_1;

PreDecode u_predecode (
    .clk(clk),
    .rst_n(rst_n),
    .in_inst_0(fetch_inst_0),
    .in_inst_1(fetch_inst_1),
    .in_inst_valid(if_inst_valid),
    .out_inst_valid(pre_valid),
    .out_fu_type_0(pre_fu_type_0),
    .out_rs1_0(pre_rs1_0),
    .out_rs2_0(pre_rs2_0),
    .out_rd_0(pre_rd_0),
    .out_imm_0(pre_imm_0),
    .out_use_imm_0(pre_use_imm_0),
    .out_rs1_is_fp_0(pre_rs1_is_fp_0),
    .out_rs2_is_fp_0(pre_rs2_is_fp_0),
    .out_rd_is_fp_0(pre_rd_is_fp_0),
    .out_fu_type_1(pre_fu_type_1),
    .out_rs1_1(pre_rs1_1),
    .out_rs2_1(pre_rs2_1),
    .out_rd_1(pre_rd_1),
    .out_imm_1(pre_imm_1),
    .out_use_imm_1(pre_use_imm_1),
    .out_rs1_is_fp_1(pre_rs1_is_fp_1),
    .out_rs2_is_fp_1(pre_rs2_is_fp_1),
    .out_rd_is_fp_1(pre_rd_is_fp_1)
);

// ----------------------------
// Register rename
// ----------------------------
wire [`IF_BATCH_SIZE-1:0] rn_valid;
wire [`INST_WIDTH-1:0] rn_inst_0;
wire [`INST_WIDTH-1:0] rn_inst_1;
wire [1:0] rn_fu_type_0;
wire [1:0] rn_fu_type_1;
wire [`REG_ADDR_WIDTH-1:0] rn_rs1_0;
wire [`REG_ADDR_WIDTH-1:0] rn_rs2_0;
wire [`REG_ADDR_WIDTH-1:0] rn_rd_0;
wire [`DATA_WIDTH-1:0] rn_imm_0;
wire rn_use_imm_0;
wire rn_rs1_is_fp_0;
wire rn_rs2_is_fp_0;
wire rn_rd_is_fp_0;
wire [`PREG_IDX_WIDTH-1:0] rn_rs1_preg_0;
wire rn_rs1_preg_valid_0;
wire [`PREG_IDX_WIDTH-1:0] rn_rs2_preg_0;
wire rn_rs2_preg_valid_0;
wire [`PREG_IDX_WIDTH-1:0] rn_rd_preg_0;
wire rn_rd_preg_valid_0;
wire [`PREG_IDX_WIDTH-1:0] rn_old_rd_preg_0;
wire rn_old_rd_preg_valid_0;

wire [`REG_ADDR_WIDTH-1:0] rn_rs1_1;
wire [`REG_ADDR_WIDTH-1:0] rn_rs2_1;
wire [`REG_ADDR_WIDTH-1:0] rn_rd_1;
wire [`DATA_WIDTH-1:0] rn_imm_1;
wire rn_use_imm_1;
wire rn_rs1_is_fp_1;
wire rn_rs2_is_fp_1;
wire rn_rd_is_fp_1;
wire [`PREG_IDX_WIDTH-1:0] rn_rs1_preg_1;
wire rn_rs1_preg_valid_1;
wire [`PREG_IDX_WIDTH-1:0] rn_rs2_preg_1;
wire rn_rs2_preg_valid_1;
wire [`PREG_IDX_WIDTH-1:0] rn_rd_preg_1;
wire rn_rd_preg_valid_1;
wire [`PREG_IDX_WIDTH-1:0] rn_old_rd_preg_1;
wire rn_old_rd_preg_valid_1;

// Commit/ROB visibility
wire commit0_valid;
wire commit1_valid;
wire [`ROB_IDX_WIDTH-1:0] commit0_rob_idx;
wire [`ROB_IDX_WIDTH-1:0] commit1_rob_idx;
wire [`INST_ADDR_WIDTH-1:0] commit0_pc;
wire [`INST_ADDR_WIDTH-1:0] commit1_pc;
wire commit0_has_dest;
wire commit1_has_dest;
wire commit0_is_float;
wire commit1_is_float;
wire [`REG_ADDR_WIDTH-1:0] commit0_arch_rd;
wire [`REG_ADDR_WIDTH-1:0] commit1_arch_rd;
wire [`PREG_IDX_WIDTH-1:0] commit0_new_preg;
wire [`PREG_IDX_WIDTH-1:0] commit1_new_preg;
wire [`PREG_IDX_WIDTH-1:0] commit0_old_preg;
wire [`PREG_IDX_WIDTH-1:0] commit1_old_preg;
wire [`DATA_WIDTH-1:0] commit0_value;
wire [`DATA_WIDTH-1:0] commit1_value;
wire commit0_exception;
wire commit1_exception;
wire rob_empty;
wire rn_stall;

RegRename u_regrename (
    .clk(clk),
    .rst_n(rst_n),
    .flush(1'b0),
    .in_inst_valid(pre_valid),
    .in_pc_0(if_inst_addr_0),
    .in_pc_1(if_inst_addr_1),
    .in_inst_0(fetch_inst_0),
    .in_inst_1(fetch_inst_1),
    .in_fu_type_0(pre_fu_type_0),
    .in_fu_type_1(pre_fu_type_1),
    .in_rs1_0(pre_rs1_0),
    .in_rs2_0(pre_rs2_0),
    .in_rd_0(pre_rd_0),
    .in_imm_0(pre_imm_0),
    .in_use_imm_0(pre_use_imm_0),
    .in_rs1_is_fp_0(pre_rs1_is_fp_0),
    .in_rs2_is_fp_0(pre_rs2_is_fp_0),
    .in_rd_is_fp_0(pre_rd_is_fp_0),
    .in_rs1_1(pre_rs1_1),
    .in_rs2_1(pre_rs2_1),
    .in_rd_1(pre_rd_1),
    .in_imm_1(pre_imm_1),
    .in_use_imm_1(pre_use_imm_1),
    .in_rs1_is_fp_1(pre_rs1_is_fp_1),
    .in_rs2_is_fp_1(pre_rs2_is_fp_1),
    .in_rd_is_fp_1(pre_rd_is_fp_1),
    .wb0_valid(1'b0),
    .wb0_rob_idx({`ROB_IDX_WIDTH{1'b0}}),
    .wb0_value({`DATA_WIDTH{1'b0}}),
    .wb0_exception(1'b0),
    .wb1_valid(1'b0),
    .wb1_rob_idx({`ROB_IDX_WIDTH{1'b0}}),
    .wb1_value({`DATA_WIDTH{1'b0}}),
    .wb1_exception(1'b0),
    .out_inst_valid(rn_valid),
    .out_inst_0(rn_inst_0),
    .out_inst_1(rn_inst_1),
    .out_fu_type_0(rn_fu_type_0),
    .out_fu_type_1(rn_fu_type_1),
    .out_rs1_0(rn_rs1_0),
    .out_rs2_0(rn_rs2_0),
    .out_rd_0(rn_rd_0),
    .out_imm_0(rn_imm_0),
    .out_use_imm_0(rn_use_imm_0),
    .out_rs1_is_fp_0(rn_rs1_is_fp_0),
    .out_rs2_is_fp_0(rn_rs2_is_fp_0),
    .out_rd_is_fp_0(rn_rd_is_fp_0),
    .out_rs1_preg_0(rn_rs1_preg_0),
    .out_rs1_preg_valid_0(rn_rs1_preg_valid_0),
    .out_rs2_preg_0(rn_rs2_preg_0),
    .out_rs2_preg_valid_0(rn_rs2_preg_valid_0),
    .out_rd_preg_0(rn_rd_preg_0),
    .out_rd_preg_valid_0(rn_rd_preg_valid_0),
    .out_old_rd_preg_0(rn_old_rd_preg_0),
    .out_old_rd_preg_valid_0(rn_old_rd_preg_valid_0),
    .out_rs1_1(rn_rs1_1),
    .out_rs2_1(rn_rs2_1),
    .out_rd_1(rn_rd_1),
    .out_imm_1(rn_imm_1),
    .out_use_imm_1(rn_use_imm_1),
    .out_rs1_is_fp_1(rn_rs1_is_fp_1),
    .out_rs2_is_fp_1(rn_rs2_is_fp_1),
    .out_rd_is_fp_1(rn_rd_is_fp_1),
    .out_rs1_preg_1(rn_rs1_preg_1),
    .out_rs1_preg_valid_1(rn_rs1_preg_valid_1),
    .out_rs2_preg_1(rn_rs2_preg_1),
    .out_rs2_preg_valid_1(rn_rs2_preg_valid_1),
    .out_rd_preg_1(rn_rd_preg_1),
    .out_rd_preg_valid_1(rn_rd_preg_valid_1),
    .out_old_rd_preg_1(rn_old_rd_preg_1),
    .out_old_rd_preg_valid_1(rn_old_rd_preg_valid_1),
    .commit0_valid(commit0_valid),
    .commit0_rob_idx(commit0_rob_idx),
    .commit0_pc(commit0_pc),
    .commit0_has_dest(commit0_has_dest),
    .commit0_is_float(commit0_is_float),
    .commit0_arch_rd(commit0_arch_rd),
    .commit0_new_preg(commit0_new_preg),
    .commit0_old_preg(commit0_old_preg),
    .commit0_value(commit0_value),
    .commit0_exception(commit0_exception),
    .commit1_valid(commit1_valid),
    .commit1_rob_idx(commit1_rob_idx),
    .commit1_pc(commit1_pc),
    .commit1_has_dest(commit1_has_dest),
    .commit1_is_float(commit1_is_float),
    .commit1_arch_rd(commit1_arch_rd),
    .commit1_new_preg(commit1_new_preg),
    .commit1_old_preg(commit1_old_preg),
    .commit1_value(commit1_value),
    .commit1_exception(commit1_exception),
    .rob_empty(rob_empty),
    .rename_stall(rn_stall)
);

// ----------------------------
// Post-decode (detailed micro-op)
// ----------------------------
wire [`IF_BATCH_SIZE-1:0] post_valid;
wire [`INST_WIDTH-1:0] post_inst_0;
wire [`INST_WIDTH-1:0] post_inst_1;
wire [1:0] post_fu_type_0;
wire [1:0] post_fu_type_1;
wire [`REG_ADDR_WIDTH-1:0] post_rs1_0;
wire [`REG_ADDR_WIDTH-1:0] post_rs2_0;
wire [`REG_ADDR_WIDTH-1:0] post_rd_0;
wire [`DATA_WIDTH-1:0] post_imm_0;
wire post_use_imm_0;
wire post_rs1_is_fp_0;
wire post_rs2_is_fp_0;
wire post_rd_is_fp_0;
wire [`PREG_IDX_WIDTH-1:0] post_rs1_preg_0;
wire post_rs1_preg_valid_0;
wire [`PREG_IDX_WIDTH-1:0] post_rs2_preg_0;
wire post_rs2_preg_valid_0;
wire [`PREG_IDX_WIDTH-1:0] post_rd_preg_0;
wire post_rd_preg_valid_0;
wire [`PREG_IDX_WIDTH-1:0] post_old_rd_preg_0;
wire post_old_rd_preg_valid_0;

wire [`REG_ADDR_WIDTH-1:0] post_rs1_1;
wire [`REG_ADDR_WIDTH-1:0] post_rs2_1;
wire [`REG_ADDR_WIDTH-1:0] post_rd_1;
wire [`DATA_WIDTH-1:0] post_imm_1;
wire post_use_imm_1;
wire post_rs1_is_fp_1;
wire post_rs2_is_fp_1;
wire post_rd_is_fp_1;
wire [`PREG_IDX_WIDTH-1:0] post_rs1_preg_1;
wire post_rs1_preg_valid_1;
wire [`PREG_IDX_WIDTH-1:0] post_rs2_preg_1;
wire post_rs2_preg_valid_1;
wire [`PREG_IDX_WIDTH-1:0] post_rd_preg_1;
wire post_rd_preg_valid_1;
wire [`PREG_IDX_WIDTH-1:0] post_old_rd_preg_1;
wire post_old_rd_preg_valid_1;

wire [`FU_DEC_WIDTH-1:0] post_fu_sel_0;
wire [`FU_DEC_WIDTH-1:0] post_fu_sel_1;
wire [`ALU_OP_WIDTH-1:0] post_int_op_0;
wire [`ALU_OP_WIDTH-1:0] post_int_op_1;
wire post_int_is_sub_0;
wire post_int_is_sub_1;
wire post_cmp_signed_0;
wire post_cmp_signed_1;
wire [`ALU_OP_WIDTH-1:0] post_muldiv_op_0;
wire [`ALU_OP_WIDTH-1:0] post_muldiv_op_1;
wire post_mul_high_0;
wire post_mul_high_1;
wire post_mul_signed_rs1_0;
wire post_mul_signed_rs1_1;
wire post_mul_signed_rs2_0;
wire post_mul_signed_rs2_1;
wire post_div_signed_0;
wire post_div_signed_1;
wire post_div_is_rem_0;
wire post_div_is_rem_1;
wire [`BR_OP_WIDTH-1:0] post_branch_op_0;
wire [`BR_OP_WIDTH-1:0] post_branch_op_1;
wire [`MEM_OP_WIDTH-1:0] post_mem_op_0;
wire [`MEM_OP_WIDTH-1:0] post_mem_op_1;
wire post_mem_is_load_0;
wire post_mem_is_load_1;
wire post_mem_unsigned_0;
wire post_mem_unsigned_1;
wire [`CSR_OP_WIDTH-1:0] post_csr_op_0;
wire [`CSR_OP_WIDTH-1:0] post_csr_op_1;
wire [11:0] post_csr_addr_0;
wire [11:0] post_csr_addr_1;
wire [`FP_OP_WIDTH-1:0] post_fp_op_0;
wire [`FP_OP_WIDTH-1:0] post_fp_op_1;
wire post_illegal_0;
wire post_illegal_1;

PostDecode u_postdecode (
    .clk(clk),
    .rst_n(rst_n),
    .in_inst_valid(rn_valid),
    .in_inst_0(rn_inst_0),
    .in_inst_1(rn_inst_1),
    .in_fu_type_0(rn_fu_type_0),
    .in_fu_type_1(rn_fu_type_1),
    .in_rs1_0(rn_rs1_0),
    .in_rs2_0(rn_rs2_0),
    .in_rd_0(rn_rd_0),
    .in_imm_0(rn_imm_0),
    .in_use_imm_0(rn_use_imm_0),
    .in_rs1_is_fp_0(rn_rs1_is_fp_0),
    .in_rs2_is_fp_0(rn_rs2_is_fp_0),
    .in_rd_is_fp_0(rn_rd_is_fp_0),
    .in_rs1_preg_0(rn_rs1_preg_0),
    .in_rs1_preg_valid_0(rn_rs1_preg_valid_0),
    .in_rs2_preg_0(rn_rs2_preg_0),
    .in_rs2_preg_valid_0(rn_rs2_preg_valid_0),
    .in_rd_preg_0(rn_rd_preg_0),
    .in_rd_preg_valid_0(rn_rd_preg_valid_0),
    .in_old_rd_preg_0(rn_old_rd_preg_0),
    .in_old_rd_preg_valid_0(rn_old_rd_preg_valid_0),
    .in_rs1_1(rn_rs1_1),
    .in_rs2_1(rn_rs2_1),
    .in_rd_1(rn_rd_1),
    .in_imm_1(rn_imm_1),
    .in_use_imm_1(rn_use_imm_1),
    .in_rs1_is_fp_1(rn_rs1_is_fp_1),
    .in_rs2_is_fp_1(rn_rs2_is_fp_1),
    .in_rd_is_fp_1(rn_rd_is_fp_1),
    .in_rs1_preg_1(rn_rs1_preg_1),
    .in_rs1_preg_valid_1(rn_rs1_preg_valid_1),
    .in_rs2_preg_1(rn_rs2_preg_1),
    .in_rs2_preg_valid_1(rn_rs2_preg_valid_1),
    .in_rd_preg_1(rn_rd_preg_1),
    .in_rd_preg_valid_1(rn_rd_preg_valid_1),
    .in_old_rd_preg_1(rn_old_rd_preg_1),
    .in_old_rd_preg_valid_1(rn_old_rd_preg_valid_1),
    .out_inst_valid(post_valid),
    .out_inst_0(post_inst_0),
    .out_inst_1(post_inst_1),
    .out_fu_type_0(post_fu_type_0),
    .out_fu_type_1(post_fu_type_1),
    .out_rs1_0(post_rs1_0),
    .out_rs2_0(post_rs2_0),
    .out_rd_0(post_rd_0),
    .out_imm_0(post_imm_0),
    .out_use_imm_0(post_use_imm_0),
    .out_rs1_is_fp_0(post_rs1_is_fp_0),
    .out_rs2_is_fp_0(post_rs2_is_fp_0),
    .out_rd_is_fp_0(post_rd_is_fp_0),
    .out_rs1_preg_0(post_rs1_preg_0),
    .out_rs1_preg_valid_0(post_rs1_preg_valid_0),
    .out_rs2_preg_0(post_rs2_preg_0),
    .out_rs2_preg_valid_0(post_rs2_preg_valid_0),
    .out_rd_preg_0(post_rd_preg_0),
    .out_rd_preg_valid_0(post_rd_preg_valid_0),
    .out_old_rd_preg_0(post_old_rd_preg_0),
    .out_old_rd_preg_valid_0(post_old_rd_preg_valid_0),
    .out_rs1_1(post_rs1_1),
    .out_rs2_1(post_rs2_1),
    .out_rd_1(post_rd_1),
    .out_imm_1(post_imm_1),
    .out_use_imm_1(post_use_imm_1),
    .out_rs1_is_fp_1(post_rs1_is_fp_1),
    .out_rs2_is_fp_1(post_rs2_is_fp_1),
    .out_rd_is_fp_1(post_rd_is_fp_1),
    .out_rs1_preg_1(post_rs1_preg_1),
    .out_rs1_preg_valid_1(post_rs1_preg_valid_1),
    .out_rs2_preg_1(post_rs2_preg_1),
    .out_rs2_preg_valid_1(post_rs2_preg_valid_1),
    .out_rd_preg_1(post_rd_preg_1),
    .out_rd_preg_valid_1(post_rd_preg_valid_1),
    .out_old_rd_preg_1(post_old_rd_preg_1),
    .out_old_rd_preg_valid_1(post_old_rd_preg_valid_1),
    .out_fu_sel_0(post_fu_sel_0),
    .out_fu_sel_1(post_fu_sel_1),
    .out_int_op_0(post_int_op_0),
    .out_int_op_1(post_int_op_1),
    .out_int_is_sub_0(post_int_is_sub_0),
    .out_int_is_sub_1(post_int_is_sub_1),
    .out_cmp_signed_0(post_cmp_signed_0),
    .out_cmp_signed_1(post_cmp_signed_1),
    .out_muldiv_op_0(post_muldiv_op_0),
    .out_muldiv_op_1(post_muldiv_op_1),
    .out_mul_high_0(post_mul_high_0),
    .out_mul_high_1(post_mul_high_1),
    .out_mul_signed_rs1_0(post_mul_signed_rs1_0),
    .out_mul_signed_rs1_1(post_mul_signed_rs1_1),
    .out_mul_signed_rs2_0(post_mul_signed_rs2_0),
    .out_mul_signed_rs2_1(post_mul_signed_rs2_1),
    .out_div_signed_0(post_div_signed_0),
    .out_div_signed_1(post_div_signed_1),
    .out_div_is_rem_0(post_div_is_rem_0),
    .out_div_is_rem_1(post_div_is_rem_1),
    .out_branch_op_0(post_branch_op_0),
    .out_branch_op_1(post_branch_op_1),
    .out_mem_op_0(post_mem_op_0),
    .out_mem_op_1(post_mem_op_1),
    .out_mem_is_load_0(post_mem_is_load_0),
    .out_mem_is_load_1(post_mem_is_load_1),
    .out_mem_unsigned_0(post_mem_unsigned_0),
    .out_mem_unsigned_1(post_mem_unsigned_1),
    .out_csr_op_0(post_csr_op_0),
    .out_csr_op_1(post_csr_op_1),
    .out_csr_addr_0(post_csr_addr_0),
    .out_csr_addr_1(post_csr_addr_1),
    .out_fp_op_0(post_fp_op_0),
    .out_fp_op_1(post_fp_op_1),
    .out_illegal_0(post_illegal_0),
    .out_illegal_1(post_illegal_1)
);

// Downstream execution/issue logic would consume post_* signals.

endmodule
