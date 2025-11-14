//----------------------------------------------------------------+
// Project: Capstone Design
// Module : tile_loader (SystemVerilog module)
// Description:
//   Loads A (persist) and B (block ping-pong) from DDR via embedded
//   axi_addr_gen + dma_read. Style follows tile_store pattern.
//   - A bulk: N×K elements (row-major)
//   - B block: K×BLOCK_M elements (row-major, per j_block)
//
// Last Updated: 2025-11-12 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns/1ps

import sa_params_pkg::*;
`include "sa_defs.svh"

module tile_loader #(
  parameter int TILE_SIZE      = 8,
  parameter int DATA_W         = 32,
  parameter int AXI_USER_WIDTH = 1,
  parameter int A_ADDR_W       = 14,  // 12,288 words (64×768 INT8 / 4)
  parameter int B_ADDR_W       = 16   // 49,152 words (768×256 INT8 / 4)
)(
  input  logic                         clk,
  input  logic                         rstn,

  // ===== Control from orchestrator =====
  input  logic                         load_req,       // pulse: start load
  input  logic                         update_a,       // 1=A bulk, 0=B block
  input  logic [15:0]                  N, K, M,
  input  logic [15:0]                  BLOCK_M,
  input  logic [15:0]                  j_block,        // B block column offset

  // ===== Memory layout (byte addresses/strides) =====
  input  logic [AXI_ADDR_WIDTH-1:0]    base_a_addr,
  input  logic [AXI_ADDR_WIDTH-1:0]    base_b_addr,
  input  logic [31:0]                  stride_a_row_bytes,  // K bytes
  input  logic [31:0]                  stride_b_row_bytes,  // M bytes

  // ===== Status =====
  output logic                         ld_busy,
  output logic                         ld_done,        // pulse when complete

  // ===== A buffer fill interface (dpram_wrapper) =====
  output logic                         a_fill_we,
  output logic [A_ADDR_W-1:0]          a_fill_addr,
  output logic [DATA_W-1:0]            a_fill_wdata,

  // ===== B buffer fill interface (bram_pingpong) =====
  output logic [31:0]                  b_seg_words,
  output logic                         b_fill_req,     // pulse
  input  logic                         b_fill_busy,
  output logic                         b_fill_we,
  output logic [B_ADDR_W-1:0]          b_fill_addr,
  output logic [DATA_W-1:0]            b_fill_wdata,
  output logic                         b_fill_done,    // pulse

  // ===== AXI4 Master Read (connect upward) =====
  output logic                         M_AXI_ARVALID,
  input  logic                         M_AXI_ARREADY,
  output logic [AXI_ADDR_WIDTH-1:0]    M_AXI_ARADDR,
  output logic [3:0]                   M_AXI_ARID,
  output logic [7:0]                   M_AXI_ARLEN,
  output logic [2:0]                   M_AXI_ARSIZE,
  output logic [1:0]                   M_AXI_ARBURST,
  output logic                         M_AXI_ARLOCK,
  output logic [3:0]                   M_AXI_ARCACHE,
  output logic [2:0]                   M_AXI_ARPROT,
  output logic [3:0]                   M_AXI_ARQOS,
  output logic [AXI_USER_WIDTH-1:0]    M_AXI_ARUSER,

  input  logic                         M_AXI_RVALID,
  output logic                         M_AXI_RREADY,
  input  logic [DATA_W-1:0]            M_AXI_RDATA,
  input  logic [1:0]                   M_AXI_RRESP,
  input  logic                         M_AXI_RLAST,
  input  logic [3:0]                   M_AXI_RID,
  input  logic [AXI_USER_WIDTH-1:0]    M_AXI_RUSER
);

  // Control FSM
  typedef enum logic [2:0] { 
    S_IDLE, S_A_BULK, S_B_REQ, S_B_PREP, S_B_ROW, S_B_WAIT, S_DONE 
  } state_e;
  state_e state;

  // Load parameters
  logic [31:0] total_words;      // A bulk or B block total
  logic [31:0] word_cnt;         // current word counter
  logic [31:0] row_cnt;          // B block row counter
  logic [31:0] row_words;        // words per B row
  logic [31:0] cur_block_m;      // effective BLOCK_M for this j_block
  logic [15:0] k_q;              // registered K for B path pipelining

  logic [31:0] col_cnt;          // column index within current B row (for B block) (0 ... row_words-1)
  logic [AXI_ADDR_WIDTH-1:0] row_base_addr;

  // Address generator (READ)
  logic                      ag_start, ag_done;
  logic                      ag_req_valid, ag_req_ready, ag_req_last;
  logic [AXI_ADDR_WIDTH-1:0] ag_req_addr;
  logic [7:0]                ag_req_len;
  logic [AXI_ADDR_WIDTH-1:0] ag_base_addr;
  logic [31:0]               ag_bytes_total;

  axi_addr_gen #(
    .ADDR_W          (AXI_ADDR_WIDTH),
    .P_BYTES_PER_BEAT(AXI_DATA_WIDTH/8),
    .P_MAX_BURST     (16)
  ) u_ag (
    .clk(clk), .rstn(rstn),
    .start(ag_start),
    .base_addr(ag_base_addr),
    .bytes_total(ag_bytes_total),
    .stride_bytes(32'd0),
    .req_valid(ag_req_valid), .req_ready(ag_req_ready),
    .req_addr(ag_req_addr), .req_len(ag_req_len), .req_last(ag_req_last),
    .done(ag_done)
  );

  // DMA read instance
  logic        start_dma, done_dma;
  logic [7:0] dma_num_trans;
  logic [AXI_ADDR_WIDTH-1:0] dma_start_addr;
  logic [DATA_W-1:0] rd_data;
  logic        rd_data_vld;

  dma_read #(
    .BITS_TRANS       (8),
    .AXI_WIDTH_AD     (AXI_ADDR_WIDTH),
    .AXI_WIDTH_DA     (DATA_W),
    .AXI_WIDTH_ARUSER (AXI_USER_WIDTH),
    .AXI_WIDTH_RUSER  (AXI_USER_WIDTH)
  ) u_dma_r (
    .clk(clk), .rstn(rstn),
    // AXI
    .M_AXI_ARVALID(M_AXI_ARVALID), .M_AXI_ARREADY(M_AXI_ARREADY), .M_AXI_ARADDR(M_AXI_ARADDR),
    .M_AXI_ARID   (M_AXI_ARID),    .M_AXI_ARLEN  (M_AXI_ARLEN),   .M_AXI_ARSIZE(M_AXI_ARSIZE),
    .M_AXI_ARBURST(M_AXI_ARBURST), .M_AXI_ARLOCK (M_AXI_ARLOCK),  .M_AXI_ARCACHE(M_AXI_ARCACHE),
    .M_AXI_ARPROT (M_AXI_ARPROT),  .M_AXI_ARQOS  (M_AXI_ARQOS),   .M_AXI_ARUSER(M_AXI_ARUSER),
    .M_AXI_RVALID (M_AXI_RVALID),  .M_AXI_RREADY (M_AXI_RREADY),  .M_AXI_RDATA (M_AXI_RDATA),
    .M_AXI_RRESP  (M_AXI_RRESP),   .M_AXI_RLAST  (M_AXI_RLAST),   .M_AXI_RID   (M_AXI_RID),
    .M_AXI_RUSER  (M_AXI_RUSER),
    // functional
    .start_dma(start_dma),
    .num_trans(dma_num_trans),
    .start_addr(dma_start_addr),
    .data_o(rd_data),
    .data_vld_o(rd_data_vld),
    .data_cnt_o(),
    .done_o(done_dma)
  );

  // Main FSM
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      state          <= S_IDLE;
      total_words    <= '0;
      word_cnt       <= '0;
      row_cnt        <= '0;
      row_words      <= '0;
      cur_block_m    <= '0;

      col_cnt        <= '0;

      // outputs
      ld_busy        <= 1'b0;
      ld_done        <= 1'b0;
      a_fill_we      <= 1'b0;
      a_fill_addr    <= '0;
      a_fill_wdata   <= '0;
      b_seg_words    <= '0;
      b_fill_req     <= 1'b0;
      b_fill_we      <= 1'b0;
      b_fill_addr    <= '0;
      b_fill_wdata   <= '0;
      b_fill_done    <= 1'b0;

      ag_start       <= 1'b0;
      ag_req_ready   <= 1'b0;
      ag_base_addr   <= '0;
      ag_bytes_total <= '0;
      start_dma      <= 1'b0;
      dma_num_trans  <= '0;
      dma_start_addr <= '0;
      k_q            <= '0;


    end else begin
      // defaults each cycle
      ld_done        <= 1'b0;
      a_fill_we      <= 1'b0;
      b_fill_req     <= 1'b0;
      b_fill_we      <= 1'b0;
      b_fill_done    <= 1'b0;
      ag_start       <= 1'b0;
      ag_req_ready   <= 1'b0;
      start_dma      <= 1'b0;

      if (done_dma) begin
        ag_req_ready <= 1'b1;
      end else if ((state == S_A_BULK) || (state == S_B_ROW)) begin
        if (ag_req_valid) begin
          ag_req_ready <= 1'b0;
        end else begin
          ag_req_ready <= 1'b1;
        end
      end


      case (state)
        S_IDLE: begin
          ld_busy <= 1'b0;
          if (load_req) begin
            if (update_a) begin
              // A bulk load: N×K bytes → N×K/4 words
              total_words    <= (N * K) >> 2;
              word_cnt       <= '0;
              a_fill_addr    <= '0;
              ag_base_addr   <= base_a_addr;
              ag_bytes_total <= N * K;
              ag_start       <= 1'b1;
              state          <= S_A_BULK;
            end else begin
              // B block load: pipeline expensive math across states
              // Stage 0 (S_IDLE): compute cur_block_m and capture K
              cur_block_m    <= ((M - j_block) < BLOCK_M) ? (M - j_block) : BLOCK_M;
              k_q            <= K;
              // Defer row_words/b_seg_words to S_B_REQ to ease timing
              word_cnt       <= '0;
              row_cnt        <= '0;
              col_cnt        <= '0;

              row_base_addr <= base_b_addr + j_block;
              state          <= S_B_REQ;
            end
            ld_busy <= 1'b1;
          end
        end

        S_A_BULK: begin
          ld_busy      <= 1'b1;
          
          // Accept addr_gen request
          if (ag_req_valid && ag_req_ready) begin
            dma_start_addr <= ag_req_addr;
            dma_num_trans  <= ag_req_len + 8'd1;
            start_dma      <= 1'b1;
          end else begin
            start_dma <= 1'b0;
          end

          // Receive data and fill A buffer
          if (rd_data_vld) begin
            a_fill_we    <= 1'b1;
            a_fill_addr  <= A_ADDR_W'(word_cnt);  // use current word_cnt as address
            a_fill_wdata <= rd_data;
            word_cnt     <= word_cnt + 1'b1;
          end

          // Check completion
          if (word_cnt + 1'b1 >= total_words && rd_data_vld) begin
            state   <= S_DONE;
          end
        end

        S_B_REQ: begin
          ld_busy    <= 1'b1;
          // Stage 1 (S_B_REQ): complete cheap shift and the multiply
          row_words   <= cur_block_m >> 2;
          b_seg_words <= (k_q * cur_block_m) >> 2;
          // Now request B buffer fill; bram latches b_seg_words with the req
          b_fill_req <= 1'b1;  // hold high until busy asserts
          if (b_fill_busy) begin
            // bram_pingpong accepted the request
            b_fill_req <= 1'b0;
            state      <= S_B_PREP;
          end
        end

        S_B_PREP: begin
          ld_busy <= 1'b1;
          if (row_cnt < K) begin
            // Start addr_gen for this row
            ag_base_addr   <= row_base_addr;
            ag_bytes_total <= cur_block_m;
            ag_start       <= 1'b1;
            state          <= S_B_ROW;
          end else begin
            // All rows done
            state <= S_B_WAIT;
          end
        end

        S_B_ROW: begin
          ld_busy      <= 1'b1;

          // Accept addr_gen request
          if (ag_req_valid && ag_req_ready) begin
            dma_start_addr <= ag_req_addr;
            dma_num_trans  <= ag_req_len + 8'd1;
            start_dma      <= 1'b1;
          end else begin
            start_dma <= 1'b0;
          end

          // Receive data and fill B buffer
          if (rd_data_vld) begin
            b_fill_we    <= 1'b1;
            // b_fill_addr  <= B_ADDR_W'(row_cnt * row_words + (word_cnt % row_words));
            b_fill_addr  <= B_ADDR_W'(row_cnt * row_words + col_cnt);
            b_fill_wdata <= rd_data;
            word_cnt     <= word_cnt + 1'b1;
          // end

          // Row complete
          // if (word_cnt % row_words == row_words - 1 && rd_data_vld) begin
          //   row_cnt <= row_cnt + 1'b1;
          //   state   <= S_B_PREP;
          // end
            if (col_cnt == row_words - 1) begin
              col_cnt <= '0;
              row_cnt <= row_cnt + 1'b1;
              row_base_addr <= row_base_addr + stride_b_row_bytes;
              state   <= S_B_PREP;
            end else begin
              col_cnt <= col_cnt + 1'b1;
            end
          end
        end

        S_B_WAIT: begin
          ld_busy <= 1'b1;
          // Wait one cycle for bram_pingpong to latch fill_done
          b_fill_done <= 1'b1;
          state       <= S_DONE;
        end

        S_DONE: begin
          ld_busy <= 1'b0;  // work complete, release busy
          ld_done <= 1'b1;  // completion pulse
          state   <= S_IDLE;
        end

      endcase
    end
  end

endmodule
