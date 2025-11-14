//----------------------------------------------------------------+
// Project: Capstone Design
// Module : tile_store (SystemVerilog)
// Description:
//   Capture C tile stream (row-major) into ping-pong BRAM and write
//   back to DRAM using AXI4 via existing dma_write.
//   - Per-row bursts for correctness (handles edges naturally).
//   - Full tile (8x8): 8 bursts of 8 beats (32B/row).
//   - Edge tile: n_eff rows, AWLEN=m_eff-1.
//
// Notes:
//   - Exposes AXI master signals from internal dma_write instance.
//   - stride_c_row_bytes should be M*4 (bytes per C row).
// Last Updated: 2025-11-11 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns/1ps

import sa_params_pkg::*;
`include "sa_defs.svh"

module tile_store #(
  parameter int TILE_SIZE       = 8,
  parameter int DATA_W          = 32,
  // AXI USER width parameterized for clean integration with dma_write and TB
  parameter int AXI_USER_WIDTH  = 1
 )(
  input  logic                         clk,
  input  logic                         rstn,

  // Tile meta
  input  logic [AXI_ADDR_WIDTH-1:0]    base_c_addr,       // byte addr base_C
  input  logic [31:0]                  stride_c_row_bytes,// M*4
  input  logic [15:0]                  i0,                // tile row origin
  input  logic [15:0]                  j0,                // tile col origin
  input  logic [3:0]                   n_eff,             // 1..8
  input  logic [3:0]                   m_eff,             // 1..8

  // Store control
  input  logic                         start_store,       // pulse to begin write-back for one tile bank

  // External C buffer (ping-pong) consumer interface
  output logic                         cbuf_consume_req,  // pulse: arm read bank
  input  logic                         cbuf_consume_busy, // optional observability
  output logic [$clog2(64)-1:0]        cbuf_rd_addr,      // word address within tile bank
  input  logic [DATA_W-1:0]            cbuf_rd_data,      // registered data (1-cycle)
  output logic                         cbuf_rd_en,        // assert per beat when data consumed (USE_CONS_COMMIT=0)
  output logic                         cbuf_cons_commit,  // pulse: release bank after done

  // Status
  output logic                         wr_busy,
  output logic                         wr_done,           // pulse per tile write-back

  // AXI4 Master (connect upward)
  output logic                         M_AXI_AWVALID,
  input  logic                         M_AXI_AWREADY,
  output logic [AXI_ADDR_WIDTH-1:0]    M_AXI_AWADDR,
  output logic [3:0]                   M_AXI_AWID,
  output logic [7:0]                   M_AXI_AWLEN,
  output logic [2:0]                   M_AXI_AWSIZE,
  output logic [1:0]                   M_AXI_AWBURST,
  output logic                         M_AXI_AWLOCK,
  output logic [3:0]                   M_AXI_AWCACHE,
  output logic [2:0]                   M_AXI_AWPROT,
  output logic [3:0]                   M_AXI_AWQOS,
  output logic [AXI_USER_WIDTH-1:0]    M_AXI_AWUSER,

  output logic                         M_AXI_WVALID,
  input  logic                         M_AXI_WREADY,
  output logic [DATA_W-1:0]            M_AXI_WDATA,
  output logic [(DATA_W/8)-1:0]        M_AXI_WSTRB,
  output logic                         M_AXI_WLAST,
  output logic [AXI_USER_WIDTH-1:0]    M_AXI_WUSER,

  input  logic                         M_AXI_BVALID,
  output logic                         M_AXI_BREADY,
  input  logic [1:0]                   M_AXI_BRESP,
  input  logic [3:0]                   M_AXI_BID,
  input  logic [AXI_USER_WIDTH-1:0]    M_AXI_BUSER
);

  // Control FSM (single always_ff style)
  typedef enum logic [2:0] { S_IDLE, S_PREP_ROW, S_REQ, S_DMA, S_NEXT_ROW, S_DONE } state_e;
  state_e state;

  logic [3:0] row_idx;     // 0..7
  logic [3:0] col_idx;     // 0..7

  // DMA write instance and parameters
  logic        start_dma, done_dma, indata_req_o;
  // Per-burst parameters
  logic [12:0] dma_words_q;                    // beats in current burst
  logic [AXI_ADDR_WIDTH-1:0] dma_start_addr_q; // start addr of current burst
  // Per-row parameters (for addr_gen)
  logic [12:0] row_words_q;                    // words per row (= m_eff)
  logic [AXI_ADDR_WIDTH-1:0] row_base_q;       // base addr of current row
  // Track whether the accepted burst is the last one for this row
  logic        burst_last_q;
  // Track first WVALID handshake within S_DMA to open a read window
//   logic wvalid_q;           // previous WVALID level
  logic rd_window_q;        // latched window after first WVALID handshake
  logic w_fire;             // AXI W channel handshake (VALID && READY)
  assign w_fire = M_AXI_AWVALID & M_AXI_AWREADY;

  // Address generator (WRITE): per-row bursts
  logic                 ag_start, ag_done;
  logic                 ag_req_valid, ag_req_ready, ag_req_last;
  logic [AXI_ADDR_WIDTH-1:0] ag_req_addr;
  logic [7:0]            ag_req_len; // beats-1

  axi_addr_gen #(
    .ADDR_W          (AXI_ADDR_WIDTH),
    .P_BYTES_PER_BEAT(AXI_DATA_WIDTH/8),
    .P_MAX_BURST     (16)
  ) u_ag (
    .clk(clk), .rstn(rstn),
    .start   (ag_start),
    .base_addr(row_base_q),
    .bytes_total(row_words_q* (AXI_DATA_WIDTH/8)),
    .stride_bytes(32'd0),
    .req_valid(ag_req_valid), .req_ready(ag_req_ready),
    .req_addr(ag_req_addr), .req_len(ag_req_len), .req_last(ag_req_last),
    .done(ag_done)
  );

  dma_write #(
    .AXI_WIDTH_AD     (AXI_ADDR_WIDTH),
    .AXI_WIDTH_DA     (DATA_W),
    .AXI_WIDTH_AWUSER (AXI_USER_WIDTH),
    .AXI_WIDTH_WUSER  (AXI_USER_WIDTH),
    .AXI_WIDTH_BUSER  (AXI_USER_WIDTH)
  ) u_dma_w (
    // AXI
    .M_AXI_AWVALID(M_AXI_AWVALID), .M_AXI_AWREADY(M_AXI_AWREADY), .M_AXI_AWADDR(M_AXI_AWADDR),
    .M_AXI_AWID   (M_AXI_AWID),    .M_AXI_AWLEN  (M_AXI_AWLEN),   .M_AXI_AWSIZE(M_AXI_AWSIZE),
    .M_AXI_AWBURST(M_AXI_AWBURST), .M_AXI_AWLOCK (M_AXI_AWLOCK),  .M_AXI_AWCACHE(M_AXI_AWCACHE),
    .M_AXI_AWPROT (M_AXI_AWPROT),  .M_AXI_AWQOS  (M_AXI_AWQOS),   .M_AXI_AWUSER(M_AXI_AWUSER),
    .M_AXI_WVALID (M_AXI_WVALID),  .M_AXI_WREADY (M_AXI_WREADY),  .M_AXI_WDATA (M_AXI_WDATA),
    .M_AXI_WSTRB  (M_AXI_WSTRB),   .M_AXI_WLAST  (M_AXI_WLAST),   .M_AXI_WUSER (M_AXI_WUSER),
    .M_AXI_BVALID (M_AXI_BVALID),  .M_AXI_BREADY (M_AXI_BREADY),  .M_AXI_BRESP (M_AXI_BRESP),
    .M_AXI_BID    (M_AXI_BID),     .M_AXI_BUSER  (M_AXI_BUSER),
    // functional
    .start_dma(start_dma), .done_o(done_dma),
    .num_trans(dma_words_q), .start_addr(dma_start_addr_q),
    .indata(cbuf_rd_data), .indata_req_o(indata_req_o),
    .fail_check(),
    .clk(clk), .rstn(rstn)
  );

  // Single always_ff: state, outputs, and registers
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      state            <= S_IDLE;
      row_idx          <= '0; col_idx <= '0;
      burst_last_q     <= 1'b0;
      row_words_q      <= '0;
      row_base_q       <= '0;
      dma_words_q      <= '0;
      dma_start_addr_q <= '0;
      // outputs
      cbuf_consume_req <= 1'b0;
      cbuf_cons_commit <= 1'b0;
      cbuf_rd_addr     <= '0;
      cbuf_rd_en       <= 1'b0;
      wr_done          <= 1'b0;
      wr_busy          <= 1'b0;
      ag_start         <= 1'b0;
      ag_req_ready     <= 1'b0;
      start_dma        <= 1'b0;
      // WVALID track
//       wvalid_q         <= 1'b0;
      rd_window_q      <= 1'b0;
    end else begin
      // defaults each cycle
      cbuf_consume_req <= 1'b0;
      cbuf_cons_commit <= 1'b0;
    //   cbuf_rd_addr     <= '0;
      cbuf_rd_en       <= 1'b0;
      wr_done          <= 1'b0;
      ag_start         <= 1'b0;
      ag_req_ready     <= 1'b0;
      start_dma        <= 1'b0;
      // track previous WVALID each cycle (for edge-handling if needed)
//       wvalid_q         <= M_AXI_WVALID;

      case (state)
        S_IDLE: begin
          wr_busy      <= 1'b0;
          row_idx      <= '0; col_idx <= '0;
          burst_last_q <= 1'b0;
          if (start_store) begin
            cbuf_consume_req <= 1'b1;

            row_base_q       <= base_c_addr + (i0 * stride_c_row_bytes) + (j0 * (DATA_W/8));
            state            <= S_PREP_ROW;
          end else begin
            state            <= S_IDLE;
          end
        end

        S_PREP_ROW: begin
          wr_busy      <= 1'b1;
          row_words_q  <= m_eff;
          // row_base_q   <= base_c_addr + ( (i0 + row_idx) * stride_c_row_bytes ) + ( j0 * (DATA_W/8) );
          ag_start     <= 1'b1;  // one-cycle pulse
          col_idx      <= 4'd0;
          burst_last_q <= 1'b0;
          state        <= S_REQ;
        end

        S_REQ: begin
          wr_busy      <= 1'b1;
          ag_req_ready <= 1'b1;
          if (ag_req_valid) begin
            dma_words_q      <= ag_req_len + 8'd1;
            dma_start_addr_q <= ag_req_addr;
            burst_last_q     <= ag_req_last;
            cbuf_rd_addr      <= (row_idx*TILE_SIZE);
            start_dma        <= 1'b1;
            rd_window_q      <= 1'b0;   // clear window for new burst
            state            <= S_DMA;
          end else begin
            state            <= S_REQ;
          end
        end

        S_DMA: begin
          wr_busy      <= 1'b1;
          start_dma    <= 1'b0;
          // Latch window open on the first W handshake; keep it until end of burst
          if (!rd_window_q && w_fire) begin
            rd_window_q <= 1'b1;
          end
          // Drive BRAM read only after the first WVALID handshake
          if ( (rd_window_q || w_fire) && (col_idx < m_eff) ) begin
            cbuf_rd_en  <= 1'b1;
            col_idx     <= col_idx + 1'b1;
            cbuf_rd_addr<= cbuf_rd_addr + 1'b1;
          end
          if (done_dma) begin
            state <= (burst_last_q) ? S_NEXT_ROW : S_REQ;
            cbuf_rd_en <= 1'b0;
            rd_window_q <= 1'b0; // close window at burst end
          end else begin
            state <= S_DMA;
          end
        end

        S_NEXT_ROW: begin
          wr_busy      <= 1'b1;
          if (row_idx + 1 < n_eff) begin
            row_idx <= row_idx + 1'b1;
            col_idx <= 4'd0;
            row_base_q <= row_base_q + stride_c_row_bytes;
            // cbuf_consume_req <= 1'b1;
            state   <= S_PREP_ROW;
          end else begin
            state   <= S_DONE;
          end
        end

        S_DONE: begin
          wr_busy          <= 1'b0;
          cbuf_cons_commit <= 1'b1; // ignored by USE_CONS_COMMIT=0 but harmless
          wr_done          <= 1'b1;  // completion pulse
          state            <= S_IDLE;
        end

      endcase
    end
  end

endmodule
