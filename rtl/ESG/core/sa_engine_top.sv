//----------------------------------------------------------------+
// Project: Capstone Design
// Module : sa_engine_top (SystemVerilog module)
// Description:
//   Top-level integration of tile-based matrix multiplication pipeline.
//   - AXI-Lite slave interface for configuration
//   - AXI4-Full master Read/Write for DRAM access
//   - Internal buffers: A (dpram), B (ping-pong), C (ping-pong)
//   - Tile controller + loader + compute + store modules
//
// Register Map:
//   0x00: CONTROL (start[0], update_A[1], done[16], busy[17])
//   0x04: N
//   0x08: K
//   0x0C: M
//   0x10: BLOCK_M
//   0x14: base_A (addr)
//   0x18: base_B (addr)
//   0x1C: base_C (addr)
//   0x20: stride_A_row_bytes
//   0x24: stride_B_row_bytes
//   0x28: stride_C_row_bytes
//
// Last Updated: 2025-11-12 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns/1ps

import sa_params_pkg::*;
`include "sa_defs.svh"

module sa_engine_top #(
  parameter int C_M_AXI_ID_WIDTH     = 1,
  parameter int C_M_AXI_ADDR_WIDTH   = 32,
  parameter int C_M_AXI_DATA_WIDTH   = 32,
  parameter int C_M_AXI_AWUSER_WIDTH = 0,
  parameter int C_M_AXI_ARUSER_WIDTH = 0,
  parameter int C_M_AXI_WUSER_WIDTH  = 0,
  parameter int C_M_AXI_RUSER_WIDTH  = 0,
  parameter int C_M_AXI_BUSER_WIDTH  = 0,
  // Tile-based pipeline configuration
  parameter int TILE_SIZE            = 8,
  parameter int A_DEPTH              = 12288,  // 64×768/4 words
  parameter int B_DEPTH              = 49152,  // 768×256/4 words
  parameter int C_DEPTH              = 64,     // 8×8 words
  parameter bit USE_DSP              = 1'b1    // PE DSP mode
)(
  // ===== Clocks and Resets =====
  input  logic                       S_AXI_ACLK,
  input  logic                       S_AXI_ARESETN,
  input  logic                       M_AXI_ACLK,
  input  logic                       M_AXI_ARESETN,

  // ===== AXI-Lite Control Registers =====
  input  logic                       i_start,        // REG[0x00][0]
  input  logic                       i_update_A,     // REG[0x00][1]
  input  logic [15:0]                i_N,            // REG[0x04][15:0]
  input  logic [15:0]                i_K,            // REG[0x08][15:0]
  input  logic [15:0]                i_M,            // REG[0x0C][15:0]
  input  logic [15:0]                i_BLOCK_M,      // REG[0x10][15:0]
  input  logic [C_M_AXI_ADDR_WIDTH-1:0] i_base_A,   // REG[0x14]
  input  logic [C_M_AXI_ADDR_WIDTH-1:0] i_base_B,   // REG[0x18]
  input  logic [C_M_AXI_ADDR_WIDTH-1:0] i_base_C,   // REG[0x1C]
  input  logic [31:0]                i_stride_A,    // REG[0x20]
  input  logic [31:0]                i_stride_B,    // REG[0x24]
  input  logic [31:0]                i_stride_C,    // REG[0x28]

  // ===== Status Outputs (to AXI-Lite) =====
  output logic                       o_busy,         // REG[0x00][17]
  output logic                       o_done,         // REG[0x00][16] (pulse)
  output logic                       o_error,        // REG[0x00][18]

  // ===== AXI4-Full Master Write =====
  output logic [C_M_AXI_ID_WIDTH-1:0]     M_AXI_AWID,
  output logic [C_M_AXI_ADDR_WIDTH-1:0]   M_AXI_AWADDR,
  output logic [7:0]                      M_AXI_AWLEN,
  output logic [2:0]                      M_AXI_AWSIZE,
  output logic [1:0]                      M_AXI_AWBURST,
  output logic                            M_AXI_AWLOCK,
  output logic [3:0]                      M_AXI_AWCACHE,
  output logic [2:0]                      M_AXI_AWPROT,
  output logic [3:0]                      M_AXI_AWQOS,
  output logic [C_M_AXI_AWUSER_WIDTH-1:0] M_AXI_AWUSER,
  output logic                            M_AXI_AWVALID,
  input  logic                            M_AXI_AWREADY,

  output logic [C_M_AXI_DATA_WIDTH-1:0]   M_AXI_WDATA,
  output logic [C_M_AXI_DATA_WIDTH/8-1:0] M_AXI_WSTRB,
  output logic                            M_AXI_WLAST,
  output logic [C_M_AXI_WUSER_WIDTH-1:0]  M_AXI_WUSER,
  output logic                            M_AXI_WVALID,
  input  logic                            M_AXI_WREADY,

  input  logic [C_M_AXI_ID_WIDTH-1:0]     M_AXI_BID,
  input  logic [1:0]                      M_AXI_BRESP,
  input  logic [C_M_AXI_BUSER_WIDTH-1:0]  M_AXI_BUSER,
  input  logic                            M_AXI_BVALID,
  output logic                            M_AXI_BREADY,

  // ===== AXI4-Full Master Read =====
  output logic [C_M_AXI_ID_WIDTH-1:0]     M_AXI_ARID,
  output logic [C_M_AXI_ADDR_WIDTH-1:0]   M_AXI_ARADDR,
  output logic [7:0]                      M_AXI_ARLEN,
  output logic [2:0]                      M_AXI_ARSIZE,
  output logic [1:0]                      M_AXI_ARBURST,
  output logic                            M_AXI_ARLOCK,
  output logic [3:0]                      M_AXI_ARCACHE,
  output logic [2:0]                      M_AXI_ARPROT,
  output logic [3:0]                      M_AXI_ARQOS,
  output logic [C_M_AXI_ARUSER_WIDTH-1:0] M_AXI_ARUSER,
  output logic                            M_AXI_ARVALID,
  input  logic                            M_AXI_ARREADY,

  input  logic [C_M_AXI_ID_WIDTH-1:0]     M_AXI_RID,
  input  logic [C_M_AXI_DATA_WIDTH-1:0]   M_AXI_RDATA,
  input  logic [1:0]                      M_AXI_RRESP,
  input  logic                            M_AXI_RLAST,
  input  logic [C_M_AXI_RUSER_WIDTH-1:0]  M_AXI_RUSER,
  input  logic                            M_AXI_RVALID,
  output logic                            M_AXI_RREADY
);

  // ========================================================================
  // Local Parameters
  // ========================================================================
  localparam int DATA_W      = 32;
  localparam int A_ADDR_W    = $clog2(A_DEPTH);  // 14-bit
  localparam int B_ADDR_W    = $clog2(B_DEPTH);  // 16-bit
  localparam int C_ADDR_W    = $clog2(C_DEPTH);  // 6-bit
  localparam int AXI_USER_W  = 1;

  // ========================================================================
  // Internal Signals
  // ========================================================================
  // Controller signals
  logic                         ctrl_busy, ctrl_done;
  logic                         loader_req, ld_done;
  logic                         start_tile, tile_done;
  logic                         start_store, wr_done;
  
  // Controller → Loader
  logic                         loader_update_a;
  logic [15:0]                  loader_N, loader_K, loader_M, loader_BLOCK_M;
  logic [15:0]                  loader_j_block;
  logic [C_M_AXI_ADDR_WIDTH-1:0] loader_base_a, loader_base_b;
  logic [31:0]                  loader_stride_a_row, loader_stride_b_row;
  
  // Controller → Compute
  logic [15:0]                  compute_k_total;
  logic [3:0]                   compute_n_eff, compute_m_eff;
  logic                         tile_load_req;
  logic [3:0]                   k_eff;
  logic                         a_ld_start, a_ld_valid;
  logic [31:0]                  a_ld_data;
  logic                         b_ld_start, b_ld_valid;
  logic [31:0]                  b_ld_data;
  
  // Controller → Store
  logic [15:0]                  store_i0, store_j0;
  logic [3:0]                   store_n_eff, store_m_eff;
  logic [C_M_AXI_ADDR_WIDTH-1:0] store_base_c;
  logic [31:0]                  store_stride_c_row;
  
  // A buffer (dpram_wrapper) signals
  logic                         a_fill_we;
  logic [A_ADDR_W-1:0]          a_fill_addr;
  logic [DATA_W-1:0]            a_fill_wdata;
  logic [A_ADDR_W-1:0]          a_rd_addr;
  logic                         a_rd_en;
  logic [DATA_W-1:0]            a_rd_data;
  
  // B buffer (bram_pingpong) signals
  logic [31:0]                  b_seg_words;
  logic                         b_fill_req, b_fill_busy, b_fill_done;
  logic                         b_fill_we;
  logic [B_ADDR_W-1:0]          b_fill_addr;
  logic [DATA_W-1:0]            b_fill_wdata;
  logic                         b_consume_req, b_consume_busy;
  logic [B_ADDR_W-1:0]          b_rd_addr;
  logic                         b_rd_en;
  logic [DATA_W-1:0]            b_rd_data;
  logic                         b_cons_commit, b_consume_done;
  logic                         b_bank_sel;
  
  // C buffer (bram_pingpong) signals
  logic [31:0]                  c_seg_words;
  logic                         c_fill_req, c_fill_busy, c_fill_done;
  logic                         c_fill_we;
  logic [C_ADDR_W-1:0]          c_fill_addr;
  logic [DATA_W-1:0]            c_fill_wdata;
  logic                         c_consume_req, c_consume_busy;
  logic [C_ADDR_W-1:0]          c_rd_addr;
  logic                         c_rd_en;
  logic [DATA_W-1:0]            c_rd_data;
  logic                         c_cons_commit, c_consume_done;
  logic                         c_bank_sel;

  // AXI Read (Loader → DRAM)
  logic                         ld_M_AXI_ARVALID;
  logic                         ld_M_AXI_ARREADY;
  logic [C_M_AXI_ADDR_WIDTH-1:0] ld_M_AXI_ARADDR;
  logic [3:0]                   ld_M_AXI_ARID;
  logic [7:0]                   ld_M_AXI_ARLEN;
  logic [2:0]                   ld_M_AXI_ARSIZE;
  logic [1:0]                   ld_M_AXI_ARBURST;
  logic                         ld_M_AXI_ARLOCK;
  logic [3:0]                   ld_M_AXI_ARCACHE;
  logic [2:0]                   ld_M_AXI_ARPROT;
  logic [3:0]                   ld_M_AXI_ARQOS;
  logic [AXI_USER_W-1:0]        ld_M_AXI_ARUSER;
  logic                         ld_M_AXI_RVALID;
  logic                         ld_M_AXI_RREADY;
  logic [DATA_W-1:0]            ld_M_AXI_RDATA;
  logic [1:0]                   ld_M_AXI_RRESP;
  logic                         ld_M_AXI_RLAST;
  logic [3:0]                   ld_M_AXI_RID;
  logic [AXI_USER_W-1:0]        ld_M_AXI_RUSER;

  // AXI Write (Store → DRAM)
  logic                         st_M_AXI_AWVALID;
  logic                         st_M_AXI_AWREADY;
  logic [C_M_AXI_ADDR_WIDTH-1:0] st_M_AXI_AWADDR;
  logic [3:0]                   st_M_AXI_AWID;
  logic [7:0]                   st_M_AXI_AWLEN;
  logic [2:0]                   st_M_AXI_AWSIZE;
  logic [1:0]                   st_M_AXI_AWBURST;
  logic                         st_M_AXI_AWLOCK;
  logic [3:0]                   st_M_AXI_AWCACHE;
  logic [2:0]                   st_M_AXI_AWPROT;
  logic [3:0]                   st_M_AXI_AWQOS;
  logic [AXI_USER_W-1:0]        st_M_AXI_AWUSER;
  logic                         st_M_AXI_WVALID;
  logic                         st_M_AXI_WREADY;
  logic [DATA_W-1:0]            st_M_AXI_WDATA;
  logic [(DATA_W/8)-1:0]        st_M_AXI_WSTRB;
  logic                         st_M_AXI_WLAST;
  logic [AXI_USER_W-1:0]        st_M_AXI_WUSER;
  logic                         st_M_AXI_BVALID;
  logic                         st_M_AXI_BREADY;
  logic [1:0]                   st_M_AXI_BRESP;
  logic [3:0]                   st_M_AXI_BID;
  logic [AXI_USER_W-1:0]        st_M_AXI_BUSER;

  // ========================================================================
  // Status Outputs
  //   - ctrl_done: pulse from tile_controller(1 cycle done signal)
  //   - done_latched: sticky level for AXI-Lite read
  // ========================================================================
  logic done_latched;

  always_ff @(posedge M_AXI_ACLK or negedge M_AXI_ARESETN) begin
    if (!M_AXI_ARESETN) begin
      done_latched <= 1'b0;
    end else begin
      // new start clears DONE
      if (i_start) begin
        done_latched <= 1'b0;
      end
      // controller done pulse sets DONE
      else if (ctrl_done) begin
        done_latched <= 1'b1;
      end
      // otherwise, keep value
    end
  end

  assign o_busy  = ctrl_busy;
  assign o_done  = done_latched;  // <-- now a sticky level, not a pulse
  assign o_error = 1'b0;          // No error handling yet


  // ========================================================================
  // AXI Mux (Read: Loader only, Write: Store only)
  // ========================================================================
  // Read channel: directly connect loader to M_AXI_AR/R
  assign M_AXI_ARVALID = ld_M_AXI_ARVALID;
  assign M_AXI_ARADDR  = ld_M_AXI_ARADDR;
  assign M_AXI_ARID    = ld_M_AXI_ARID;
  assign M_AXI_ARLEN   = ld_M_AXI_ARLEN;
  assign M_AXI_ARSIZE  = ld_M_AXI_ARSIZE;
  assign M_AXI_ARBURST = ld_M_AXI_ARBURST;
  assign M_AXI_ARLOCK  = ld_M_AXI_ARLOCK;
  assign M_AXI_ARCACHE = ld_M_AXI_ARCACHE;
  assign M_AXI_ARPROT  = ld_M_AXI_ARPROT;
  assign M_AXI_ARQOS   = ld_M_AXI_ARQOS;
  assign M_AXI_ARUSER  = ld_M_AXI_ARUSER;
  
  assign ld_M_AXI_ARREADY = M_AXI_ARREADY;
  assign ld_M_AXI_RVALID  = M_AXI_RVALID;
  assign ld_M_AXI_RDATA   = M_AXI_RDATA;
  assign ld_M_AXI_RRESP   = M_AXI_RRESP;
  assign ld_M_AXI_RLAST   = M_AXI_RLAST;
  assign ld_M_AXI_RID     = M_AXI_RID;
  assign ld_M_AXI_RUSER   = M_AXI_RUSER;
  assign M_AXI_RREADY     = ld_M_AXI_RREADY;

  // Write channel: directly connect store to M_AXI_AW/W/B
  assign M_AXI_AWVALID = st_M_AXI_AWVALID;
  assign M_AXI_AWADDR  = st_M_AXI_AWADDR;
  assign M_AXI_AWID    = st_M_AXI_AWID;
  assign M_AXI_AWLEN   = st_M_AXI_AWLEN;
  assign M_AXI_AWSIZE  = st_M_AXI_AWSIZE;
  assign M_AXI_AWBURST = st_M_AXI_AWBURST;
  assign M_AXI_AWLOCK  = st_M_AXI_AWLOCK;
  assign M_AXI_AWCACHE = st_M_AXI_AWCACHE;
  assign M_AXI_AWPROT  = st_M_AXI_AWPROT;
  assign M_AXI_AWQOS   = st_M_AXI_AWQOS;
  assign M_AXI_AWUSER  = st_M_AXI_AWUSER;
  
  assign st_M_AXI_AWREADY = M_AXI_AWREADY;
  
  assign M_AXI_WVALID = st_M_AXI_WVALID;
  assign M_AXI_WDATA  = st_M_AXI_WDATA;
  assign M_AXI_WSTRB  = st_M_AXI_WSTRB;
  assign M_AXI_WLAST  = st_M_AXI_WLAST;
  assign M_AXI_WUSER  = st_M_AXI_WUSER;
  
  assign st_M_AXI_WREADY = M_AXI_WREADY;
  
  assign st_M_AXI_BVALID = M_AXI_BVALID;
  assign st_M_AXI_BRESP  = M_AXI_BRESP;
  assign st_M_AXI_BID    = M_AXI_BID;
  assign st_M_AXI_BUSER  = M_AXI_BUSER;
  assign M_AXI_BREADY    = st_M_AXI_BREADY;

  // ========================================================================
  // Module Instances
  // ========================================================================

    // -------------------- A Buffer (BRAM: sa_bramA_32x12288) --------------------
    // Port A : write side (tile_loader)
    // Port B : read side  (tile_controller)
    sa_bramA_32x12288 u_a_buf (
      // Port A (write)
      .clka  (M_AXI_ACLK),
      .ena   (1'b1),
      .wea   (a_fill_we),
      .addra (a_fill_addr),
      .dina  (a_fill_wdata),

      // Port B (read)
      .clkb  (M_AXI_ACLK),
      .enb   (1'b1),
      .addrb (a_rd_addr),
      .doutb (a_rd_data)
    );


  // -------------------- B Buffer (BRAM Ping-Pong) --------------------
  bram_pingpong #(
    .DATA_W          (DATA_W),
    .DEPTH           (B_DEPTH),
    .USE_CONS_COMMIT (1'b1)  // External commit mode (B block)
  ) u_b_buf (
    .clk           (M_AXI_ACLK),
    .rstn          (M_AXI_ARESETN),
    .seg_words     (b_seg_words),
    // Producer (tile_loader)
    .fill_req      (b_fill_req),
    .fill_busy     (b_fill_busy),
    .fill_we       (b_fill_we),
    .fill_addr     (b_fill_addr),
    .fill_wdata    (b_fill_wdata),
    .fill_done     (b_fill_done),
    // Consumer (tile_controller)
    .consume_req   (b_consume_req),
    .consume_busy  (b_consume_busy),
    .rd_addr       (b_rd_addr),
    .rd_en         (b_rd_en),
    .rd_rdata      (b_rd_data),
    .cons_commit   (b_cons_commit),
    .consume_done  (b_consume_done),
    .bank_sel      (b_bank_sel)
  );

  // -------------------- C Buffer (BRAM Ping-Pong) --------------------
  bram_pingpong #(
    .DATA_W          (DATA_W),
    .DEPTH           (C_DEPTH),
    .USE_CONS_COMMIT (1'b0)  // Internal rd_en counting mode (C tile)
  ) u_c_buf (
    .clk           (M_AXI_ACLK),
    .rstn          (M_AXI_ARESETN),
    .seg_words     (c_seg_words),
    // Producer (tile_compute drain)
    .fill_req      (c_fill_req),
    .fill_busy     (c_fill_busy),
    .fill_we       (c_fill_we),
    .fill_addr     (c_fill_addr),
    .fill_wdata    (c_fill_wdata),
    .fill_done     (c_fill_done),
    // Consumer (tile_store)
    .consume_req   (c_consume_req),
    .consume_busy  (c_consume_busy),
    .rd_addr       (c_rd_addr),
    .rd_en         (c_rd_en),
    .rd_rdata      (c_rd_data),
    .cons_commit   (c_cons_commit),
    .consume_done  (c_consume_done),
    .bank_sel      (c_bank_sel)
  );

  // -------------------- Tile Controller --------------------
  tile_controller #(
    .TILE_SIZE(TILE_SIZE)
  ) u_controller (
    .clk                (M_AXI_ACLK),
    .rstn               (M_AXI_ARESETN),
    // Control from AXI-Lite
    .start              (i_start),
    .update_A           (i_update_A),
    .N                  (i_N),
    .K                  (i_K),
    .M                  (i_M),
    .BLOCK_M            (i_BLOCK_M),
    .base_A             (i_base_A),
    .base_B             (i_base_B),
    .base_C             (i_base_C),
    .stride_A_row_bytes (i_stride_A),
    .stride_B_row_bytes (i_stride_B),
    .stride_C_row_bytes (i_stride_C),
    // To tile_loader
    .loader_req         (loader_req),
    .loader_update_a    (loader_update_a),
    .loader_N           (loader_N),
    .loader_K           (loader_K),
    .loader_M           (loader_M),
    .loader_BLOCK_M     (loader_BLOCK_M),
    .loader_j_block     (loader_j_block),
    .loader_base_a      (loader_base_a),
    .loader_base_b      (loader_base_b),
    .loader_stride_a_row(loader_stride_a_row),
    .loader_stride_b_row(loader_stride_b_row),
    .ld_done            (ld_done),
    // To tile_compute
    .start_tile         (start_tile),
    .compute_k_total    (compute_k_total),
    .compute_n_eff      (compute_n_eff),
    .compute_m_eff      (compute_m_eff),
    .tile_done          (tile_done),
    .tile_load_req      (tile_load_req),
    .k_eff              (k_eff),
    .a_ld_start         (a_ld_start),
    .a_ld_valid         (a_ld_valid),
    .a_ld_data          (a_ld_data),
    .b_ld_start         (b_ld_start),
    .b_ld_valid         (b_ld_valid),
    .b_ld_data          (b_ld_data),
    // C buffer fill completion
    .c_fill_done        (c_fill_done),
    // A buffer read interface
    .a_rd_addr          (a_rd_addr),
    .a_rd_en            (a_rd_en),
    .a_rd_data          (a_rd_data),
    // B buffer consume interface
    .b_consume_req      (b_consume_req),
    .b_consume_busy     (b_consume_busy),
    .b_rd_addr          (b_rd_addr),
    .b_rd_en            (b_rd_en),
    .b_rd_data          (b_rd_data),
    .b_cons_commit      (b_cons_commit),
    // To tile_store
    .start_store        (start_store),
    .store_i0           (store_i0),
    .store_j0           (store_j0),
    .store_n_eff        (store_n_eff),
    .store_m_eff        (store_m_eff),
    .store_base_c       (store_base_c),
    .store_stride_c_row (store_stride_c_row),
    .wr_done            (wr_done),
    // Status
    .busy               (ctrl_busy),
    .done               (ctrl_done)
  );

  // -------------------- Tile Loader --------------------
  tile_loader #(
    .TILE_SIZE      (TILE_SIZE),
    .DATA_W         (DATA_W),
    .AXI_USER_WIDTH (AXI_USER_W),
    .A_ADDR_W       (A_ADDR_W),
    .B_ADDR_W       (B_ADDR_W)
  ) u_loader (
    .clk                (M_AXI_ACLK),
    .rstn               (M_AXI_ARESETN),
    // Control from controller
    .load_req           (loader_req),
    .update_a           (loader_update_a),
    .N                  (loader_N),
    .K                  (loader_K),
    .M                  (loader_M),
    .BLOCK_M            (loader_BLOCK_M),
    .j_block            (loader_j_block),
    .base_a_addr        (loader_base_a),
    .base_b_addr        (loader_base_b),
    .stride_a_row_bytes (loader_stride_a_row),
    .stride_b_row_bytes (loader_stride_b_row),
    // Status
    .ld_busy            (),  // unused
    .ld_done            (ld_done),
    // A buffer fill interface
    .a_fill_we          (a_fill_we),
    .a_fill_addr        (a_fill_addr),
    .a_fill_wdata       (a_fill_wdata),
    // B buffer fill interface
    .b_seg_words        (b_seg_words),
    .b_fill_req         (b_fill_req),
    .b_fill_busy        (b_fill_busy),
    .b_fill_we          (b_fill_we),
    .b_fill_addr        (b_fill_addr),
    .b_fill_wdata       (b_fill_wdata),
    .b_fill_done        (b_fill_done),
    // AXI4 Master Read
    .M_AXI_ARVALID      (ld_M_AXI_ARVALID),
    .M_AXI_ARREADY      (ld_M_AXI_ARREADY),
    .M_AXI_ARADDR       (ld_M_AXI_ARADDR),
    .M_AXI_ARID         (ld_M_AXI_ARID),
    .M_AXI_ARLEN        (ld_M_AXI_ARLEN),
    .M_AXI_ARSIZE       (ld_M_AXI_ARSIZE),
    .M_AXI_ARBURST      (ld_M_AXI_ARBURST),
    .M_AXI_ARLOCK       (ld_M_AXI_ARLOCK),
    .M_AXI_ARCACHE      (ld_M_AXI_ARCACHE),
    .M_AXI_ARPROT       (ld_M_AXI_ARPROT),
    .M_AXI_ARQOS        (ld_M_AXI_ARQOS),
    .M_AXI_ARUSER       (ld_M_AXI_ARUSER),
    .M_AXI_RVALID       (ld_M_AXI_RVALID),
    .M_AXI_RREADY       (ld_M_AXI_RREADY),
    .M_AXI_RDATA        (ld_M_AXI_RDATA),
    .M_AXI_RRESP        (ld_M_AXI_RRESP),
    .M_AXI_RLAST        (ld_M_AXI_RLAST),
    .M_AXI_RID          (ld_M_AXI_RID),
    .M_AXI_RUSER        (ld_M_AXI_RUSER)
  );

  // -------------------- Tile Compute --------------------
  tile_compute #(
    .TILE_SIZE  (TILE_SIZE),
    .SIDE       (TILE_SIZE),
    .ELEM_BITS  (8),
    .ACC_BITS   (32),
    .C_DEPTH    (C_DEPTH),
    .USE_DSP    (USE_DSP)
  ) u_compute (
    .clk          (M_AXI_ACLK),
    .rstn         (M_AXI_ARESETN),
    // Tile control
    .start_tile   (start_tile),
    .k_total      (compute_k_total),
    .n_eff        (compute_n_eff),
    .m_eff        (compute_m_eff),
    // Loader handshake
    .load_req     (tile_load_req),
    .k_eff        (k_eff),
    // A/B loader interface
    .a_ld_start   (a_ld_start),
    .a_ld_valid   (a_ld_valid),
    .a_ld_data    (a_ld_data),
    .b_ld_start   (b_ld_start),
    .b_ld_valid   (b_ld_valid),
    .b_ld_data    (b_ld_data),
    // C buffer fill interface (direct to bram_pingpong)
    .c_seg_words  (c_seg_words),
    .c_fill_req   (c_fill_req),
    .c_fill_busy  (c_fill_busy),
    .c_fill_we    (c_fill_we),
    .c_fill_addr  (c_fill_addr),
    .c_fill_wdata (c_fill_wdata),
    .c_fill_done  (c_fill_done),
    // Status
    .busy         (),  // unused
    .tile_done    (tile_done)
  );

  // -------------------- Tile Store --------------------
  tile_store #(
    .TILE_SIZE      (TILE_SIZE),
    .DATA_W         (DATA_W),
    .AXI_USER_WIDTH (AXI_USER_W)
  ) u_store (
    .clk                (M_AXI_ACLK),
    .rstn               (M_AXI_ARESETN),
    // Tile meta
    .base_c_addr        (store_base_c),
    .stride_c_row_bytes (store_stride_c_row),
    .i0                 (store_i0),
    .j0                 (store_j0),
    .n_eff              (store_n_eff),
    .m_eff              (store_m_eff),
    // Store control
    .start_store        (start_store),
    // C buffer consumer interface
    .cbuf_consume_req   (c_consume_req),
    .cbuf_consume_busy  (c_consume_busy),
    .cbuf_rd_addr       (c_rd_addr),
    .cbuf_rd_data       (c_rd_data),
    .cbuf_rd_en         (c_rd_en),
    .cbuf_cons_commit   (c_cons_commit),
    // Status
    .wr_busy            (),  // unused
    .wr_done            (wr_done),
    // AXI4 Master Write
    .M_AXI_AWVALID      (st_M_AXI_AWVALID),
    .M_AXI_AWREADY      (st_M_AXI_AWREADY),
    .M_AXI_AWADDR       (st_M_AXI_AWADDR),
    .M_AXI_AWID         (st_M_AXI_AWID),
    .M_AXI_AWLEN        (st_M_AXI_AWLEN),
    .M_AXI_AWSIZE       (st_M_AXI_AWSIZE),
    .M_AXI_AWBURST      (st_M_AXI_AWBURST),
    .M_AXI_AWLOCK       (st_M_AXI_AWLOCK),
    .M_AXI_AWCACHE      (st_M_AXI_AWCACHE),
    .M_AXI_AWPROT       (st_M_AXI_AWPROT),
    .M_AXI_AWQOS        (st_M_AXI_AWQOS),
    .M_AXI_AWUSER       (st_M_AXI_AWUSER),
    .M_AXI_WVALID       (st_M_AXI_WVALID),
    .M_AXI_WREADY       (st_M_AXI_WREADY),
    .M_AXI_WDATA        (st_M_AXI_WDATA),
    .M_AXI_WSTRB        (st_M_AXI_WSTRB),
    .M_AXI_WLAST        (st_M_AXI_WLAST),
    .M_AXI_WUSER        (st_M_AXI_WUSER),
    .M_AXI_BVALID       (st_M_AXI_BVALID),
    .M_AXI_BREADY       (st_M_AXI_BREADY),
    .M_AXI_BRESP        (st_M_AXI_BRESP),
    .M_AXI_BID          (st_M_AXI_BID),
    .M_AXI_BUSER        (st_M_AXI_BUSER)
  );

endmodule
