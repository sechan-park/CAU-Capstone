//----------------------------------------------------------------+
// Project: Capstone Design
// Module : bram_pingpong
// Description:
//   Ping-pong (double-buffer) wrapper around two BRAM IP instances.
//   - Bank0 / Bank1 : each is a BRAM (B or C size depending on USE_CONS_COMMIT)
//   - One bank is being filled (Producer), the other is being consumed (Consumer)
//   - When a segment is fully filled & consumed, banks swap roles.
//   - Read latency = 1 cycle (BRAM output directly used, no extra FF).
//
//   NOTE:
//   - For sa_engine_top:
//       USE_CONS_COMMIT = 1, DEPTH = 49152 -> uses sa_bramB_32x49152
//       USE_CONS_COMMIT = 0, DEPTH = 64    -> uses sa_bramC_32x64
// Last Updated: 2025-11-13 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns/1ps

import sa_params_pkg::*;
`include "sa_defs.svh"

module bram_pingpong #(
  parameter int DATA_W          = AXI_DATA_WIDTH,
  parameter int DEPTH           = 64,                 // words per bank
  parameter int ADDR_W          = $clog2(DEPTH),
  parameter bit USE_CONS_COMMIT = 1'b1                // 1: use cons_commit, 0: use rd_en counting
)(
  input  logic                 clk,
  input  logic                 rstn,

  // Segment length in words (for last block smaller than DEPTH). If 0 -> DEPTH
  input  logic [31:0]          seg_words,

  // Producer (fill) interface
  input  logic                 fill_req,     // pulse to start a new fill segment
  output logic                 fill_busy,
  input  logic                 fill_we,      // write one word
  input  logic [ADDR_W-1:0]    fill_addr,    // word address within bank
  input  logic [DATA_W-1:0]    fill_wdata,
  output logic                 fill_done,    // pulse when segment finished

  // Consumer (read) interface
  input  logic                 consume_req,  // pulse to arm active bank
  output logic                 consume_busy,
  input  logic [ADDR_W-1:0]    rd_addr,      // read address
  input  logic                 rd_en,        // read enable for internal counting mode
  output logic [DATA_W-1:0]    rd_rdata,     // 1-cycle data from BRAM
  input  logic                 cons_commit,  // external completion pulse
  output logic                 consume_done, // pulse on segment consumed

  // Bank selection (observability)
  output logic                 bank_sel      // 0: bank0 ACTIVE, 1: bank1 ACTIVE
);

  // ---------------------------------------------------------------------------
  // Bank states
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] { S_EMPTY, S_FILL, S_FULL, S_ACTIVE } bstate_e;
  bstate_e st0, st1;

  logic        filling;
  logic        fill_bank;      // 0/1 : which bank is currently being filled
  logic [31:0] fill_count;

  logic        consuming;
  logic [31:0] cons_count;

  // BRAM read data from each bank (1-cycle latency from addr -> data)
  logic [DATA_W-1:0] bank0_rdata;
  logic [DATA_W-1:0] bank1_rdata;

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------
  function logic [31:0] eff_seg_words(input logic [31:0] sw);
    return (sw==32'd0) ? DEPTH : sw;
  endfunction

  function logic has_empty();
    return (st0==S_EMPTY) || (st1==S_EMPTY);
  endfunction

  function logic choose_empty_bank();
    if (st0==S_EMPTY) return 1'b0; else return 1'b1;
  endfunction

  function logic has_full();
    return (st0==S_FULL) || (st1==S_FULL);
  endfunction

  function logic choose_full_bank();
    if (st0==S_FULL) return 1'b0; else return 1'b1;
  endfunction

  // ---------------------------------------------------------------------------
  // BRAM IP instances (two banks)
  //   - Port A : write (fill path)
  //   - Port B : read  (consume path)
  // ---------------------------------------------------------------------------

  // Write enables per bank (Port A)
  wire we_bank0 = filling && (fill_bank == 1'b0) && fill_we;
  wire we_bank1 = filling && (fill_bank == 1'b1) && fill_we;

  // Read enables per bank (Port B)
  wire ren_bank0 = consuming && (bank_sel == 1'b0);
  wire ren_bank1 = consuming && (bank_sel == 1'b1);

  generate
    if (USE_CONS_COMMIT) begin : gen_b_bram
      // -----------------------------------------------------------------------
      // B buffer BRAMs : 32 x 49152 (bank0 / bank1)
      // -----------------------------------------------------------------------
      sa_bramB_32x49152 u_bank0 (
        // Port A (write)
        .clka  (clk),
        .ena   (we_bank0),              // enable only when writing this bank
        .wea   (we_bank0),
        .addra (fill_addr),
        .dina  (fill_wdata),
        // Port B (read)
        .clkb  (clk),
        .enb   (ren_bank0),
        .web   ('0),                    // no write on port B
        .addrb (rd_addr),
        .dinb  ('0),
        .doutb (bank0_rdata)
      );

      sa_bramB_32x49152 u_bank1 (
        // Port A (write)
        .clka  (clk),
        .ena   (we_bank1),
        .wea   (we_bank1),
        .addra (fill_addr),
        .dina  (fill_wdata),
        // Port B (read)
        .clkb  (clk),
        .enb   (ren_bank1),
        .web   ('0),
        .addrb (rd_addr),
        .dinb  ('0),
        .doutb (bank1_rdata)
      );
    end else begin : gen_c_bram
      // -----------------------------------------------------------------------
      // C buffer BRAMs : 32 x 64 (bank0 / bank1)
      // -----------------------------------------------------------------------
      sa_bramC_32x64 u_bank0 (
        // Port A (write)
        .clka  (clk),
        .ena   (we_bank0),
        .wea   (we_bank0),
        .addra (fill_addr),
        .dina  (fill_wdata),
        // Port B (read)
        .clkb  (clk),
        .enb   (ren_bank0),
        .web   ('0),
        .addrb (rd_addr),
        .dinb  ('0),
        .doutb (bank0_rdata)
      );

      sa_bramC_32x64 u_bank1 (
        // Port A (write)
        .clka  (clk),
        .ena   (we_bank1),
        .wea   (we_bank1),
        .addra (fill_addr),
        .dina  (fill_wdata),
        // Port B (read)
        .clkb  (clk),
        .enb   (ren_bank1),
        .web   ('0),
        .addrb (rd_addr),
        .dinb  ('0),
        .doutb (bank1_rdata)
      );
    end
  endgenerate

  // ---------------------------------------------------------------------------
  // Read data mux (NO extra register → overall 1-cycle latency)
  //   rd_addr asserted at cycle N
  //   bank*_rdata valid at cycle N+1 (from BRAM)
  //   rd_rdata is just combinational mux of bank*_rdata
  // ---------------------------------------------------------------------------
  always_comb begin
    if (!consuming) begin
      rd_rdata = '0;
    end else begin
      rd_rdata = (bank_sel == 1'b0) ? bank0_rdata : bank1_rdata;
    end
  end

  // ---------------------------------------------------------------------------
  // Main FSM (fill / consume / bank state)
  //
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      st0          <= S_EMPTY;
      st1          <= S_EMPTY;
      bank_sel     <= 1'b0;

      // fill side
      filling      <= 1'b0;
      fill_bank    <= 1'b0;
      fill_count   <= 32'd0;
      fill_busy    <= 1'b0;
      fill_done    <= 1'b0;

      // consume side
      consuming    <= 1'b0;
      cons_count   <= 32'd0;
      consume_busy <= 1'b0;
      consume_done <= 1'b0;
    end else begin
      fill_done    <= 1'b0;
      consume_done <= 1'b0;

      // ---------- Fill path ----------
      if (!filling) begin
        fill_busy <= 1'b0;
        if (fill_req && has_empty()) begin
          fill_bank  <= choose_empty_bank();
          filling    <= 1'b1;
          fill_busy  <= 1'b1;
          fill_count <= 32'd0;
          if (choose_empty_bank()==1'b0) st0 <= S_FILL; else st1 <= S_FILL;
        end
      end else begin
        fill_busy <= 1'b1;
        if (fill_we) begin
          fill_count <= fill_count + 1;
          if (fill_count + 1 >= eff_seg_words(seg_words)) begin
            // segment filled
            filling   <= 1'b0;
            fill_busy <= 1'b0;
            fill_done <= 1'b1;
            if (fill_bank==1'b0) st0 <= S_FULL; else st1 <= S_FULL;
          end
        end
      end

      // ---------- Consume path ----------
      if (!consuming) begin
        consume_busy <= 1'b0;
        if (consume_req && has_full()) begin
          bank_sel     <= choose_full_bank();
          consuming    <= 1'b1;
          consume_busy <= 1'b1;
          cons_count   <= 32'd0;
          if (choose_full_bank()==1'b0) st0 <= S_ACTIVE; else st1 <= S_ACTIVE;
        end
      end else begin
        consume_busy <= 1'b1;
        if (USE_CONS_COMMIT) begin
          // External completion pulse
          if (cons_commit) begin
            consuming    <= 1'b0;
            consume_busy <= 1'b0;
            consume_done <= 1'b1;
            if (bank_sel==1'b0) st0 <= S_EMPTY; else st1 <= S_EMPTY;
          end
        end else begin
          // Internal rd_en counting
          if (rd_en) begin
            cons_count <= cons_count + 1;
            if (cons_count + 1 >= eff_seg_words(seg_words)) begin
              consuming    <= 1'b0;
              consume_busy <= 1'b0;
              consume_done <= 1'b1;
              if (bank_sel==1'b0) st0 <= S_EMPTY; else st1 <= S_EMPTY;
            end
          end
        end
      end
    end
  end

endmodule
