//----------------------------------------------------------------+
// Project: Capstone Design
// Module : pe_array_8x8 (SystemVerilog module)
// Description:
//   8x8 PE array with internal A/B shift and PE accumulation.
//   - Signed int8 operands, int32 accumulate inside PEs.
//   - Row-wise A injection (left edge), column-wise B injection (top edge).
//   - Asserts 'done' after K_CYCLES + (2*SIDE-2) cycles from start.
//
// Last Updated: 2025-11-10 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns/1ps

module pe_array_8x8 #(
  parameter int SIDE       = 8,
  parameter int ELEM_BITS  = 8,
  parameter int ACC_BITS   = 32,
  parameter bit USE_DSP    = 1'b1,
  // Number of K iterations to accumulate (beats per dot-product)
  parameter int K_CYCLES   = 8
)(
  input  logic                           clk,
  input  logic                           rstn,
  input  logic                           start,
  input  logic                           acc_clr,
  output logic                           done,

  input  logic                           a_ld_start,
  input  logic                           a_ld_valid,
  input  logic [31:0]                    a_ld_data,
  input  logic                           b_ld_start,
  input  logic                           b_ld_valid,
  input  logic [31:0]                    b_ld_data,
  output logic                           ld_done,
  // Result drain stream (row-major, one word per cycle)
  input  logic                           c_drain_req,
  output logic                           c_busy,
  output logic                           c_valid,
  output logic [ACC_BITS-1:0]            c_data,
  output logic                           c_last
);

  // ---------------------------------------------------------------------------
  // Internal state
  // ---------------------------------------------------------------------------
  localparam int SA_LAT = (2*SIDE) - 2; // propagation latency through systolic

  logic run;
  logic [$clog2(K_CYCLES+SA_LAT+1)-1:0] ctr;
  // Drain row/col indices
  logic [$clog2(SIDE)-1:0] dr_r, dr_c;


  // ---------------------------------------------------------------------------
  // Local tile storage (optional): A[SIDE][K], B[K][SIDE]
  // ---------------------------------------------------------------------------
  logic signed [ELEM_BITS-1:0] localA [0:SIDE-1][0:K_CYCLES-1];
  logic signed [ELEM_BITS-1:0] localB [0:K_CYCLES-1][0:SIDE-1];
  logic                        a_loaded, b_loaded;
  logic [15:0]                 a_wr_idx, b_wr_idx; // linear indices

  // Loader for A (row-major: r then k)
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      a_loaded <= 1'b0; a_wr_idx <= '0;
    end else begin
      if (a_ld_start) begin
        a_loaded <= 1'b0;
        a_wr_idx <= '0;
      end
      if (a_ld_valid && !a_loaded) begin
        // write 4 bytes: idx .. idx+3
        for (int bbyte=0; bbyte<4; bbyte++) begin
          int li = a_wr_idx + bbyte;
          if (li < SIDE*K_CYCLES) begin
            int rli = li / K_CYCLES; int kli = li % K_CYCLES;
            localA[rli][kli] <= $signed(a_ld_data[8*bbyte +: 8]);
          end
        end
        if (a_wr_idx + 4 >= SIDE*K_CYCLES) begin
          a_loaded <= 1'b1;
        end else begin
          a_wr_idx <= a_wr_idx + 4;
        end
      end
    end
  end

  // Loader for B (row-major: k then c)
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      b_loaded <= 1'b0; b_wr_idx <= '0;
    end else begin
      if (b_ld_start) begin b_loaded <= 1'b0; b_wr_idx <= '0; end
      if (b_ld_valid && !b_loaded) begin
        for (int bbyte=0; bbyte<4; bbyte++) begin
          int li = b_wr_idx + bbyte;
          if (li < K_CYCLES*SIDE) begin
            int kli = li / SIDE; int cli = li % SIDE;
            localB[kli][cli] <= $signed(b_ld_data[8*bbyte +: 8]);
          end
        end
        if (b_wr_idx + 4 >= K_CYCLES*SIDE) begin
          b_loaded <= 1'b1;
        end else begin
          b_wr_idx <= b_wr_idx + 4;
        end
      end
    end
  end

  // ld_done: convert level (a_loaded && b_loaded) to a one-cycle pulse
  logic ld_done_lvl_q;
  logic ld_done_lvl;
  assign ld_done_lvl = a_loaded && b_loaded;
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      ld_done_lvl_q <= 1'b0;
    end else begin
      // On new load sequence, clear edge history
      if (a_ld_start || b_ld_start) ld_done_lvl_q <= 1'b0;
      else                          ld_done_lvl_q <= ld_done_lvl;
    end
  end
  assign ld_done = ld_done_lvl & ~ld_done_lvl_q;

  // Mesh interconnect (neighbor pass-through)
  logic signed [ELEM_BITS-1:0] a_sig [0:SIDE-1][0:SIDE-1];
  logic signed [ELEM_BITS-1:0] b_sig [0:SIDE-1][0:SIDE-1];
  logic                       a_v_sig [0:SIDE-1][0:SIDE-1];
  logic                       b_v_sig [0:SIDE-1][0:SIDE-1];
  logic [ACC_BITS-1:0]         c_sig [0:SIDE-1][0:SIDE-1];

  // Run/ctr/done FSM
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      run  <= 1'b0;
      ctr  <= '0;
      done <= 1'b0;
      // drain reset
      c_busy  <= 1'b0;
      c_valid <= 1'b0;
      c_last  <= 1'b0;
      c_data  <= '0;
      dr_r    <= '0;
      dr_c    <= '0;
    end else begin
      done   <= 1'b0;
      c_valid<= 1'b0; // default low unless driving data
      c_last <= 1'b0;
      if (!run) begin
        if (start) begin
          run <= 1'b1;
          ctr <= '0;
        end
        // Accept drain request only when not running and not already busy
        if (!c_busy && c_drain_req) begin
          c_busy <= 1'b1;
          dr_r   <= '0;
          dr_c   <= '0;
        end
      end else begin
        ctr <= ctr + 1'b1;
        if (ctr == (K_CYCLES + SA_LAT - 1)) begin
          run  <= 1'b0;
          done <= 1'b1; // one-cycle pulse
        end
      end

      // Drain engine: read out acc_pipe row-major when busy
      if (c_busy) begin
        c_valid <= 1'b1;
        c_data  <= c_sig[dr_r][dr_c];
        c_last  <= (dr_r == SIDE-1) && (dr_c == SIDE-1);
        // advance indices
        if (dr_c == SIDE-1) begin
          dr_c <= '0;
          if (dr_r == SIDE-1) begin
            dr_r  <= '0;
            c_busy<= 1'b0; // finished draining
          end else begin
            dr_r <= dr_r + 1'b1;
          end
        end else begin
          dr_c <= dr_c + 1'b1;
        end
      end
    end
  end

  // integer r,c;
  // always_ff @(posedge clk or negedge rstn) begin
  //   if (!rstn) begin
  //     for (r=0; r<SIDE; r++) begin
  //       for (c=0; c<SIDE; c++) begin
  //         a_sig[r][c] <= '0;
  //         b_sig[r][c] <= '0;
  //         a_v_sig[r][c] <= 1'b0;
  //         b_v_sig[r][c] <= 1'b0;
  //         c_sig[r][c] <= '0;
  //       end
  //     end
  //   end else begin
  //     if (acc_clr) begin
  //       // clear accumulators at the beginning of a compute
  //       for (r=0; r<SIDE; r++) begin
  //         for (c=0; c<SIDE; c++) begin
  //           c_sig[r][c] <= '0;
  //         end
  //       end
  //     end
  //     // No array-level shift; PEs forward A/B and valids themselves. c_sig is driven by PEs below.
  //   end
  // end

  // Instantiate PEs
  genvar rr, cc;
  generate
    for (rr=0; rr<SIDE; rr++) begin : ROW
      for (cc=0; cc<SIDE; cc++) begin : COL
        // Per-PE enable: start accumulating when the data wavefront reaches (rr,cc)
        // and stop after K_CYCLES contributions.
        // localparam int RC_SUM = rr + cc;
        wire shift_en= run || start; // capture first edge on start cycle
        // Diagonal skew injection windows for edges
        wire a_win = run && (ctr >= rr) && (ctr < rr + K_CYCLES);
        wire b_win = run && (ctr >= cc) && (ctr < cc + K_CYCLES);
        // Guarded indexing for skewed edge injections to avoid negative (ctr-rr/cc)
        logic signed [ELEM_BITS-1:0] a_edge, b_edge;
        always_comb begin
          if (a_win) a_edge = localA[rr][ctr - rr];
          else       a_edge = '0;
        end
        always_comb begin
          if (b_win) b_edge = localB[ctr - cc][cc];
          else       b_edge = '0;
        end
        wire signed [ELEM_BITS-1:0] a_in_cell = (cc==0) ? a_edge : a_sig[rr][cc-1];
        wire signed [ELEM_BITS-1:0] b_in_cell = (rr==0) ? b_edge : b_sig[rr-1][cc];
        // Edge valids follow the same windows; interior cells forward neighbor valids
        wire                        a_v_in_cell = (cc==0) ? a_win : a_v_sig[rr][cc-1];
        wire                        b_v_in_cell = (rr==0) ? b_win : b_v_sig[rr-1][cc];
        if (USE_DSP) begin : DSP
          pe_int8_dsp #(.ELEM_BITS(ELEM_BITS), .ACC_BITS(ACC_BITS)) u_pe (
            .clk     (clk),
            .rstn    (rstn),
            .clr     (acc_clr),
            .shift_en(shift_en),
            .a_in    (a_in_cell),
            .b_in    (b_in_cell),
            .a_v_in  (a_v_in_cell),
            .b_v_in  (b_v_in_cell),
            .a_out   (a_sig[rr][cc]),
            .b_out   (b_sig[rr][cc]),
            .a_v_out (a_v_sig[rr][cc]),
            .b_v_out (b_v_sig[rr][cc]),
            .c_out   (c_sig[rr][cc])
          );
        end else begin : LUT
          pe_int8_lut #(.ELEM_BITS(ELEM_BITS), .ACC_BITS(ACC_BITS)) u_pe (
            .clk     (clk),
            .rstn    (rstn),
            .clr     (acc_clr),
            .shift_en(shift_en),
            .a_in    (a_in_cell),
            .b_in    (b_in_cell),
            .a_v_in  (a_v_in_cell),
            .b_v_in  (b_v_in_cell),
            .a_out   (a_sig[rr][cc]),
            .b_out   (b_sig[rr][cc]),
            .a_v_out (a_v_sig[rr][cc]),
            .b_v_out (b_v_sig[rr][cc]),
            .c_out   (c_sig[rr][cc])
          );
        end
      end
    end
  endgenerate

endmodule
