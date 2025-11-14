//----------------------------------------------------------------+
// Project: Capstone Design
// Module : tile_compute (SystemVerilog module)
// Description:
//   Integrated tile compute module with embedded pe_array_8x8:
//   - Segments K in chunks of T(=TILE_SIZE) and accumulates across segments.
//   - First segment only: acc_clr=1 to clear PE accumulators.
//   - After final segment: drains result to C bram_pingpong via fill interface.
//   - Expects loader to 0-pad A/B when k_eff<T and to honor n_eff/m_eff masks.
//
// Integration:
//   - Embeds pe_array_8x8 instance internally (configurable via USE_DSP).
//   - Loader interface: load_req/k_eff → external loader → a_ld_*/b_ld_* streams in.
//   - C buffer fill interface: directly connects to bram_pingpong producer side.
//   - tile_store later reads from C bram_pingpong (consumer) → AXI write.
//
// Last Updated: 2025-11-12 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns/1ps

module tile_compute #(
  parameter int TILE_SIZE = 8,
  parameter int SIDE      = 8,
  parameter int ELEM_BITS = 8,
  parameter int ACC_BITS  = 32,
  parameter int C_DEPTH   = 64,    // C buffer depth (TILE_SIZE*TILE_SIZE)
  parameter bit USE_DSP   = 1'b1
)(
  input  logic                    clk,
  input  logic                    rstn,

  // Tile meta/control
  input  logic                    start_tile,
  input  logic [15:0]             k_total,      // total K for this tile
  input  logic [3:0]              n_eff,        // 1..TILE_SIZE
  input  logic [3:0]              m_eff,        // 1..TILE_SIZE

  // Loader handshake
  output logic                    load_req,
  output logic [3:0]              k_eff,        // min(T, k_rem)

  // A/B loader interface (from tile_loader)
  input  logic                    a_ld_start,
  input  logic                    a_ld_valid,
  input  logic [31:0]             a_ld_data,
  input  logic                    b_ld_start,
  input  logic                    b_ld_valid,
  input  logic [31:0]             b_ld_data,

  // C buffer fill interface (to C bram_pingpong producer side)
  output logic [31:0]             c_seg_words,  // segment size (fixed 64)
  output logic                    c_fill_req,
  input  logic                    c_fill_busy,
  output logic                    c_fill_we,
  output logic [$clog2(C_DEPTH)-1:0] c_fill_addr,
  output logic [ACC_BITS-1:0]     c_fill_wdata,
  input  logic                    c_fill_done,

  // Status
  output logic                    busy,
  output logic                    tile_done
);

  // ---------------------------------------------------------------------------
  // Internal signals for pe_array_8x8 connection
  // ---------------------------------------------------------------------------
  logic                    start_pe;      // internal: tile_compute FSM → pe_array
  logic                    acc_clr;       // internal: tile_compute FSM → pe_array
  logic                    pe_done;       // internal: pe_array → tile_compute FSM
  logic                    c_drain_req;   // internal: tile_compute FSM → pe_array
  logic                    ld_done_pe;    // internal: pe_array → tile_compute (via ld_done input)
  
  // PE array drain stream (internal)
  logic                    pe_c_busy;     // internal: PE array is draining
  logic                    pe_c_valid;    // internal: drain data valid
  logic [ACC_BITS-1:0]     pe_c_data;     // internal: drain data
  logic                    pe_c_last;     // internal: last drain word

  // ---------------------------------------------------------------------------
  // State / counters
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    S_IDLE, S_PREP, S_LOAD, S_RUN, S_WAIT, S_DRAIN, S_DONE
  } state_e;

  state_e st, st_n;
  logic [15:0] k_rem;
  logic [15:0] k_rem_n;
  logic        first_seg;
  logic        first_seg_n;

  // ---------------------------------------------------------------------------
  // PE Array Instance (8x8 systolic array)
  // ---------------------------------------------------------------------------
  pe_array_8x8 #(
    .SIDE      (SIDE),
    .ELEM_BITS (ELEM_BITS),
    .ACC_BITS  (ACC_BITS),
    .USE_DSP   (USE_DSP),
    .K_CYCLES  (TILE_SIZE)
  ) u_pe_array (
    .clk         (clk),
    .rstn        (rstn),
    .start       (start_pe),
    .acc_clr     (acc_clr),
    .done        (pe_done),
    .a_ld_start  (a_ld_start),
    .a_ld_valid  (a_ld_valid),
    .a_ld_data   (a_ld_data),
    .b_ld_start  (b_ld_start),
    .b_ld_valid  (b_ld_valid),
    .b_ld_data   (b_ld_data),
    .ld_done     (ld_done_pe),
    .c_drain_req (c_drain_req),
    .c_busy      (pe_c_busy),    // internal signal
    .c_valid     (pe_c_valid),   // internal signal
    .c_data      (pe_c_data),    // internal signal
    .c_last      (pe_c_last)     // internal signal
  );

  // combinational default outputs
  always_comb begin
    st_n         = st;
    k_rem_n      = k_rem;
    first_seg_n  = first_seg;
    // outs
    load_req     = 1'b0;
    k_eff        = '0;
    start_pe     = 1'b0;
    acc_clr      = 1'b0;
    c_drain_req  = 1'b0;
    tile_done    = 1'b0;

    case (st)
      // Idle state: wait for start_tile signal
      S_IDLE: begin
        if (start_tile) begin
          st_n        = S_PREP;
        end
      end

      // Prepare state: compute next k_eff
      S_PREP: begin
        // compute next k_eff
        if (k_rem == 0) begin
          st_n = S_DRAIN;
        end else begin
          k_eff = (k_rem >= TILE_SIZE) ? TILE_SIZE[3:0] : k_rem[3:0];
          st_n  = S_LOAD;
        end
      end

      // Load state: request loader fill for next segment
      S_LOAD: begin
        // request loader fill for next segment
        load_req = 1'b1;
        k_eff    = (k_rem >= TILE_SIZE) ? TILE_SIZE[3:0] : k_rem[3:0];
        if (ld_done_pe) begin
          st_n = S_RUN;
        end
      end

      // Run state: start PE for this segment; clear only on first segment
      S_RUN: begin
        // start PE for this segment; clear only on first segment
        start_pe = 1'b1;
        acc_clr  = first_seg;
        st_n     = S_WAIT;
      end

      // Wait state: wait for PE to finish
      S_WAIT: begin
        if (pe_done) begin
          // segment finished
          k_rem_n     = k_rem - ((k_rem >= TILE_SIZE) ? TILE_SIZE : k_rem);
          first_seg_n = 1'b0;
          st_n        = (k_rem_n == 0) ? S_DRAIN : S_PREP;
        end
      end

      // Drain state: drain the result
      S_DRAIN: begin
        c_drain_req = 1'b1; // single-cycle pulse
        st_n        = S_DONE;
      end

      // Done state: tile is done
      S_DONE: begin
        tile_done   = 1'b1;
        st_n        = S_IDLE;
      end
    endcase
  end

  // registers
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      st        <= S_IDLE;
      k_rem     <= '0;
      first_seg <= 1'b0;
      busy      <= 1'b0;
    end else begin
      st    <= st_n;
      k_rem <= k_rem_n;
      first_seg <= first_seg_n;

      case (st)
        S_IDLE: begin
          if (start_tile) begin
            k_rem     <= k_total;
            first_seg <= 1'b1;
            busy      <= 1'b1;  // accept start and latch busy high
          end else begin
            busy      <= 1'b0;  // idle when no start
          end
        end
        S_DONE: begin
          // keep busy high through DONE; it will drop next cycle in IDLE
          busy <= 1'b1;
        end
        default: begin
          // PREP/LOAD/RUN/WAIT/DRAIN: busy stays high
          busy <= 1'b1;
        end
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // C Buffer Fill Logic (capture PE drain stream → bram_pingpong fill)
  // ---------------------------------------------------------------------------
  localparam int C_ADDR_W = $clog2(C_DEPTH);
  logic [C_ADDR_W-1:0] c_wr_addr;

  assign c_seg_words = 32'd64;  // Fixed for TILE_SIZE=8

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      c_fill_req   <= 1'b0;
      c_fill_we    <= 1'b0;
      c_fill_addr  <= '0;
      c_fill_wdata <= '0;
      c_wr_addr    <= '0;
    end else begin
      c_fill_we <= 1'b0;  // default

      // Start new fill when PE starts draining
      if (pe_c_busy && !c_fill_busy) begin
        c_fill_req  <= 1'b1;
        c_wr_addr   <= '0;
      end else begin
        c_fill_req <= 1'b0;
      end

      // Capture PE drain stream
      if (pe_c_valid) begin
        c_fill_we    <= 1'b1;
        c_fill_addr  <= c_wr_addr;
        c_fill_wdata <= pe_c_data;
        c_wr_addr    <= c_wr_addr + 1'b1;
      end
    end
  end

endmodule
