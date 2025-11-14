//----------------------------------------------------------------+
// Project: Capstone Design
// Module : axi_addr_gen (SystemVerilog module)
// Description:
//   Address/burst generator for AXI32 (4B/beat), fixed 16-beat policy (V1).
//   - Splits (base_addr, bytes_total) into contiguous bursts.
//   - req_valid/ready handshake with backpressure.
//   - req_last=1 on final burst; done pulses after last handshake.
//
// Last Updated: 2025-11-07 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns/1ps

import sa_params_pkg::*;
`include "sa_defs.svh"

module axi_addr_gen #(
  parameter int ADDR_W          = AXI_ADDR_WIDTH,
  parameter int P_BYTES_PER_BEAT= (AXI_DATA_WIDTH/8),
  parameter int P_MAX_BURST     = 16
)(
  input  logic                 clk,
  input  logic                 rstn,

  // Command
  input  logic                 start,        // one-shot command
  input  logic [ADDR_W-1:0]    base_addr,    // byte address (aligned to beat)
  input  logic [31:0]          bytes_total,  // total transfer bytes
  input  logic [31:0]          stride_bytes, // optional stride per tile/row (can be 0)

  // Burst output (stream of requests)
  output logic                 req_valid,
  input  logic                 req_ready,
  output logic [ADDR_W-1:0]    req_addr,
  output logic [7:0]           req_len,      // AXI: beats-1
  output logic                 req_last,

  // Command completion
  output logic                 done
);

  // ---------------------------------------------------------------------------
  // Internal: simple FSM (IDLE -> EMIT [repeat] -> DONE)
  // ---------------------------------------------------------------------------
  localparam int BEAT_BYTES = P_BYTES_PER_BEAT; // expect 4 for AXI32

  typedef enum logic [1:0] { S_IDLE, S_EMIT, S_DONE } state_e;
  state_e state;

  logic [ADDR_W-1:0] ptr;            // byte pointer
  logic [15:0]       rem_beats;      // remaining beats (words)
  logic [7:0]        cur_beats;      // beats for current burst (1..16)
  logic              fire;           // handshake

  assign fire = (req_valid && req_ready);

  // Assertions (simulation)
`ifndef SYNTHESIS
  initial begin
    if (BEAT_BYTES != 4) begin
      $fatal(1, "axi_addr_gen: V1 expects AXI32 (4B/beat)");
    end
  end
`endif

  // FSM / Outputs
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      state     <= S_IDLE;
      ptr       <= '0;
      rem_beats <= '0;
      cur_beats <= '0;

      req_valid <= 1'b0;
      req_addr  <= '0;
      req_len   <= 8'd0;
      req_last  <= 1'b0;
      done      <= 1'b0;
    end else begin
      // defaults (may be overridden below)
      done <= 1'b0;

      case (state)
        S_IDLE: begin
          req_valid <= 1'b0;
          req_last  <= 1'b0;
          if (start) begin
            // alignment checks
            `SA_ASSERT(base_addr[1:0] == 2'b00,   "axi_addr_gen: base_addr not 4B-aligned")
            `SA_ASSERT(bytes_total[1:0] == 2'b00, "axi_addr_gen: bytes_total not multiple of 4")

            ptr       <= base_addr;
            rem_beats <= bytes_total / BEAT_BYTES;

            if (bytes_total == 32'd0) begin
              // no burst; immediately done
              state <= S_DONE;
            end else begin
              // emit first burst
              cur_beats <= ( (bytes_total / BEAT_BYTES) >= P_MAX_BURST ) ? 8'(P_MAX_BURST)
                                                                           : 8'(bytes_total / BEAT_BYTES);
              req_addr  <= base_addr;
              req_len   <= ( ( (bytes_total / BEAT_BYTES) >= P_MAX_BURST ) ? 8'(P_MAX_BURST)
                                                                           : 8'(bytes_total / BEAT_BYTES) ) - 8'd1;
              req_last  <= ( (bytes_total / BEAT_BYTES) <= P_MAX_BURST );
              req_valid <= 1'b1;
              state     <= S_EMIT;
            end
          end
        end

        S_EMIT: begin
          // hold current request stable until handshake
          req_valid <= 1'b1;
          if (fire) begin
            // advance pointer / remaining
            ptr       <= ptr + (cur_beats * BEAT_BYTES);
            rem_beats <= rem_beats - cur_beats;

            if ((rem_beats - cur_beats) == 0) begin
              // last was consumed
              req_valid <= 1'b0;
              state     <= S_DONE;
            end else begin
              // prepare next burst immediately
              cur_beats <= ((rem_beats - cur_beats) >= P_MAX_BURST) ? 8'(P_MAX_BURST)
                                                                     : 8'(rem_beats - cur_beats);
              req_addr  <= ptr + (cur_beats * BEAT_BYTES);
              req_len   <= ( ((rem_beats - cur_beats) >= P_MAX_BURST) ? 8'(P_MAX_BURST)
                                                                     : 8'(rem_beats - cur_beats) ) - 8'd1;
              req_last  <= ((rem_beats - cur_beats) <= P_MAX_BURST);
              // remain in S_EMIT with req_valid held high
            end
          end
        end

        S_DONE: begin
          req_valid <= 1'b0;
          done      <= 1'b1; // one-cycle pulse
          state     <= S_IDLE;
        end

        default: state <= S_IDLE;
      endcase
    end
  end

endmodule
