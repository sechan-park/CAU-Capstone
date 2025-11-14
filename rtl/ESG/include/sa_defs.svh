//----------------------------------------------------------------+
// Project: Capstone Design for graduation
// Module: sa_defs (SystemVerilog header file)
// Description:
//		Definitions for the system
//
// Last Updated: 2025-11-06 (by Jimin Hwang)
//----------------------------------------------------------------+

`ifndef SA_DEFS_SVH
`define SA_DEFS_SVH

//------------------------------------------------------------------------------
// Assertions (simulation-only)
//------------------------------------------------------------------------------
`ifdef SYNTHESIS
  `define SA_ASSERT(EXPR, MSG)
  `define SA_ASSERT_KNOWN(SIG, MSG)
  `define SA_ASSERT_RANGE(SIG, LO, HI, MSG)
`else
  `define SA_ASSERT(EXPR, MSG) \
    assert (EXPR) else $fatal(1, {"ASSERT: ", MSG});

  `define SA_ASSERT_KNOWN(SIG, MSG) \
    assert (!$isunknown(SIG)) else $fatal(1, {"X/Z detected: ", MSG});

  `define SA_ASSERT_RANGE(SIG, LO, HI, MSG) \
    assert ( (SIG) >= (LO) && (SIG) <= (HI) ) else $fatal(1, {"Out-of-range: ", MSG});
`endif

//------------------------------------------------------------------------------
// Handshake helpers
//------------------------------------------------------------------------------
`define SA_FIRE(VALID, READY)   ( (VALID) && (READY) )
`define SA_BUSY(VALID, READY)   ( (VALID) && !(READY) )
`define SA_IDLE(VALID, READY)   ( !(VALID) && !(READY) )

// VALID must hold high until READY goes high (simulation check)
`ifndef SYNTHESIS
  `define SA_VALID_HOLD(CLK, RSTN, VALID, READY) \
    assert property (@(posedge (CLK)) disable iff (~(RSTN)) \
      ( (VALID) && !(READY) ) |=> (VALID) );
`else
  `define SA_VALID_HOLD(CLK, RSTN, VALID, READY)
`endif

//------------------------------------------------------------------------------
// Small utilities
//------------------------------------------------------------------------------
`define SA_MIN(A,B)            ( ((A) < (B)) ? (A) : (B) )
`define SA_MAX(A,B)            ( ((A) > (B)) ? (A) : (B) )
`define SA_CLAMP(X,LO,HI)      ( ((X) < (LO)) ? (LO) : (((X) > (HI)) ? (HI) : (X)) )
`define SA_ONEHOT0(V)          ( ((V) == '0) || $onehot(V) )

`endif // SA_DEFS_SVH
