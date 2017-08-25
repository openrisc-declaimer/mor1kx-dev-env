//////////////////////////////////////////////////////////////////////
///                                                               ////
/// ORPSoC testbench                                              ////
///                                                               ////
/// Instantiate ORPSoC, monitors, provide stimulus                ////
///                                                               ////
/// Julius Baxter, julius@opencores.org                           ////
/// Jin Fei,       jm8371@gmail.com                               ////
///                                                               ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2009, 2010 Authors and OPENCORES.ORG           ////
////                                                              ////
//// This source file may be used and distributed without         ////
//// restriction provided that this copyright statement is not    ////
//// removed from the file and that any derivative work contains  ////
//// the original copyright notice and the associated disclaimer. ////
////                                                              ////
//// This source file is free software; you can redistribute it   ////
//// and/or modify it under the terms of the GNU Lesser General   ////
//// Public License as published by the Free Software Foundation; ////
//// either version 2.1 of the License, or (at your option) any   ////
//// later version.                                               ////
////                                                              ////
//// This source is distributed in the hope that it will be       ////
//// useful, but WITHOUT ANY WARRANTY; without even the implied   ////
//// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      ////
//// PURPOSE.  See the GNU Lesser General Public License for more ////
//// details.                                                     ////
////                                                              ////
//// You should have received a copy of the GNU Lesser General    ////
//// Public License along with this source; if not, download it   ////
//// from http://www.opencores.org/lgpl.shtml                     ////
////                                                              ////
//////////////////////////////////////////////////////////////////////

//
// Define to processor type
//
`define OR1200

//
// Define to running time
//
`ifndef END_TIME
`define END_TIME 500000
`endif

//
// Define to enable monitor processor system, it is a main macro
//
`define OR1200_SYSTEM_CHECKER

//
// Define to enable monitor processor stage
//
`define OR1200_MONITOR_EXEC_STATE
`define OR1200_MONITOR_EXEC_LOG_DISASSEMBLY
`define OR1200_MONITOR_SPRS
`define OR1200_MONITOR_LOOKUP
`define OR1200_DISPLAY_EXECUTED
`define OR1200_MONITOR_VERBOSE_NOPS

//
// Define to enable VCD wave dump
//
`define VCD

//
// Define to enable VPD wave dump
//
`define VPD
