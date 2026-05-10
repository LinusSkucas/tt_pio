/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_LinusSkucas_pio (
  input  wire [7:0] ui_in,    // Dedicated inputs
  output wire [7:0] uo_out,   // Dedicated outputs
  input  wire [7:0] uio_in,   // IOs: Input path
  output wire [7:0] uio_out,  // IOs: Output path
  output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
  input  wire       ena,      // always 1 when the design is powered, so you can ignore it
  input  wire       clk,      // clock
  input  wire       rst_n     // reset_n - low to reset
);

  // All output pins must be assigned. If not used, assign to 0.
  assign uo_out  = ui_in + uio_in;  // Example: ou_out is the sum of ui_in and uio_in
  assign uio_out = 0;
  assign uio_oe  = 0;

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, clk, rst_n, 1'b0};

endmodule

module tiny_pio (
  input logic clk, rst_n,
  input logic load
);

  logic [3:0] pc, next_pc, addr, load_addr;

  assign addr = (load) ? load_addr : pc;
endmodule

typedef enum {JMP, WAIT, IN, OUT, PUSH, MOV, PULL, MOV_TORX, MOV_FRRX, IRQ, SET} op_code_t;
typedef enum logic [2:0] {
  JMP_ALWAYS = 3'b000,
  JMP_X_ZERO = 3'b001,
  JMP_X_DEC = 3'b010,
  JMP_Y_ZERO = 3'b011,
  JMP_Y_DEC = 3'b100,
  JMP_X_NEQ_Y = 3'b101,
  JMP_PIN = 3'b110,
  JMP_OSRE = 3'b111  // OSR NOT empty
} jmp_cond_t;

typedef enum logic [1:0] {
  WAIT_GPIO = 2'b00,
  WAIT_PIN = 2'b01,
  WAIT_IRQ = 2'b10,
  WAIT_JMPPIN = 2'b11
} wait_src_t;

typedef enum logic [2:0] {
  SRC_PINS = 3'b000,
  SRC_X = 3'b001,
  SRC_Y = 3'b010,
  SRC_NULL = 3'b011,
  SRC_RESERVED = 3'b100,
  SRC_STATUS = 3'b101,  // Only for MOV: reserved for IN instr
  SRC_ISR = 3'b110,
  SRC_OSR = 3'b111
} src_sel_t;  // for IN and MOV

typedef enum logic [2:0] {
    OUT_DST_PINS = 3'b000,
    OUT_DST_X = 3'b001,
    OUT_DST_Y = 3'b010,
    OUT_DST_NULL = 3'b011,
    OUT_DST_PINDIRS = 3'b100,
    OUT_DST_PC = 3'b101,
    OUT_DST_ISR = 3'b110,
    OUT_DST_EXEC = 3'b111
} out_dst_t;

typedef enum logic [2:0] {
  MOV_DST_PINS = 3'b000,
  MOV_DST_X = 3'b001,
  MOV_DST_Y = 3'b010,
  MOV_DST_PINDIRS = 3'b011,
  MOV_DST_EXEC = 3'b100,
  MOV_DST_PC = 3'b101,
  MOV_DST_ISR = 3'b110,
  MOV_DST_OSR = 3'b111
} mov_dst_t;

typedef enum logic [1:0] {
  MOV_OP_NONE = 2'b00,
  MOV_OP_INVERT = 2'b01,
  MOV_OP_BITREV = 2'b10,
  MOV_OP_RSVD = 2'b11
} mov_op_t;

typedef enum logic [2:0] {
  SET_DST_PINS = 3'b000,
  SET_DST_X = 3'b001,
  SET_DST_Y = 3'b010,
  SET_DST_RSVD3 = 3'b011,
  SET_DST_PINDIRS = 3'b100,
  SET_DST_RSVD5 = 3'b101,
  SET_DST_RSVD6 = 3'b110,
  SET_DST_RSVD7 = 3'b111
} set_dst_t;

module decoder (
  input logic [15:0] instr,
  input logic [2:0] config_sideset_cnt,
  output op_code_t op,
  output [4:0] jump_addr,
  output [2:0] jump_cond,
  output logic wait_pol,
  output wait_src_t wait_src,
  output logic [4:0] wait_idx,
  output src_sel_t src_sel,  // IN and MOV
  output logic [5:0] bit_cnt,
  output out_dst_t out_dst,
  output logic fifo_block,  // Block if rx is full/tx is empty
  output logic fifo_threshold,  // Do nothing unless output shift count has reached threshold (rx:isfull/tx:isempty)
  output logic [2:0] mov_idx,
  output logic mov_idxi,  // index by immediate
  output mov_dst_t mov_dst,
  output mov_op_t mov_op,
  output set_dst_t set_dst,
  output logic [4:0] set_data,
  output logic [4:0] delay_cnt, sideset_data
);

  always_comb begin
    src_sel = SRC_NULL;

    unique case (instr[15:13])
      3'b000: op = JMP;
      3'b001: op = WAIT;
      3'b010: begin
        op = IN;
        src_sel = src_sel_t'(instr[7:5]);
      end
      3'b011: op = OUT;
      3'b100: unique case ({instr[7], instr[4]})
        2'b00: op = PUSH;
        2'b01: op = MOV_TORX;
        2'b10: op = PULL;
        2'b11: op = MOV_FRRX;
      endcase
      3'b101: begin
        op = MOV;
        src_sel = src_sel_t'(instr[2:0]);
      end
      3'b110: op = IRQ;
      3'b111: op = SET;
    endcase

    jump_addr = instr[4:0];
    jump_cond = instr[7:5];  // don't care about making even more enums now

    wait_pol = instr[7];
    wait_src = wait_src_t'(instr[6:5]);
    wait_idx = instr[4:0];

    bit_cnt = instr[4:0] == 0 ? 32 : {1'b0, instr[4:0]};
    out_dst = out_dst_t'(instr[7:5]);

    fifo_block = instr[5];
    fifo_threshold = instr[6];

    mov_idxi = instr[3];
    mov_idx = instr[2:0];

    mov_dst = mov_dst_t'(instr[7:5]);
    mov_op = mov_op_t'(instr[4:3]);

    // IRQs not supported in this implementation

    set_dst = set_dst_t'(instr[7:5]);
    set_data = instr[4:0];

    // Sideset/Delay
    delay_cnt = instr[12:8] & (5'h1F >> config_sideset_cnt);  // 5 - config for delay
    sideset_data = instr[12:8] >> (5 - config_sideset_cnt);  // 5 MSB configured by sidestep count
  end
endmodule

module pc_handler (
  input logic clk, rst_n,
  input op_code_t op,
  input logic [3:0] jump_addr,  // Ignore MSB
  input logic [2:0] jump_cond,
  input logic [31:0] X_scratch, Y_scratch,
  input logic [2:0] config_jmp_pin_idx,  // Ignore MSB index
  input logic [7:0] gpio_in,
  input logic [5:0] osr_shift_cnt, config_osr_pull_thresh, isr_shift_cnt, config_isr_push_thresh,
  input logic wait_pol,
  input logic [4:0] wait_idx,
  input wait_src_t wait_src,
  input logic config_autopush, config_autopull,
  input logic fifo_block,
  input logic [1:0] rx_fifo_cnt, tx_fifo_cnt,
  input logic [4:0] delay_cnt,
  input logic [3:0] config_wrap_bottom, config_wrap_top,
  output logic [3:0] pc
);
  // Screw your irqs

  logic [3:0] next_pc, incremented_pc;
  logic [4:0] delay_counter, next_delay_counter;

  typedef enum {EXEC, DELAY, FLAG} delay_state_t;
  delay_state_t delay_state, next_delay_state;

  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      pc <= 0;
    end else begin
      pc <= (delay_cnt == 0 || (delay_cnt != 0 && delay_state == FLAG)) ? next_pc : pc;
    end
  end

  always_comb begin
    if (pc == config_wrap_top) begin  // "if PC matches wrap_top set pc to wrap_bottom"
      incremented_pc = config_wrap_bottom;
    end else if (pc == 4'd15) begin
      incremented_pc = 0;
    end else begin
      incremented_pc = pc + 1'd1;
    end
  end

  // Stalling
  always_comb begin
    case (op)
      JMP: begin
        unique case (jump_cond)
          3'b000: next_pc = jump_addr;  // Always
          3'b001: next_pc = (X_scratch == 0) ? jump_addr : incremented_pc;  // X == 0
          3'b010: next_pc = |X_scratch ? jump_addr : incremented_pc;  // X-- != 0
          3'b011: next_pc = (Y_scratch == 0) ? jump_addr : incremented_pc;  // Y == 0
          3'b100: next_pc = |Y_scratch ? jump_addr : incremented_pc;  // Y-- != 0
          3'b101: next_pc = (X_scratch != Y_scratch) ? jump_addr : incremented_pc;  // X != Y
          3'b110: next_pc = gpio_in[config_jmp_pin_idx] ? jump_addr : incremented_pc;  // Configed jump pin high
          3'b111: next_pc = (config_osr_pull_thresh > osr_shift_cnt) ? jump_addr : incremented_pc;  // if the threshold > osr shift: osr not empty
        endcase
      end
      WAIT: begin
        case (wait_src)
          WAIT_GPIO, WAIT_PIN: next_pc = (gpio_in[wait_idx[2:0]] == wait_pol) ? incremented_pc : pc;  // Wait for gpio
          WAIT_JMPPIN: next_pc = ((gpio_in[config_jmp_pin_idx]) == wait_pol) ? incremented_pc : pc;  // Wait for the jump pin
          default: next_pc = incremented_pc;  // idc about your irqs, wait_gpio and wait_pin are going to be the same too
        endcase
      end
      PUSH: next_pc = (rx_fifo_cnt == 2'd3 && fifo_block) ? pc : incremented_pc;  // Blocks when rx fifo (all 4 words) is full
      PULL: next_pc = (tx_fifo_cnt == 0 && fifo_block) ? pc : incremented_pc;  // Blocks when tx fof is empty
      OUT: next_pc = (config_autopull && osr_shift_cnt == config_osr_pull_thresh) ? pc : incremented_pc;  // Block when autopull and osr is at threshold
      IN: next_pc = (config_autopush && isr_shift_cnt == config_isr_push_thresh) ? pc : incremented_pc;
      // Boring normal increment
      default: next_pc = incremented_pc;
    endcase
  end

  // Delays
  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      delay_counter <= 0;
      delay_state <= EXEC;
    end else begin
      delay_counter <= next_delay_counter;
      delay_state <= next_delay_state;
    end
  end

  always_comb begin
    next_delay_state = delay_state;
    next_delay_counter = delay_counter;
    unique case (delay_state)
      EXEC: begin
        if (delay_cnt != 0 && pc != next_pc) begin  // start delay when there is a delay and not stalling
          next_delay_state = DELAY;
          next_delay_counter = delay_cnt - 1'd1;
        end
      end
      DELAY: begin
        if (delay_counter == 0) begin
          next_delay_state = FLAG;
        end else begin
          next_delay_counter = delay_counter - 1;
        end
      end
      FLAG: begin
        next_delay_state = EXEC;
      end
    endcase
  end
endmodule

module sm_datapath (
  input logic clk, rst_n,
  input logic [3:0] pc,
  input logic [7:0] gpio_in,
  input op_code_t op,
  input jmp_cond_t jmp_cond,
  input logic config_osr_shift_dir, config_isr_shift_dir,
  input logic config_autopush,
  input logic [5:0] config_isr_push_thresh,
  input src_sel_t src_sel_in_mov,
  input logic [5:0] bit_cnt
);

  logic [31:0] X_scratch, next_X_scratch, Y_scratch, next_Y_scratch;
  logic [31:0] tx_fifo_in, tx_fifo_out, rx_fifo_in, rx_fifo_out;
  logic tx_fifo_shift_en, rx_fifo_shift_en;

  logic load_osr_en, shift_osr_en, osr_serial_out;
  logic load_isr_en, shift_isr_en;

  logic [5:0] input_shift_count, next_input_shift_count;
  logic [31:0] osr_out;

  // Fifos are on ozempic, combining them to make them bigger is not supported in this implementation
  xtra_flexy_sr #(.DEPTH(4), .SIZE(32), .RESET_VAL(0)) tx_fifo(.clk(clk), .rst_n(rst_n), .serial_in(tx_fifo_in), .load_en(0), .shift_en(tx_fifo_shift_en), .shift_dir(1), .parallel_in(0), .serial_out(tx_fifo_out), .parallel_out());
  xtra_flexy_sr #(.DEPTH(4), .SIZE(32), .RESET_VAL(0)) rx_fifo(.clk(clk), .rst_n(rst_n), .serial_in(rx_fifo_in), .load_en(0), .shift_en(rx_fifo_shift_en), .shift_dir(1), .parallel_in(0), .serial_out(rx_fifo_out), .parallel_out());

  // ISR/OSR
  logic [31:0] osr, next_osr, osr_in_data;
  logic [31:0] isr, next_isr, isr_in_data;

  // Scratch Registers
  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      X_scratch <= 0;
      Y_scratch <= 0;
    end else begin
      X_scratch <= next_X_scratch;
      Y_scratch <= next_Y_scratch;
    end
  end

  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      isr <= 0;
      input_shift_count <= 0;
    end else begin
      isr <= next_isr;
      input_shift_count <= next_input_shift_count;  // saturates at 32
    end
  end

  always_comb begin
    next_X_scratch = X_scratch;
    next_Y_scratch = Y_scratch;
    load_isr_en = 0;
    next_input_shift_count = input_shift_count;

    case (op)
      JMP: begin
        case (jmp_cond)
          JMP_X_DEC: next_X_scratch = X_scratch - 1;
          JMP_Y_DEC: next_Y_scratch = Y_scratch - 1;
          default: begin
            next_X_scratch = X_scratch;
            next_Y_scratch = Y_scratch;
          end
        endcase
      end

      IN: begin
        case (src_sel_in_mov)
          SRC_PINS: isr_in_data = {24'b0, gpio_in};
          SRC_X: isr_in_data = X_scratch;
          SRC_Y: isr_in_data = Y_scratch;
          SRC_NULL: isr_in_data = 32'b0;
          SRC_ISR:  isr_in_data = isr;
          SRC_OSR:  isr_in_data = osr;
          default: isr_in_data = 32'b0;
        endcase

        if (bit_cnt == 6'd32) begin
          next_isr = isr_in_data;
        end else if (config_isr_shift_dir) begin  // shift right
          next_isr = (isr >> bit_cnt) | (isr_in_data << (6'd32 - bit_cnt));
        end else begin  // shift left
          next_isr = (isr << bit_cnt) | (isr_in_data & ~(32'hFFFFFFFF << bit_cnt));
        end

        next_input_shift_count = (input_shift_count + bit_cnt > 6'd32) ? 6'd32 : input_shift_count + bit_cnt;

        if (config_autopush && input_shift_count >= config_isr_push_thresh) begin  // Autopush
          
        end
      end
    endcase
  end

endmodule

module instr_mem (
  input logic clk, rst_n,
  input logic load, serial_in,
  input logic [3:0] addr,
  output logic [15:0] instr
);

logic [15:0][15:0] mem;
localparam NOP = 16'hA042;

always_ff @(posedge clk, negedge rst_n) begin
  if (!rst_n) begin
    mem <= {16{NOP}};  // nop
  end else if (load) begin
    mem[addr] <= {mem[addr][14:0], serial_in};  // Shift in at addr
  end
end

assign instr = mem[addr];

endmodule

// Hi Eric: 6 7
module xtra_flexy_sr #(
  parameter DEPTH = 8,
  parameter SIZE = 1,
  parameter RESET_VAL = 1'b0
) (
  input logic clk, rst_n,
  input logic [SIZE-1:0] serial_in,
  input logic load_en, shift_en, shift_dir,  // shift_dir: 0 for Left, 1 for Right
  input logic [DEPTH-1:0][SIZE-1:0] parallel_in,
  output logic serial_out,
  output logic [DEPTH-1:0][SIZE-1:0] parallel_out
);

  logic [DEPTH-1:0][SIZE-1:0] shift_in;

  always_ff @(posedge clk, negedge rst_n) begin
    if(!rst_n) begin
      parallel_out <= {DEPTH*SIZE{RESET_VAL}};
    end else begin
      parallel_out <= shift_in;
    end
  end

  always_comb begin
    if (load_en) begin
      shift_in = parallel_in;
    end else if (shift_en) begin
      if (shift_dir) begin
        shift_in = {serial_in, parallel_out[DEPTH-1:1]};
      end else begin
        shift_in = {parallel_out[DEPTH-2:0], serial_in};
      end
    end else begin
      shift_in = parallel_out;
    end

    if(shift_dir) begin
      serial_out = parallel_out[DEPTH-1];
    end else begin
      serial_out = parallel_out[0];
    end
  end
endmodule
