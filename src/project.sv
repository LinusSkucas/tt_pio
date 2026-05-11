
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
  assign uio_out[3:0] = 0;
  assign uio_oe = {{3{~uio_in[1]}}, !uio_in[0] && !uio_in[1], 4'h0};  // when loading, uio4 will be an input

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, 1'b0};

  tiny_pio pio(.clk(clk), .rst_n(rst_n),
    .load(uio_in[0]), .load_serial_in(uio_in[4]),
    .ext_addr(uio_in[3:1]), .raw_gpio_in(ui_in), .in(uio_in[7:4]),
    .gpio_out(uo_out), .out(uio_out[7:4]));

endmodule

typedef enum {JMP, WAIT, IN, OUT, PUSH, MOV, PULL, SET} op_code_t;  // Ignoring MOV_TORX, MOV_FRRX, IRQ

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

module tiny_pio (
  input logic clk, rst_n,
  input logic load, load_serial_in,
  input logic [2:0] ext_addr,
  input logic [7:0] raw_gpio_in,
  input logic [3:0] in,
  output logic [7:0] gpio_out,
  output logic [3:0] out
);
  logic [3:0] pc;
  logic [15:0] instr;

  op_code_t op;
  logic [4:0] jump_addr;
  logic [2:0] jump_cond;
  logic wait_pol;
  wait_src_t wait_src;
  logic [4:0] wait_idx;
  src_sel_t src_sel;
  logic [5:0] bit_cnt;
  out_dst_t out_dst;
  logic fifo_block;
  logic fifo_threshold;
  mov_dst_t mov_dst;
  mov_op_t mov_op;
  set_dst_t set_dst;
  logic [4:0] set_data;
  logic sideset_en;
  logic [4:0] delay_cnt;
  logic [4:0] sideset_data;

  logic [7:0] config_clk_div;
  logic config_osr_shift_dir;
  logic config_isr_shift_dir;
  logic [5:0] config_isr_push_thresh;
  logic [5:0] config_osr_pull_thresh;
  logic [2:0] config_jmp_pin_idx;
  logic [2:0] config_sideset_cnt;
  logic [2:0] config_sideset_base;
  logic config_sideset_en;
  logic config_status_sel;
  logic [3:0] config_wrap_bottom;
  logic [3:0] config_wrap_top;
  logic config_sync_gpio;

  logic [31:0] X_scratch, Y_scratch;
  logic [5:0] output_shift_count;
  logic tx_empty, rx_full, rx_read, tx_read;

  logic [31:0] rx_fifo, tx_fifo;

  logic [3:0] imem_addr;

  always_comb begin
    if (load) begin
      if (ext_addr == 3'b000) begin
        // Stream instr
        imem_addr = raw_gpio_in[3:0];
      end else begin
        // keep pc at 0
        imem_addr = 0;
      end
    end else begin
      imem_addr = pc;
    end
  end

  instr_mem imem(.clk(clk), .rst_n(rst_n), .load(load && ext_addr == 3'b000), .serial_in(load_serial_in), .addr(imem_addr), .instr(instr));

  config_rf rf(
    .clk(clk), .rst_n(rst_n),
    .ext_addr(ext_addr),
    .load(load), .serial_in(load_serial_in),
    .tx_read(tx_read),
    .in(in),
    .rx_fifo(rx_fifo),
    .config_clk_div(config_clk_div),
    .config_osr_shift_dir(config_osr_shift_dir),
    .config_isr_shift_dir(config_isr_shift_dir),
    .config_isr_push_thresh(config_isr_push_thresh),
    .config_osr_pull_thresh(config_osr_pull_thresh),
    .config_jmp_pin_idx(config_jmp_pin_idx),
    .config_sideset_cnt(config_sideset_cnt),
    .config_sideset_base(config_sideset_base),
    .config_sideset_en(config_sideset_en),
    .config_status_sel(config_status_sel),
    .config_wrap_bottom(config_wrap_bottom),
    .config_wrap_top(config_wrap_top),
    .config_sync_gpio(config_sync_gpio),
    .tx_empty(tx_empty),
    .rx_read(rx_read),
    .tx_fifo(tx_fifo),
    .out(out)
  );

  logic [7:0] div_counter;
  logic clk_en;
  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      div_counter <= 0;
    end else if (div_counter == config_clk_div) begin
      div_counter <= 0;
    end else begin
      div_counter <= div_counter + 1'b1;
    end
  end

  assign clk_en = (div_counter == config_clk_div);

  decoder dec (
    .instr(instr),
    .config_sideset_cnt(config_sideset_cnt),
    .config_sideset_en(config_sideset_en),
    .op(op),
    .jump_addr(jump_addr),
    .jump_cond(jump_cond),
    .wait_pol(wait_pol),
    .wait_src(wait_src),
    .wait_idx(wait_idx),
    .src_sel(src_sel),
    .bit_cnt(bit_cnt),
    .out_dst(out_dst),
    .fifo_block(fifo_block),
    .fifo_threshold(fifo_threshold),
    .mov_dst(mov_dst),
    .mov_op(mov_op),
    .set_dst(set_dst),
    .set_data(set_data),
    .sideset_en(sideset_en),
    .delay_cnt(delay_cnt),
    .sideset_data(sideset_data)
  );

  // Syncronize GPIO in
  logic [7:0] gpio_stage1, gpio_stage2, gpio_in;
  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      gpio_stage1 <= 0;
      gpio_stage2 <= 0;
    end else begin
      gpio_stage1 <= raw_gpio_in;
      gpio_stage2 <= gpio_stage1;
    end
  end

  assign gpio_in = config_sync_gpio ? gpio_stage2 : raw_gpio_in;

  pc_handler pch (
    .clk(clk), .rst_n(rst_n),
    .clk_en(clk_en),
    .op(load ? JMP : op), // while loading, force JMP-always-to-0
    .jump_addr(load ? 4'd0 : jump_addr[3:0]),
    .jump_cond(load ? 3'b000 : jump_cond),
    .X_scratch(X_scratch),
    .Y_scratch(Y_scratch),
    .config_jmp_pin_idx(config_jmp_pin_idx),
    .gpio_in(gpio_in),
    .osr_shift_cnt(output_shift_count),
    .config_osr_pull_thresh(config_osr_pull_thresh),
    .wait_pol(wait_pol),
    .wait_idx(wait_idx),
    .wait_src(wait_src),
    .fifo_block(fifo_block),
    .rx_full(rx_full),
    .tx_empty(tx_empty),
    .delay_cnt(delay_cnt),
    .config_wrap_bottom(config_wrap_bottom),
    .config_wrap_top(config_wrap_top),
    .pc(pc)
  );

  sm_control smc (
    .clk(clk), .rst_n(rst_n),
    .clk_en(clk_en),
    .gpio_in(gpio_in),
    .op(load ? JMP : op),
    .jmp_cond(jump_cond),
    .config_osr_shift_dir(config_osr_shift_dir),
    .config_isr_shift_dir(config_isr_shift_dir),
    .config_isr_push_thresh(config_isr_push_thresh),
    .config_osr_pull_thresh(config_osr_pull_thresh),
    .src_sel_in_mov(src_sel),
    .out_dst(out_dst),
    .rx_read(rx_read),
    .tx_empty(tx_empty),
    .bit_cnt(bit_cnt),
    .fifo_threshold(fifo_threshold),
    .fifo_block(fifo_block),
    .tx_fifo(tx_fifo),
    .mov_dst(mov_dst),
    .mov_op(mov_op),
    .config_status_sel(config_status_sel),
    .set_dst(set_dst),
    .set_data(set_data),
    .sideset_data(sideset_data),
    .sideset_en(sideset_en),
    .config_sideset_base(config_sideset_base),
    .config_sideset_cnt(config_sideset_cnt),
    .X_scratch(X_scratch),
    .Y_scratch(Y_scratch),
    .output_shift_count(output_shift_count),
    .out_pins(gpio_out),
    .rx_fifo(rx_fifo),
    .rx_full(rx_full),
    .tx_read(tx_read)
  );

endmodule

module config_rf (
  input logic clk, rst_n,
  input logic [2:0] ext_addr,
  input logic load, serial_in,

  input logic tx_read,
  input logic [3:0] in,
  input logic [31:0] rx_fifo,

  output logic [7:0] config_clk_div,
  output logic config_osr_shift_dir,
  output logic config_isr_shift_dir,
  output logic [5:0] config_isr_push_thresh,
  output logic [5:0] config_osr_pull_thresh,
  output logic [2:0] config_jmp_pin_idx,
  output logic [2:0] config_sideset_cnt,
  output logic [2:0] config_sideset_base,
  output logic config_sideset_en,
  output logic config_status_sel,
  output logic [3:0] config_wrap_bottom,
  output logic [3:0] config_wrap_top,
  output logic config_sync_gpio,

  output logic tx_empty, rx_read,
  output logic [31:0] tx_fifo,
  output logic [3:0] out
);

  logic [41:0] config_sr;
  wire _unused;

  // could use a struct for this but laziness got in the way
  logic [7:0] next_config_clk_div;
  logic next_config_osr_shift_dir;
  logic next_config_isr_shift_dir;
  logic [5:0] next_config_isr_push_thresh;
  logic [5:0] next_config_osr_pull_thresh;
  logic [2:0] next_config_jmp_pin_idx;
  logic [2:0] next_config_sideset_cnt;
  logic [2:0] next_config_sideset_base;
  logic next_config_sideset_en;
  logic next_config_status_sel;
  logic [3:0] next_config_wrap_bottom;
  logic [3:0] next_config_wrap_top;
  logic next_config_sync_gpio;

  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      config_clk_div <= 0;
      config_osr_shift_dir <= 0;
      config_isr_shift_dir <= 0;
      config_osr_pull_thresh <= 0;
      config_isr_push_thresh <= 0;
      config_jmp_pin_idx <= 0;
      config_sideset_cnt <= 0;
      config_sideset_base <= 0;
      config_sideset_en <= 0;
      config_status_sel <= 0;
      config_wrap_bottom <= 0;
      config_wrap_top <= 0;
      config_sync_gpio <= 0;
    end else begin
      config_clk_div <= next_config_clk_div;
      config_osr_shift_dir <= next_config_osr_shift_dir;
      config_isr_shift_dir <= next_config_isr_shift_dir;
      config_osr_pull_thresh <= next_config_osr_pull_thresh;
      config_isr_push_thresh <= next_config_isr_push_thresh;
      config_jmp_pin_idx <= next_config_jmp_pin_idx;
      config_sideset_cnt <= next_config_sideset_cnt;
      config_sideset_base <= next_config_sideset_base;
      config_sideset_en <= next_config_sideset_en;
      config_status_sel <= next_config_status_sel;
      config_wrap_bottom <= next_config_wrap_bottom;
      config_wrap_top <= next_config_wrap_top;
      config_sync_gpio <= next_config_sync_gpio;
    end
  end

  // Shift everything LSB first (to the right)
  xtra_flexy_sr #(.SIZE(1), .DEPTH(42), .RESET_VAL(0)) input_config_sr(.clk(clk), .rst_n(rst_n), .serial_in(serial_in), .load_en(0), .shift_en(ext_addr == 3'b001 && load), .shift_dir(1), .parallel_in(0), .serial_out(_unused), .parallel_out(config_sr));

  always_comb begin
    if (ext_addr == 3'b010 && load) begin
      next_config_clk_div = config_sr[41:34];
      next_config_osr_shift_dir = config_sr[33];
      next_config_isr_shift_dir = config_sr[32];
      next_config_osr_pull_thresh = config_sr[31:26];
      next_config_isr_push_thresh = config_sr[25:20];
      next_config_jmp_pin_idx = config_sr[19:17];
      next_config_sideset_cnt = config_sr[16:14];
      next_config_sideset_base = config_sr[13:11];
      next_config_sideset_en = config_sr[10];
      next_config_status_sel = config_sr[9];
      next_config_wrap_bottom = config_sr[8:5];
      next_config_wrap_top = config_sr[4:1];
      next_config_sync_gpio = config_sr[0];
    end else begin
      next_config_clk_div = config_clk_div;
      next_config_osr_shift_dir = config_osr_shift_dir;
      next_config_isr_shift_dir = config_isr_shift_dir;
      next_config_osr_pull_thresh = config_osr_pull_thresh;
      next_config_isr_push_thresh = config_isr_push_thresh;
      next_config_jmp_pin_idx = config_jmp_pin_idx;
      next_config_sideset_cnt = config_sideset_cnt;
      next_config_sideset_base = config_sideset_base;
      next_config_sideset_en = config_sideset_en;
      next_config_status_sel = config_status_sel;
      next_config_wrap_bottom = config_wrap_bottom;
      next_config_wrap_top = config_wrap_top;
      next_config_sync_gpio = config_sync_gpio;
    end
  end

  // TX Fifo stuff
  logic [31:0] next_tx_fifo;
  logic [3:0] next_out;
  logic advance_addr, next_advance_addr, next_tx_empty, next_rx_read;

  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      tx_fifo <= 0;
      advance_addr <= 0;
      tx_empty <= 1;
      rx_read <= 0;
      out <= 0;
    end else begin
      tx_fifo <= next_tx_fifo;
      advance_addr <= next_advance_addr;
      tx_empty <= next_tx_empty;
      rx_read <= next_rx_read;
      out <= next_out;
    end
  end

  always_comb begin
    next_tx_fifo = tx_fifo;
    next_advance_addr = 0;
    next_tx_empty = tx_read ? 1 : tx_empty;
    next_rx_read = 0;
    next_out = out;

    case (ext_addr)
      // RX
      3'b000: begin
        if (advance_addr) begin
          next_out = rx_fifo[7:4];
        end else begin
          next_out = rx_fifo[3:0];
          next_advance_addr = 1;
        end
      end
      3'b010: begin
        if (advance_addr) begin
          next_out = rx_fifo[15:12];
        end else begin
          next_out = rx_fifo[11:8];
          next_advance_addr = 1;
        end
      end
      3'b100: begin
        if (advance_addr) begin
          next_out = rx_fifo[23:20];
        end else begin
          next_out = rx_fifo[19:16];
          next_advance_addr = 1;
        end
      end
      3'b110: begin
        if (advance_addr) begin
          next_out = rx_fifo[31:28];
          next_rx_read = 1;
        end else begin
          next_out = rx_fifo[27:24];
          next_advance_addr = 1;
        end
      end

      // TX
      3'b001: begin
        if (advance_addr) begin
          next_tx_fifo[7:4] = in;
        end else begin
          next_tx_fifo[3:0] = in;
          next_advance_addr = 1;
        end
      end
      3'b011: begin
        if (advance_addr) begin
          next_tx_fifo[15:12] = in;
        end else begin
          next_tx_fifo[11:8] = in;
          next_advance_addr = 1;
        end
      end
      3'b101: begin
        if (advance_addr) begin
          next_tx_fifo[23:20] = in;
        end else begin
          next_tx_fifo[19:16] = in;
          next_advance_addr = 1;
        end
      end
      3'b111: begin
        if (advance_addr) begin
          next_tx_fifo[31:28] = in;
          next_tx_empty = 0;
        end else begin
          next_tx_fifo[27:24] = in;
          next_advance_addr = 1;
        end
      end
      default: ;
    endcase
  end
endmodule

module decoder (
  input logic [15:0] instr,
  input logic [2:0] config_sideset_cnt,
  input logic config_sideset_en,
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
  output mov_dst_t mov_dst,
  output mov_op_t mov_op,
  output set_dst_t set_dst,
  output logic [4:0] set_data,
  output logic sideset_en,
  output logic [4:0] delay_cnt, sideset_data
);

  always_comb begin
    src_sel = SRC_NULL;

    case (instr[15:13])
      3'b000: op = JMP;
      3'b001: op = WAIT;
      3'b010: begin
        op = IN;
        src_sel = src_sel_t'(instr[7:5]);
      end
      3'b011: op = OUT;
      3'b100: op = (instr[7]) ? PULL : PUSH;
      3'b101: begin
        op = MOV;
        src_sel = src_sel_t'(instr[2:0]);
      end
      3'b111: op = SET;
      default: op = JMP;
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

    mov_dst = mov_dst_t'(instr[7:5]);
    mov_op = mov_op_t'(instr[4:3]);

    // IRQs not supported in this implementation

    set_dst = set_dst_t'(instr[7:5]);
    set_data = instr[4:0];

    // Sideset/Delay
    delay_cnt = instr[12:8] & (5'h1F >> config_sideset_cnt);  // 5 - config for delay

    // Sideset is 5 MSB configured by sidestep count
    if (config_sideset_cnt == 0) begin  // no sideset
      sideset_en = 0;
      sideset_data = 0;
    end else if (config_sideset_en) begin  // Eat the MSB to determine sideset enabledness
      sideset_en = instr[12];
      sideset_data = {1'b0, instr[11:8] >> (5 - config_sideset_cnt)};
    end else begin
      sideset_en = 1;
      sideset_data = instr[12:8] >> (5 - config_sideset_cnt);
    end
  end
endmodule

// No PC overrides from instructions
module pc_handler (
  input logic clk, rst_n,
  input logic clk_en,
  input op_code_t op,
  input logic [3:0] jump_addr,  // Ignore MSB
  input logic [2:0] jump_cond,
  input logic [31:0] X_scratch, Y_scratch,
  input logic [2:0] config_jmp_pin_idx,  // Ignore MSB index
  input logic [7:0] gpio_in,
  input logic [5:0] osr_shift_cnt, config_osr_pull_thresh,
  input logic wait_pol,
  input logic [4:0] wait_idx,
  input wait_src_t wait_src,
  input logic fifo_block,
  input logic rx_full, tx_empty,
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
    end else if (clk_en) begin
      pc <= (delay_cnt == 0 || delay_state == FLAG) ? next_pc : pc;
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
      PUSH: next_pc = (rx_full && fifo_block) ? pc : incremented_pc;  // Blocks when rx fifo (all 4 words) is full
      PULL: next_pc = (tx_empty && fifo_block) ? pc : incremented_pc;  // Blocks when tx fof is empty
      // No autopush/pull
      // Boring normal increment
      default: next_pc = incremented_pc;
    endcase
  end

  // Delays
  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      delay_counter <= 0;
      delay_state <= EXEC;
    end else if (clk_en) begin
      delay_counter <= next_delay_counter;
      delay_state <= next_delay_state;
    end
  end

  always_comb begin
    next_delay_state = delay_state;
    next_delay_counter = delay_counter;
    unique case (delay_state)
      EXEC: begin
        if (delay_cnt != 0 && pc != next_pc) begin  // start delay when there is a delay and not stalling; Don't jmp always to pc or else
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

module sm_control (
  input logic clk, rst_n,
  input logic clk_en,
  input logic [7:0] gpio_in,
  input op_code_t op,
  input logic [2:0] jmp_cond,
  input logic config_osr_shift_dir, config_isr_shift_dir,
  input logic [5:0] config_isr_push_thresh, config_osr_pull_thresh,
  input src_sel_t src_sel_in_mov,
  input out_dst_t out_dst,
  input rx_read, tx_empty,
  input logic [5:0] bit_cnt,
  input logic fifo_threshold, fifo_block,  // nop unless thresh met for push/pull; Block when fifo is full
  input logic [31:0] tx_fifo,
  input mov_dst_t mov_dst,
  input mov_op_t mov_op,
  input logic config_status_sel,
  input set_dst_t set_dst,
  input logic [4:0] set_data,
  input logic [4:0] sideset_data,
  input logic sideset_en,
  input logic [2:0] config_sideset_base,
  input logic [2:0] config_sideset_cnt,

  output logic [7:0] out_pins,
  output logic [31:0] rx_fifo,
  output logic rx_full, tx_read,
  output logic [5:0] output_shift_count,
  output logic [31:0] X_scratch, Y_scratch
);

  logic [31:0] next_X_scratch, next_Y_scratch;

  logic [5:0] input_shift_count, next_input_shift_count, next_output_shift_count;

  // Fifos are on ozempic, combining them to make them bigger is not supported in this implementation
  // For tiny tapeout reasons, they're only 1 deep too
  // xtra_flexy_sr #(.DEPTH(1), .SIZE(32), .RESET_VAL(0)) tx_fifo(.clk(clk), .rst_n(rst_n), .serial_in(tx_fifo_in), .load_en(0), .shift_en(tx_fifo_shift_en), .shift_dir(1), .parallel_in(0), .serial_out(tx_fifo_out), .parallel_out());
  // xtra_flexy_sr #(.DEPTH(1), .SIZE(32), .RESET_VAL(0)) rx_fifo(.clk(clk), .rst_n(rst_n), .serial_in(rx_fifo_in), .load_en(0), .shift_en(rx_fifo_shift_en), .shift_dir(1), .parallel_in(0), .serial_out(rx_fifo_out), .parallel_out());

  // ISR/OSR
  logic [31:0] osr, next_osr, osr_out_data;
  logic [31:0] isr, next_isr, isr_in_data;

  logic [31:0] next_rx_fifo;
  logic next_rx_full, next_tx_read;

  logic [7:0] next_out_pins, next_pins;

  logic [31:0] mov_src_data, mov_data;

  // Scratch Registers
  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      X_scratch <= 0;
      Y_scratch <= 0;
    end else if (clk_en) begin
      X_scratch <= next_X_scratch;
      Y_scratch <= next_Y_scratch;
    end
  end

  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      isr <= 0;
      osr <= 0;
      input_shift_count <= 0;
      output_shift_count <= 6'd32;
      rx_fifo <= 0;
      out_pins <= 0;
    end else if (clk_en) begin
      isr <= next_isr;
      osr <= next_osr;
      input_shift_count <= next_input_shift_count;  // saturates at 32
      output_shift_count <= next_output_shift_count;
      rx_fifo <= next_rx_fifo;
      out_pins <= next_pins;
    end
  end

  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      rx_full <= 0;
    end else begin
      rx_full <= next_rx_full;
      tx_read <= clk_en ? next_tx_read : 0;
    end
  end

  always_comb begin
    next_X_scratch = X_scratch;
    next_Y_scratch = Y_scratch;
    next_isr = isr;
    next_osr = osr;
    next_input_shift_count = input_shift_count;
    next_output_shift_count = output_shift_count;
    next_out_pins = out_pins;  // "If no state machine writes to this GPIO's level or direction, the value does not change from the previous cycle."
    next_rx_fifo = rx_fifo;
    next_rx_full = rx_read ? 1'b0 : rx_full;
    next_tx_read = 1'b0;
    isr_in_data = 32'b0;
    osr_out_data = 32'b0;
    mov_src_data = 32'b0;
    mov_data = 32'b0;

    case (op)
      JMP: begin
        case (jmp_cond)
          3'b010: next_X_scratch = X_scratch - 1;
          3'b100: next_Y_scratch = Y_scratch - 1;
          default: ;
        endcase
      end

      IN: begin  // Shift data from srcs to isr
        case (src_sel_in_mov)
          SRC_PINS: isr_in_data = {24'b0, gpio_in};
          SRC_X: isr_in_data = X_scratch;
          SRC_Y: isr_in_data = Y_scratch;
          SRC_NULL: isr_in_data = 32'b0;
          SRC_ISR: isr_in_data = isr;
          SRC_OSR: isr_in_data = osr;
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
      end

      OUT: begin  // Shift data from osr to other dsts
        if (bit_cnt == 6'd32) begin
          osr_out_data = osr;
          next_osr = 0;
        end else if (config_osr_shift_dir) begin  // shift right
          osr_out_data = osr & ~(32'hFFFFFFFF << bit_cnt);
          next_osr = osr >> bit_cnt;
        end else begin
          osr_out_data = osr >> (6'd32 - bit_cnt);
          next_osr = osr << bit_cnt;
        end

        case (out_dst)
          OUT_DST_PINS: next_out_pins = osr_out_data[7:0];
          OUT_DST_X: next_X_scratch = osr_out_data;
          OUT_DST_Y: next_Y_scratch = osr_out_data;
          OUT_DST_NULL, OUT_DST_PINDIRS:;  // Discard
          OUT_DST_ISR: begin
            next_isr = osr_out_data;
            next_input_shift_count = bit_cnt;
          end
          default: ;  // No PC/EXEC
        endcase

        next_output_shift_count = (output_shift_count + bit_cnt > 6'd32) ? 6'd32 : output_shift_count + bit_cnt;
      end

      PUSH: begin
        if (!fifo_threshold || input_shift_count >= config_isr_push_thresh) begin  // should meet threshold
          // if full and blocking: wait it to unstall
          if (rx_full && !fifo_block) begin  // full and not blocking: clear isr
            next_isr = 0;
            next_input_shift_count = 0;
          end else if (!rx_full) begin  // Normal push, not full
            next_rx_full = 1;
            next_rx_fifo = isr;
            next_isr = 0;
            next_input_shift_count = 0;
          end
        end
      end

      PULL: begin
        if (!fifo_threshold || output_shift_count >= config_osr_pull_thresh) begin  // should meet threshold
          if (tx_empty) begin
            if (!fifo_block) begin
              // Empty and not blocking: copy X
              next_osr = X_scratch;
              next_output_shift_count = 0;
            end
            // otherwise, stall
          end else begin
            next_osr = tx_fifo;
            next_tx_read = 1;
            next_output_shift_count = 0;
          end
        end
      end

      // Removed instrs due to no fifos, also no irqs, Justin I wish you the best trying to write something for this

      MOV: begin
        case (src_sel_in_mov)
          SRC_PINS: mov_src_data = {24'b0, gpio_in};
          SRC_X: mov_src_data = X_scratch;
          SRC_Y: mov_src_data = Y_scratch;
          SRC_NULL: mov_src_data = 32'b0;
          SRC_STATUS: begin
            case (config_status_sel)  // 0: TX 1: RX
              1'b0: mov_src_data = tx_empty ? 32'h0 : 32'hFFFFFFFF;  // TX has data
              1'b1: mov_src_data = rx_full ? 32'hFFFFFFFF : 32'b0;  // RX has data
            endcase
          end
          SRC_ISR: mov_src_data = isr;
          SRC_OSR: mov_src_data = osr;
          default: mov_src_data = 32'b0;
        endcase

        case(mov_op)
          MOV_OP_NONE: mov_data = mov_src_data;
          MOV_OP_INVERT: mov_data = ~mov_src_data;
          MOV_OP_BITREV: begin
            for(int i=0; i<32; i++) begin
              mov_data[i] = mov_src_data[31-i];
          end
          end
          default: mov_data = mov_src_data;  // reserved stuff falls through
        endcase

        case (mov_dst)
          MOV_DST_PINS: next_out_pins = mov_data[7:0];
          MOV_DST_X: next_X_scratch = mov_data;
          MOV_DST_Y: next_Y_scratch = mov_data;
          MOV_DST_ISR: begin
            next_input_shift_count = 0;
            next_isr = mov_data;
          end
          MOV_DST_OSR: begin
            next_output_shift_count = 0;
            next_osr = mov_data;
          end
        default: ;  // NO PC/EXEC
        endcase
      end

      SET: begin
        case (set_dst)
          SET_DST_PINS: next_out_pins = {3'b0, set_data};
          SET_DST_X: next_X_scratch = {27'b0, set_data};
          SET_DST_Y: next_Y_scratch = {27'b0, set_data};
          default: ;  // Ignore PINDIRS and reserved stuff
        endcase
      end
    endcase
  end

  logic [7:0] sideset_mask;

  // Sideset
  assign sideset_mask = ((8'hFF >> (8 - config_sideset_cnt)) << config_sideset_base) & {8{sideset_en}};  // if config_sideset_cnt=0, en=0
  assign next_pins = (next_out_pins & ~sideset_mask) | (({3'b0, sideset_data} << config_sideset_base) & sideset_mask);

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
